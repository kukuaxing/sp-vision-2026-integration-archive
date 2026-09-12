// ============================================================================
//  IMU → CAN 转发器  (IMU to CAN Bridge)
//  ----------------------------------------------------------------------------
//  用途：这是一个独立的 IMU 数据转发程序。
//        把通过 USB 串口(CH340)直连上位机的 IMU（5A A5 协议，115200）
//        姿态数据，转换为项目标定工具能识别的 CAN 四元数帧，发送到
//        虚拟 CAN 接口 can0。
//
//  背景：机器构型为「IMU 直接连接上位机」。
//        项目标定工具 capture 通过 io::CBoard(SocketCAN) 读取四元数，
//        而本机没有真实 CAN 硬件。本项目利用内核 vcan 虚拟 CAN 接口，
//        在项目源代码之外搭建一个桥：
//             IMU 串口 → (本程序) → vcan0(can0) → capture
//
//  CAN 帧协议（与项目 cboard.cpp 解析对称）:
//    帧 ID   : 0x150 (可配置)
//    8 字节  : data[0..1]=Yaw   float16 大端
//              data[2..3]=Pitch float16 大端
//              data[4..5]=Roll  float16 大端
//              data[6]  = 打包: 高2位颜色, bit4-5 模式, 低4位 IMU计数(0-15)
//              data[7]  = 子弹速度(项目内固定, 填 0)
//    角度单位: 弧度 (rad)
//
//  依赖: 内核 vcan 模块。先创建虚拟接口:
//      sudo ip link add can0 type vcan
//      sudo ip link set can0 up
//
//  编译:
//      g++ -O2 -o imu_to_can imu_to_can.cpp
//
//  用法:
//      ./imu_to_can [串口设备] [波特率] [CAN接口] [CAN ID(hex)]
//      例: ./imu_to_can /dev/gimbal 115200 can0 0x150
//
//  注意: 本程序完全独立于 sp_vision_25 项目源码，不依赖项目任何头文件/库。
// ============================================================================

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

// ---- 串口侧: 5A A5 协议帧定义 (与 imu_parser.cpp 相同) -------------------
static constexpr uint8_t FRAME_HDR0 = 0x5A;
static constexpr uint8_t FRAME_HDR1 = 0xA5;
static constexpr size_t  FRAME_LEN  = 82;
static constexpr size_t  OFF_TS     = 0x0E;   // u32 LE, normally +10 per frame
static constexpr size_t  OFF_QUAT   = 0x42;   // 4 x float32 wxyz

static inline float rd_f32(const uint8_t *p) {
  float v;
  std::memcpy(&v, p, sizeof(float));
  return v;
}

static inline uint32_t rd_u32(const uint8_t *p) {
  uint32_t v;
  std::memcpy(&v, p, sizeof(uint32_t));
  return v;
}

// ---- f32 -> f16 (IEEE 754 half, 标准 round-to-nearest) --------------------
static uint16_t f32_to_f16(float x) {
  uint32_t f;
  std::memcpy(&f, &x, sizeof(uint32_t));
  uint16_t s   = (uint16_t)((f >> 16) & 0x8000);
  int32_t  e   = (int32_t)((f >> 23) & 0xFF);
  uint32_t m   = f & 0x7FFFFF;
  if (e == 255) {                         // Inf / NaN
    return (uint16_t)(s | 0x7C00 | (m ? 0x0200 : 0));
  }
  int32_t ne = e - 127 + 15;
  if (ne >= 31) return (uint16_t)(s | 0x7C00);   // 溢出 -> Inf
  if (ne <= 0) {                          // 次正规 / 下溢
    if (ne < -10) return s;
    m |= 0x800000;
    uint32_t shift = (uint32_t)(14 - ne);
    uint16_t h = (uint16_t)(s | (m >> shift));
    uint32_t rem = m & ((1u << shift) - 1);
    uint16_t half = (uint16_t)(1u << (shift - 1));
    if (rem > half || (rem == half && (h & 1))) h++;
    return h;
  }
  uint16_t h = (uint16_t)(s | (ne << 10) | (m >> 13));
  if (m & 0x1000) h++;                    // round-to-nearest
  return h;
}

// ---- 四元数 -> ZYX 欧拉角(rad), 与 CBoard 的 Rz*Ry*Rx 约定对称 -------------
static void quat_to_zyx(double w, double x, double y, double z,
                        double &yaw, double &pitch, double &roll) {
  double sinr = 2.0 * (w * x + y * z);
  double cosr = 1.0 - 2.0 * (x * x + y * y);
  roll = std::atan2(sinr, cosr);

  double sinp = std::fmax(-1.0, std::fmin(1.0, 2.0 * (w * y - z * x)));
  pitch = std::asin(sinp);

  double siny = 2.0 * (w * z + x * y);
  double cosy = 1.0 - 2.0 * (y * y + z * z);
  yaw = std::atan2(siny, cosy);
}

// ---- 串口封装 (termios) ----------------------------------------------------
static int open_serial(const std::string &dev, uint32_t baud) {
  int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) { perror("open"); return -1; }
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK);

  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) { perror("tcgetattr"); ::close(fd); return -1; }
  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 20;

  speed_t spd;
  switch (baud) {
    case 9600:   spd = B9600;   break;
    case 19200:  spd = B19200;  break;
    case 38400:  spd = B38400;  break;
    case 57600:  spd = B57600;  break;
    case 115200: spd = B115200; break;
    case 230400: spd = B230400; break;
    case 460800: spd = B460800; break;
    case 500000: spd = B500000; break;
    case 921600: spd = B921600; break;
    default:     spd = B115200; break;
  }
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);
  if (tcsetattr(fd, TCSANOW, &tty) != 0) { perror("tcsetattr"); ::close(fd); return -1; }
  tcflush(fd, TCIOFLUSH);
  return fd;
}

static bool read_exact(int fd, uint8_t *buf, size_t n) {
  size_t got = 0;
  while (got < n) {
    ssize_t r = ::read(fd, buf + got, n - got);
    if (r > 0) { got += (size_t)r; continue; }
    if (r < 0 && errno == EINTR) continue;
    return false;
  }
  return true;
}

// ---- CAN 侧 ----------------------------------------------------------------
static int open_can(const std::string &ifname) {
  int s = socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) { perror("socket CAN"); return -1; }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
    perror("ioctl SIOCGIFINDEX");
    ::close(s);
    return -1;
  }
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("bind CAN");
    ::close(s);
    return -1;
  }
  return s;
}

// ---------------------------------------------------------------------------
int main(int argc, char *argv[]) {
  std::string dev   = (argc > 1) ? argv[1] : "/dev/gimbal";
  uint32_t    baud  = (argc > 2) ? (uint32_t)strtoul(argv[2], nullptr, 10) : 115200;
  std::string canif = (argc > 3) ? argv[3] : "can0";
  uint32_t    canid = (argc > 4) ? (uint32_t)strtoul(argv[4], nullptr, 0) : 0x150;

  fprintf(stderr, "================================================================\n");
  fprintf(stderr, " IMU → CAN 转发器 (IMU to CAN Bridge) — 5A A5 协议\n");
  fprintf(stderr, " 用途: 把 IMU 串口姿态转为 CAN 四元数帧发送到 %s\n", canif.c_str());
  fprintf(stderr, " 串口: %s @ %u   CAN ID: 0x%03X\n", dev.c_str(), baud, canid);
  fprintf(stderr, "================================================================\n");

  int canfd = open_can(canif);
  if (canfd < 0) {
    fprintf(stderr,
            "[转发器] 无法打开 CAN 接口 %s。\n"
            "  请先创建虚拟接口(需 sudo):\n"
            "    sudo ip link add %s type vcan\n"
            "    sudo ip link set %s up\n",
            canif.c_str(), canif.c_str(), canif.c_str());
    return 1;
  }

  int fd = open_serial(dev, baud);
  if (fd < 0) {
    fprintf(stderr, "[转发器] 无法打开串口 %s。检查设备连接/上电/权限。\n", dev.c_str());
    ::close(canfd);
    return 1;
  }
  fprintf(stderr, "[转发器] 串口已打开，开始转发... (Ctrl+C 退出)\n");

  // 主循环: 帧同步 + 解析 + 发送
  std::vector<uint8_t> buf(FRAME_LEN);
  uint8_t b;
  uint64_t sent = 0;
  uint8_t imu_count = 0;
  uint64_t last_log = 0;

  uint32_t last_ts = 0;
  double last_q[4] = {1.0, 0.0, 0.0, 0.0};
  bool have_last_q = false;
  unsigned int stable_frames = 0;
  static constexpr unsigned int WARMUP_FRAMES = 20;

  while (true) {
    // 1) 找帧头 5A A5
    while (true) {
      if (!read_exact(fd, &b, 1)) { ::close(fd); ::close(canfd); return 1; }
      if (b == FRAME_HDR0) {
        if (!read_exact(fd, &b, 1)) { ::close(fd); ::close(canfd); return 1; }
        if (b == FRAME_HDR1) break;
        if (b == FRAME_HDR0) continue;
      }
    }
    // 2) 读剩余 80 字节
    buf[0] = FRAME_HDR0; buf[1] = FRAME_HDR1;
    if (!read_exact(fd, buf.data() + 2, FRAME_LEN - 2)) {
      ::close(fd); ::close(canfd); return 1;
    }

    // 3) 固定字段检查，避免把数据区中的 5A A5 误判成帧头。
    // buf[9] 是运行状态字段，实测会出现 0x1A、0x1B、0x1E，不能固定限制。
    if (buf[2] != 0x4C || buf[3] != 0x00 ||
        buf[6] != 0x91 || buf[7] != 0x01 ||
        buf[8] != 0x00) {
      stable_frames = 0;
      have_last_q = false;
      continue;
    }

    uint32_t ts = rd_u32(buf.data() + OFF_TS);

    // 4) 解析并校验四元数 wxyz。
    double q[4];
    double norm_sq = 0.0;
    bool finite = true;
    for (int i = 0; i < 4; i++) {
      q[i] = rd_f32(buf.data() + OFF_QUAT + 4 * i);
      finite = finite && std::isfinite(q[i]);
      norm_sq += q[i] * q[i];
    }

    if (!finite || norm_sq < 0.64 || norm_sq > 1.44) {
      stable_frames = 0;
      have_last_q = false;
      continue;
    }

    double norm = std::sqrt(norm_sq);
    for (double &value : q) value /= norm;

    bool continuous = true;
    if (have_last_q) {
      uint32_t dt = ts - last_ts;
      double dot = std::fabs(
        q[0] * last_q[0] + q[1] * last_q[1] +
        q[2] * last_q[2] + q[3] * last_q[3]);

      continuous = dt > 0 && dt <= 100 && dot >= 0.95;
    }

    if (!continuous) stable_frames = 0;

    for (int i = 0; i < 4; i++) last_q[i] = q[i];
    last_ts = ts;
    have_last_q = true;

    if (stable_frames < WARMUP_FRAMES) ++stable_frames;
    if (stable_frames < WARMUP_FRAMES) continue;

    // 5) 四元数 -> ZYX 欧拉角(rad)
    double yaw, pitch, roll;
    quat_to_zyx(q[0], q[1], q[2], q[3], yaw, pitch, roll);

    // 5) f16 大端编码 + 组 CAN 帧
    uint16_t y16 = f32_to_f16((float)yaw);
    uint16_t p16 = f32_to_f16((float)pitch);
    uint16_t r16 = f32_to_f16((float)roll);

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id  = canid;
    frame.can_dlc = 8;
    frame.data[0] = (uint8_t)(y16 >> 8);   frame.data[1] = (uint8_t)(y16 & 0xFF);
    frame.data[2] = (uint8_t)(p16 >> 8);   frame.data[3] = (uint8_t)(p16 & 0xFF);
    frame.data[4] = (uint8_t)(r16 >> 8);   frame.data[5] = (uint8_t)(r16 & 0xFF);
    frame.data[6] = imu_count & 0x0F;       // 低4位 IMU 计数 0-15 循环
    frame.data[7] = 0;                      // 子弹速度(项目内固定)

    imu_count = (imu_count + 1) & 0x0F;

    if (write(canfd, &frame, sizeof(frame)) != (ssize_t)sizeof(frame)) {
      perror("write CAN");
      ::close(fd); ::close(canfd); return 1;
    }
    sent++;

    // 每秒统计
    uint64_t now = time(nullptr);
    if (now != last_log) {
      last_log = now;
      fprintf(stderr, "\r[转发器] 已发送 %llu 帧 | yaw=%.2f° pitch=%.2f° roll=%.2f° | q=(%.4f %.4f %.4f %.4f)  ",
              (unsigned long long)sent, yaw * 180.0 / M_PI, pitch * 180.0 / M_PI, roll * 180.0 / M_PI,
              q[0], q[1], q[2], q[3]);
      fflush(stderr);
    }
  }
  return 0;
}
