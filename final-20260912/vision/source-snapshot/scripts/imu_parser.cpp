// ============================================================================
//  IMU 解析器  (IMU Parser)
//  ----------------------------------------------------------------------------
//  用途：这是一个独立的 IMU 数据解析程序。
//        用于解析通过 USB 串口(CH340)连接到电脑的 IMU 姿态数据，
//        该 IMU 使用 5A A5 帧头协议（维特智能系列常用帧头）。
//
//  串口协议（实测）:
//    - 波特率 : 115200, 8N1
//    - 帧率   : 100 Hz
//    - 帧长   : 82 字节, 帧头 0x5A 0xA5
//    - 数据   : float32(小端), 含 四元数(wxyz)、欧拉角(度)、加速度(g)、角速度
//
//  帧布局 (82 字节):
//    [0x00] 5A A5         帧头
//    [0x02] 4C 00         固定
//    [0x04] xx xx         CRC / 校验 (未验证)
//    [0x06] 91 01         固定
//    [0x08] 00 1A         固定
//    [0x0A] 00 00 00 00   固定
//    [0x0E] u32 LE        时间戳 (每帧 +10)
//    [0x12] float32       AccX (g)
//    [0x16] float32       AccY (g)
//    [0x1A] float32       AccZ (g)      (静止时 ≈ -1.0)
//    [0x1E] float32       温度或附加
//    [0x22] float32       GyroX
//    [0x26] float32       GyroY
//    [0x2A] float32       GyroZ
//    [0x2E] float32       保留
//    [0x32] float32       保留
//    [0x36] float32       Yaw   (deg)
//    [0x3A] float32       Roll  (deg)
//    [0x3E] float32       Pitch (deg)
//    [0x42] float32       四元数 w
//    [0x46] float32       四元数 x
//    [0x4A] float32       四元数 y
//    [0x4E] float32       四元数 z
//    [0x52] -------------- 帧结束 (共 82 字节)
//
//  编译:
//      g++ -O2 -o imu_parser imu_parser.cpp
//
//  用法:
//      ./imu_parser [设备] [波特率] [--raw] [--file <数据文件>]
//      ./imu_parser                          # 默认读 /dev/gimbal @115200
//      ./imu_parser /dev/ttyUSB1             # 指定串口
//      ./imu_parser /dev/gimbal 115200 --raw # 紧凑一行输出，便于脚本处理
//      ./imu_parser --file dump.bin          # 离线回放串口抓包验证
//
//  注意: 该程序完全独立于 sp_vision_25 项目源码，不依赖项目任何头文件/库。
// ============================================================================

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

// ---------------------------------------------------------------------------
// 帧头 / 帧长定义
// ---------------------------------------------------------------------------
static constexpr uint8_t FRAME_HDR0 = 0x5A;
static constexpr uint8_t FRAME_HDR1 = 0xA5;
static constexpr size_t  FRAME_LEN  = 82;

// 字段偏移
static constexpr size_t OFF_TS     = 0x0E;   // u32 LE 时间戳
static constexpr size_t OFF_ACC    = 0x12;   // 3 x float32 加速度
static constexpr size_t OFF_GYRO   = 0x22;   // 3 x float32 角速度
static constexpr size_t OFF_YAW    = 0x36;   // float32 度
static constexpr size_t OFF_ROLL   = 0x3A;   // float32 度
static constexpr size_t OFF_PITCH  = 0x3E;   // float32 度
static constexpr size_t OFF_QUAT   = 0x42;   // 4 x float32 wxyz

// ---------------------------------------------------------------------------
// 解析后的一帧 IMU 数据
// ---------------------------------------------------------------------------
struct ImuFrame {
  uint32_t ts = 0;
  float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
  float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;   // 度
  float acc[3] = {0.0f, 0.0f, 0.0f};             // g
  float gyro[3] = {0.0f, 0.0f, 0.0f};
  float quat_norm = 1.0f;                        // 四元数模长(检查用)
};

// 小端读 float32
static inline float rd_f32(const uint8_t *p) {
  float v;
  std::memcpy(&v, p, sizeof(float));
  return v;
}

// 小端读 uint32
static inline uint32_t rd_u32(const uint8_t *p) {
  uint32_t v;
  std::memcpy(&v, p, sizeof(uint32_t));
  return v;
}

// ---------------------------------------------------------------------------
// 解析一帧 (82 字节)。成功返回 true。
// ---------------------------------------------------------------------------
static bool parse_frame(const uint8_t *f, ImuFrame &out) {
  if (f[0] != FRAME_HDR0 || f[1] != FRAME_HDR1) return false;

  out.ts    = rd_u32(f + OFF_TS);
  out.yaw   = rd_f32(f + OFF_YAW);
  out.roll  = rd_f32(f + OFF_ROLL);
  out.pitch = rd_f32(f + OFF_PITCH);

  for (int i = 0; i < 4; i++) {
    out.quat[i] = rd_f32(f + OFF_QUAT + 4 * i);       // w, x, y, z
  }
  for (int i = 0; i < 3; i++) {
    out.acc[i]  = rd_f32(f + OFF_ACC + 4 * i);
    out.gyro[i] = rd_f32(f + OFF_GYRO + 4 * i);
  }

  const float *q = out.quat;
  out.quat_norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  return true;
}

// ---------------------------------------------------------------------------
// 串口封装 (termios, 无第三方依赖)
// ---------------------------------------------------------------------------
class SerialPort {
 public:
  SerialPort() : fd_(-1) {}
  ~SerialPort() { close(); }

  bool open(const std::string &dev, uint32_t baud) {
    fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      perror("open");
      return false;
    }
    // 恢复为阻塞(配合 VTIME 超时)
    int fl = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, fl & ~O_NONBLOCK);

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) { perror("tcgetattr"); close(); return false; }

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE);   // 8N1
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 20;                        // 2 秒读超时

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
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { perror("tcsetattr"); close(); return false; }

    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void close() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
  int fd() const { return fd_; }

 private:
  int fd_;
};

// 读取满 n 字节, 成功返回 true (超时返回 false)
static bool read_exact(int fd, uint8_t *buf, size_t n) {
  size_t got = 0;
  while (got < n) {
    ssize_t r = ::read(fd, buf + got, n - got);
    if (r > 0) { got += (size_t)r; continue; }
    if (r < 0 && errno == EINTR) continue;
    return false;  // 超时(0) 或错误
  }
  return true;
}

// ---------------------------------------------------------------------------
// 从串口流中同步并解析帧 (阻塞循环)
//   on_frame : 每解析成功一帧回调一次, 返回 false 可退出
// ---------------------------------------------------------------------------
static void run_serial(SerialPort &sp, bool raw_mode,
                       bool (*on_frame)(const ImuFrame &, void *), void *ctx) {
  int fd = sp.fd();
  std::vector<uint8_t> buf(FRAME_LEN);
  (void)raw_mode;
  uint8_t b;

  while (true) {
    // 1) 找帧头 (同步)
    while (true) {
      if (!read_exact(fd, &b, 1)) return;
      if (b == FRAME_HDR0) {
        if (!read_exact(fd, &b, 1)) return;
        if (b == FRAME_HDR1) break;      // 找到 5A A5
        if (b == FRAME_HDR0) { continue; } // 连续 5A，把当前当帧头候选
      }
    }
    // 2) 读剩余 80 字节
    buf[0] = FRAME_HDR0; buf[1] = FRAME_HDR1;
    if (!read_exact(fd, buf.data() + 2, FRAME_LEN - 2)) return;

    ImuFrame f;
    if (!parse_frame(buf.data(), f)) { continue; }
    if (!on_frame(f, ctx)) return;
  }
}

// ---------------------------------------------------------------------------
// 从抓包文件回放解析 (离线验证用)
// ---------------------------------------------------------------------------
static void run_file(const char *path, bool raw_mode) {
  FILE *fp = fopen(path, "rb");
  if (!fp) { perror("fopen"); return; }
  fseek(fp, 0, SEEK_END);
  long sz = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  std::vector<uint8_t> data(sz);
  if (fread(data.data(), 1, sz, fp) != (size_t)sz) { fclose(fp); return; }
  fclose(fp);

  size_t i = 0;
  while (i + FRAME_LEN <= data.size()) {
    if (data[i] == FRAME_HDR0 && data[i + 1] == FRAME_HDR1) {
      ImuFrame f;
      if (parse_frame(&data[i], f)) {
        if (raw_mode) {
          printf("%u %.6f %.6f %.6f %.6f %.3f %.3f %.3f %.4f %.4f %.4f\n",
                 f.ts, f.quat[0], f.quat[1], f.quat[2], f.quat[3],
                 f.yaw, f.pitch, f.roll, f.acc[0], f.acc[1], f.acc[2]);
        } else {
          printf("[%04zu] ts=%u  q=(w%.4f x%.4f y%.4f z%.4f)  Yaw=%8.2f  Pitch=%8.3f  Roll=%8.3f  Acc=(%.4f %.4f %.4f)  |q|=%.4f\n",
                 i / FRAME_LEN, f.ts,
                 f.quat[0], f.quat[1], f.quat[2], f.quat[3],
                 f.yaw, f.pitch, f.roll, f.acc[0], f.acc[1], f.acc[2], f.quat_norm);
        }
        i += FRAME_LEN;
        continue;
      }
    }
    i++;  // 重同步
  }
  printf("\n[回放完成] 共解析 %zu 帧\n", data.size() / FRAME_LEN);
}

// ---------------------------------------------------------------------------
// 串口实时输出回调: 打印 + 统计
// ---------------------------------------------------------------------------
static uint64_t s_count = 0;
static uint32_t s_last_ts = 0;
static uint32_t s_ts_drops = 0;
static uint64_t s_last_sec = 0;
static uint64_t s_last_count = 0;

static bool cb_print(const ImuFrame &f, void *ctx) {
  (void)ctx;
  s_count++;

  // 时间戳连续性检查(丢帧检测: 正常每帧 +10, 间隔 >20 视为丢帧)
  if (s_count > 1) {
    int32_t d = (int32_t)(f.ts - s_last_ts);
    if (d > 20) s_ts_drops++;
  }
  s_last_ts = f.ts;

  // 每秒向 stderr 打印一次统计
  uint64_t now = time(nullptr);
  if (now != s_last_sec) {
    if (s_count > 1) {
      fprintf(stderr, "\r[IMU 解析器] 累计 %llu 帧  |  %llu 帧/秒  |  丢帧 %llu",
              (unsigned long long)s_count,
              (unsigned long long)(s_count - s_last_count),
              (unsigned long long)s_ts_drops);
      fflush(stderr);
    }
    s_last_count = s_count;
    s_last_sec = now;
  }

  printf("%u %.6f %.6f %.6f %.6f %.3f %.3f %.3f %.4f %.4f %.4f\n",
         f.ts, f.quat[0], f.quat[1], f.quat[2], f.quat[3],
         f.yaw, f.pitch, f.roll, f.acc[0], f.acc[1], f.acc[2]);
  fflush(stdout);
  return true;
}

// ---------------------------------------------------------------------------
// 程序入口
// ---------------------------------------------------------------------------
int main(int argc, char *argv[]) {
  std::string dev = "/dev/gimbal";
  uint32_t baud = 115200;
  bool raw = false;
  std::string file;

  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--raw") { raw = true; }
    else if (a == "--file" && i + 1 < argc) { file = argv[++i]; }
    else if (a == "--help" || a == "-h") {
      printf("IMU 解析器 (5A A5 协议)\n"
             "用法: %s [设备] [波特率] [--raw] [--file <数据文件>]\n"
             "  默认设备 /dev/gimbal @ 115200\n"
             "  例: %s /dev/ttyUSB1 115200 --raw\n",
             argv[0], argv[0]);
      return 0;
    }
    else if (dev == "/dev/gimbal" && !isdigit(a[0])) { dev = a; }
    else if (isdigit(a[0])) { baud = (uint32_t)strtoul(a.c_str(), nullptr, 10); }
  }

  // banner 统一输出到 stderr：stdout 只留给数据行，便于管道/脚本消费
  fprintf(stderr, "================================================================\n");
  fprintf(stderr, " IMU 解析器 (IMU Parser)  — 5A A5 协议\n");
  fprintf(stderr, " 用途: 解析串口 IMU 姿态数据 (四元数/欧拉角/加速度)\n");
  if (file.empty()) {
    fprintf(stderr, " 串口设备: %s  波特率: %u\n", dev.c_str(), baud);
  } else {
    fprintf(stderr, " 回放文件: %s\n", file.c_str());
  }
  fprintf(stderr, "================================================================\n");

  if (!file.empty()) {
    run_file(file.c_str(), raw);
    return 0;
  }

  SerialPort sp;
  if (!sp.open(dev, baud)) {
    fprintf(stderr, "[IMU 解析器] 无法打开串口 %s。检查设备是否连接/上电，以及权限(dialout组)。\n",
            dev.c_str());
    return 1;
  }
  fprintf(stderr, "[IMU 解析器] 串口已打开，等待 IMU 数据... 按 Ctrl+C 退出\n");

  run_serial(sp, raw, cb_print, &raw);
  fprintf(stderr, "\n[IMU 解析器] 停止。共解析 %llu 帧\n", (unsigned long long)s_count);
  return 0;
}
