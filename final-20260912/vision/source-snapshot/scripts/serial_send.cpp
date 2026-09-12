#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

int main() {
    // 1. 打开串口（根据实际情况修改）
    const char* port = "/dev/ttyUSB2";
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1) {
        perror("Failed to open serial port");
        return 1;
    }

    std::cout << "Serial port opened successfully: " << port << std::endl;

    // 2. 配置串口
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr failed");
        close(fd);
        return 1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);   // 本地连接，启用接收
    tty.c_cflag &= ~PARENB;            // 无校验
    tty.c_cflag &= ~CSTOPB;            // 1 个停止位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                // 8 位数据位

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始模式
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // 关闭软件流控
    tty.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &tty);

    // 3. 要发送的数据
    unsigned char cmd[] = {
        0xA5, 0x01, 0x00, 0x00, 0x68,
        0x01, 0x0F, 0x01, 0x8B, 0xBD
    };

    // std::cout << "Waiting 60 seconds before sending..." << std::endl;
    // sleep(60);  // 等待 60 秒

    // 4. 每 2 秒发送一次，共 5 次
    for (int i = 1; i <= 5; ++i) {
        ssize_t bytes_written = write(fd, cmd, sizeof(cmd));

        if (bytes_written == sizeof(cmd)) {
            std::cout << "Send " << i << "/5 success, bytes: "
                      << bytes_written << std::endl;
        } else {
            std::cout << "Send " << i << "/5 failed" << std::endl;
        }

        if (i < 5) {
            sleep(2);  // 等待 2 秒
        }
    }

    // 5. 关闭串口
    close(fd);
    std::cout << "Finished sending, serial port closed" << std::endl;

    return 0;
}
//ls /dev/tty*
//rm serial_send
//g++ -o serial_send serial_send.cpp
//./serial_send