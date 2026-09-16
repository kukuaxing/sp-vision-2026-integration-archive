#include "../STM32F405/imu_packet.h"
#include <cassert>
#include <cstdio>
#include <limits>
#include <initializer_list>

static void checksum(uint8_t* p) {
    const auto crc=ImuCrc(ImuCrc(0,p,4),p+6,76);
    p[4]=crc&255; p[5]=crc>>8;
}
int main() {
    // CRC-valid snapshot from UART5, 2026-09-17. Includes nonzero roll,
    // pitch, yaw and real quaternion; catches the former swapped axes.
    const char* hex="5aa54c00a6b39123231c00000000d2894601ada1a8bcd75d163c75857f3f2908cf3cfc7c90bc1ef188be000000000000000000000000d0be8b3faceaed3ed5583742aab16b3ff3fce4372f9b293c7c52c73e";
    uint8_t bytes[82];
    for (size_t i=0;i<82;++i) { unsigned v; assert(std::sscanf(hex+2*i,"%2x",&v)==1); bytes[i]=v; }
    ImuPacket result{};
    assert(ParseImuPacket(bytes,82,result));
    assert(std::fabs(result.rpy[0]-1.0917606f)<1e-6);
    assert(std::fabs(result.rpy[1]-0.464681f)<1e-6);
    assert(std::fabs(result.rpy[2]-45.83675f)<1e-5);
    assert(result.sensor_time_ms==0x014689d2);
    for (size_t n=0;n<82;++n) assert(!ParseImuPacket(bytes,n,result));
    assert(!ParseImuPacket(bytes,83,result));
    for (size_t i=0;i<82;++i) {
        bytes[i]^=1; assert(!ParseImuPacket(bytes,82,result)); bytes[i]^=1;
    }
    uint8_t altered[82];
    std::memcpy(altered,bytes,82); altered[3]=1; checksum(altered);
    assert(!ParseImuPacket(altered,82,result));
    std::memcpy(altered,bytes,82); altered[6]=0x90; checksum(altered);
    assert(!ParseImuPacket(altered,82,result));
    for (size_t offset : {size_t(18),size_t(30),size_t(54),size_t(66)}) {
        std::memcpy(altered,bytes,82);
        float nan=std::numeric_limits<float>::quiet_NaN();
        std::memcpy(altered+offset,&nan,4); checksum(altered);
        assert(!ParseImuPacket(altered,82,result));
    }
    std::memcpy(altered,bytes,82); std::memset(altered+66,0,16); checksum(altered);
    assert(!ParseImuPacket(altered,82,result));
    assert(std::fabs(result.rpy[2]-45.83675f)<1e-5); // Failure leaves output intact.
    std::puts("IMU packet tests passed: real frame, axes, CRC, truncation, lengths, NaN and quaternion");
}
