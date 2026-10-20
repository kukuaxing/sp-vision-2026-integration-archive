#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cmath>

// HI91 wire format and CRC: hipnuc/products, c/hipnuc/hipnuc_dec.c.
// These are sensor axes, before any installation rotation.
struct ImuPacket {
    float acceleration[3], gyro[3], rpy[3], quaternion[4];
    uint32_t sensor_time_ms;
};

inline uint16_t ImuCrc(uint16_t crc, const uint8_t* data, size_t size) {
    for (size_t i=0; i<size; ++i) {
        crc ^= uint16_t(data[i]) << 8;
        for (int bit=0; bit<8; ++bit)
            crc = uint16_t((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
    return crc;
}

inline bool ParseImuPacket(const uint8_t* data, size_t size, ImuPacket& output) {
    if (size != 82 || data[0] != 0x5a || data[1] != 0xa5) return false;
    const uint16_t payload_size = uint16_t(data[2]) | (uint16_t(data[3]) << 8);
    if (payload_size != 76 || data[6] != 0x91) return false;
    const uint16_t expected = uint16_t(data[4]) | (uint16_t(data[5]) << 8);
    if (ImuCrc(ImuCrc(0, data, 4), data+6, payload_size) != expected) return false;
    ImuPacket candidate{};
    std::memcpy(&candidate.sensor_time_ms, data+14, 4);
    std::memcpy(candidate.acceleration, data+18, 12);
    std::memcpy(candidate.gyro, data+30, 12);
    std::memcpy(candidate.rpy, data+54, 12);
    std::memcpy(candidate.quaternion, data+66, 16);
    for (float value : candidate.acceleration) if (!std::isfinite(value)) return false;
    for (float value : candidate.gyro) if (!std::isfinite(value)) return false;
    for (float value : candidate.rpy) if (!std::isfinite(value)) return false;
    float norm2=0;
    for (float value : candidate.quaternion) {
        if (!std::isfinite(value)) return false;
        norm2 += value*value;
    }
    if (norm2 < 0.98f || norm2 > 1.02f) return false;
    output=candidate;
    return true;
}
