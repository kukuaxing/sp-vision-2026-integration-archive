#include "pitch_zero.h"
#include "can.h"
#include "task.h"
#include <cmath>
#include <cstring>

extern volatile uint32_t calibration_pitch_rx_count;
extern volatile uint32_t calibration_pitch_rx_tick;
volatile uint32_t pitch_zero_state = 0; // 0 init, 1 disabled check, 2 enabling, 3 holding, 4 fault
volatile uint32_t pitch_zero_fault = 0; // 1 feedback timeout, 2 motor fault, 3 outside return corridor
volatile float pitch_relative_rad = 0;
volatile float pitch_command_rad = PITCH_ZERO_RAD;

volatile uint32_t gimbal_remote_active = 0;
volatile uint32_t pitch_home_ready = 0;
volatile uint32_t yaw_manual_state = 0; // 0 off, 1 disabled check, 2 enabling, 3 ready, 4 fault
volatile uint32_t yaw_manual_fault = 0;
extern volatile uint32_t calibration_yaw_rx_count, calibration_yaw_rx_tick;

volatile uint32_t small_yaw_state = 0, small_yaw_fault = 0;
volatile float small_yaw_relative_rad = 0, small_yaw_target_rad = 0;
volatile int32_t small_yaw_output = 0;
extern volatile uint32_t small_yaw_rx_count, small_yaw_rx_tick;
namespace {
struct Remote { uint32_t tick; int16_t pitch, yaw, small_yaw; bool valid, manual; } remote{};
bool remote_armed = false;
uint32_t settled_since = 0;
float pitch_target = PITCH_ZERO_RAD;
void YawSpecial(uint8_t cmd) {
    uint8_t data[8] = {255,255,255,255,255,255,255,cmd};
    can2.Transmit(0x206, data, 8);
}
void YawSpeed(float speed) {
    uint8_t data[8] = {};
    memcpy(data, &speed, 4);
    can2.Transmit(0x206, data, 8);
}
void YawUpdate(bool active, int16_t stick, uint32_t now) {
    static uint32_t phase = 0, count = 0, last_stop = 0;
    static bool anchored = false;
    static float origin = 0;
    if (!active || yaw_manual_state == 4) {
        if ((yaw_manual_state != 0 && yaw_manual_state != 4) || now - last_stop >= 500) {
            YawSpeed(0); YawSpecial(0xFD); last_stop = now;
        }
        if (yaw_manual_state != 4) yaw_manual_state = 0;
        return;
    }
    if (yaw_manual_state == 0) {
        count = calibration_yaw_rx_count; phase = now;
        YawSpeed(0); YawSpecial(0xFD); yaw_manual_state = 1; return;
    }
    uint8_t raw[8]; uint32_t rx, tick;
    taskENTER_CRITICAL();
    rx = calibration_yaw_rx_count; tick = calibration_yaw_rx_tick;
    memcpy(raw, can2.DMmotor_data[0], 8);
    taskEXIT_CRITICAL();
    if (now - phase > 100 || int32_t(now - tick) > 100) {
        yaw_manual_fault = 1; yaw_manual_state = 4;
        YawSpeed(0); YawSpecial(0xFD); return;
    }
    if (rx == count) return;
    const uint8_t status = raw[0] >> 4;
    const float pos = ((uint16_t(raw[1]) << 8) | raw[2]) * (6.283186f / 65535.0f) - 3.141593f;
    if ((raw[0] & 15) != 6 || status >= 8 || (yaw_manual_state == 1 && status != 0) || (yaw_manual_state != 1 && status != 1)) {
        yaw_manual_fault = 2; yaw_manual_state = 4;
        YawSpeed(0); YawSpecial(0xFD); return;
    }
    count = rx; phase = now;
    if (!anchored) { origin = pos; anchored = true; }
    float offset = pos - origin;
    if (offset > 3.141593f) offset -= 6.283186f;
    if (offset < -3.141593f) offset += 6.283186f;
    if (fabsf(offset) > .20f) {
        yaw_manual_fault = 3; yaw_manual_state = 4;
        YawSpeed(0); YawSpecial(0xFD); return;
    }
    if (yaw_manual_state == 1) {
        YawSpeed(0); YawSpecial(0xFC); yaw_manual_state = 2; return;
    }
    yaw_manual_state = 3;
    float speed = -stick / 660.0f * .10f;
    if ((offset >= .18f && speed > 0) || (offset <= -.18f && speed < 0)) speed = 0;
    YawSpeed(speed);
}

void SmallYawSend(int32_t value) {
    small_yaw_output = value;
    const uint16_t bits = uint16_t(int16_t(value));
    uint8_t payload[8] = {uint8_t(bits >> 8), uint8_t(bits), 0,0,0,0,0,0};
    can1.Transmit(0x1FF, payload, 8); // Only GM6020 ID 0x205's command slot.
}
void SmallYawUpdate(bool active, int16_t stick, uint32_t now) {
    static bool anchored = false;
    static uint16_t previous_raw = 0;
    static float position = 0;
    static uint32_t previous_count = 0, previous_tick = 0;
    uint8_t data[8]; uint32_t count, tick;
    taskENTER_CRITICAL();
    count = small_yaw_rx_count; tick = small_yaw_rx_tick;
    memcpy(data, can1.data[4], 8);
    taskEXIT_CRITICAL();
    if (small_yaw_state == 4) { SmallYawSend(0); return; }
    if (!count || int32_t(now - tick) > 100) {
        if (anchored) { small_yaw_state = 4; small_yaw_fault = 1; }
        SmallYawSend(0); return;
    }
    const uint16_t raw = uint16_t(data[0]) << 8 | data[1];
    if (raw >= 8192 || data[6] >= 80) {
        small_yaw_state = 4; small_yaw_fault = 2; SmallYawSend(0); return;
    }
    if (!anchored) {
        if (!active) { SmallYawSend(0); return; }
        anchored = true; previous_raw = raw; position = 0;
        small_yaw_target_rad = 0; previous_tick = now; previous_count = count;
    }
    if (count != previous_count) {
        int delta = int(raw) - int(previous_raw);
        if (delta > 4096) delta -= 8192;
        if (delta < -4096) delta += 8192;
        position += delta * (6.283185307f / 8192.0f);
        previous_raw = raw; previous_count = count;
    }
    small_yaw_relative_rad = position;
    if (fabsf(position) > .20f) {
        small_yaw_state = 4; small_yaw_fault = 3; SmallYawSend(0); return;
    }
    const float dt = fminf((now - previous_tick) * .001f, .02f);
    previous_tick = now;
    if (!active) {
        small_yaw_state = 0; small_yaw_target_rad = position; SmallYawSend(0); return;
    }
    small_yaw_state = 3;
    small_yaw_target_rad = fmaxf(-.18f, fminf(.18f, small_yaw_target_rad - stick / 660.0f * .10f * dt));
    const int16_t rpm = int16_t(uint16_t(data[2]) << 8 | data[3]);
    const float velocity = rpm * .104719755f;
    // Encoder PD for slow commissioning; independent of unverified IMU alignment.
    const float effort = 18000.0f * (small_yaw_target_rad - position) - 600.0f * velocity;
    int32_t command = int32_t(fmaxf(-2500.0f, fminf(2500.0f, effort)));
    if ((position >= .18f && command > 0) || (position <= -.18f && command < 0)) command = 0;
    SmallYawSend(command);
}

uint32_t phase_tick = 0, phase_count = 0, command_tick = 0;
void Special(uint8_t command) {
    uint8_t data[8] = {255,255,255,255,255,255,255,command};
    can1.Transmit(0x109, data, 8);
}
void Fail(uint32_t reason) {
    Special(0xFD);
    pitch_zero_fault = reason;
    pitch_zero_state = 4;
    gimbal_remote_active = 0; remote_armed = false;
    SmallYawSend(0);
    YawSpeed(0); YawSpecial(0xFD);
}
}

void GimbalRemoteFrame(const uint8_t* f) {
    int ch[4] = {((f[0] | f[1] << 8) & 2047) - 1024,
        ((f[1] >> 3 | f[2] << 5) & 2047) - 1024,
        ((f[2] >> 6 | f[3] << 2 | f[4] << 10) & 2047) - 1024,
        ((f[4] >> 1 | f[5] << 7) & 2047) - 1024};
    const int left = (f[5] >> 6) & 3, right = (f[5] >> 4) & 3;
    bool valid = left != 0 && right != 0;
    for (int v : ch) valid = valid && v >= -660 && v <= 660;
    taskENTER_CRITICAL();
    remote = {HAL_GetTick(), int16_t(abs(ch[3]) <= 20 ? 0 : ch[3]),
        int16_t(abs(ch[0]) <= 20 ? 0 : ch[0]), int16_t(abs(ch[2]) <= 20 ? 0 : ch[2]), valid, left == 1 && right == 2};
    taskEXIT_CRITICAL();
}

void PitchZeroUpdate() {
    const uint32_t now = HAL_GetTick();
    Remote input;
    taskENTER_CRITICAL(); input = remote; taskEXIT_CRITICAL();
    const bool requested = input.valid && int32_t(now - input.tick) <= 100 && input.manual && pitch_home_ready && pitch_zero_state == 3;
    if (!requested) remote_armed = false;
    else if (!input.pitch && !input.yaw && !input.small_yaw) remote_armed = true;
    gimbal_remote_active = requested && remote_armed;
    YawUpdate(gimbal_remote_active, input.yaw, now);
    SmallYawUpdate(gimbal_remote_active, input.small_yaw, now);
    if (pitch_zero_state == 4) {
        // Read protection threshold for diagnosis; never clear faults or write parameters.
        static uint32_t last_query = 0;
        if (now - last_query >= 500) {
            const uint8_t query[8] = {9, 0, 0x33, 2, 0, 0, 0, 0};
            can1.Transmit(0x7FF, query, 8);
            last_query = now;
        }
        return;
    }
    if (pitch_zero_state == 0) {
        phase_count = calibration_pitch_rx_count;
        Special(0xFD); // Establish a fresh disabled-state response before enabling.
        phase_tick = now;
        pitch_zero_state = 1;
        return;
    }
    const uint32_t count = calibration_pitch_rx_count;
    if (count == phase_count || int32_t(now - calibration_pitch_rx_tick) > 100) {
        if (now - phase_tick > 500) Fail(1);
        return;
    }
    uint8_t raw[8];
    taskENTER_CRITICAL();
    memcpy(raw, can1.DMmotor_data[1], 8);
    taskEXIT_CRITICAL();
    if (count != calibration_pitch_rx_count) return;
    const uint8_t status = raw[0] >> 4;
    const uint16_t encoded = (uint16_t(raw[1]) << 8) | raw[2];
    const float position = encoded * (25.13274f / 65535.0f) - 12.56637f;
    pitch_relative_rad = position - PITCH_ZERO_RAD;
    if ((raw[0] & 15) != 9 || status >= 8) { Fail(2); return; }
    if (!std::isfinite(position) || std::fabs(pitch_relative_rad) > PITCH_RETURN_WINDOW) {
        Fail(3); return;
    }
    if (pitch_zero_state == 1) {
        if (status != 0) { Fail(2); return; }
        pitch_command_rad = position;
        phase_count = count;
        phase_tick = now;
        Special(0xFC);
        pitch_zero_state = 2;
        return;
    }
    if (status != 1) { Fail(2); return; }
    if (pitch_zero_state == 2) {
        pitch_zero_state = 3;
        command_tick = now;
    }
    const float dt = fminf((now - command_tick) * .001f, .02f);
    if (!pitch_home_ready) {
        pitch_target = PITCH_ZERO_RAD;
        if (fabsf(pitch_relative_rad) < .003f && fabsf(pitch_command_rad - PITCH_ZERO_RAD) < .001f) {
            if (!settled_since) settled_since = now;
            if (now - settled_since >= 100) pitch_home_ready = 1;
        } else settled_since = 0;
    } else if (gimbal_remote_active) {
        pitch_target -= input.pitch / 660.0f * PITCH_RETURN_SPEED * dt;
        pitch_target = fmaxf(PITCH_ZERO_RAD - .18f, fminf(PITCH_ZERO_RAD + .18f, pitch_target));
    } else {
        // Stop target integration immediately on mode exit, invalid frame or radio timeout.
        pitch_target = pitch_command_rad;
    }
    const float delta = pitch_target - pitch_command_rad;
    const float step = PITCH_RETURN_SPEED * dt;
    pitch_command_rad += fmaxf(-step, fminf(step, delta));
    command_tick = now;
    phase_tick = now;
    phase_count = count;
    const float command = pitch_command_rad;
    const float velocity = PITCH_RETURN_SPEED;
    uint8_t payload[8];
    memcpy(payload, &command, 4);
    memcpy(payload + 4, &velocity, 4);
    can1.Transmit(0x109, payload, 8);
}
