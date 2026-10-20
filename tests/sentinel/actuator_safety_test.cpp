#include "tests/sentinel/actuator_hal_stubs.hpp"
#include "firmware/sentry/STM32F405/motor.h"
#include "firmware/sentry/STM32F405/HTmotor.h"
#include "firmware/sentry/STM32F405/control.h"
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>

Motor can1_motor[CAN1_MOTOR_NUM] = {
    Motor(M3508, SPD, shooter, ID2, PID(10, 0, 1.5f)),
    Motor(M3508, SPD, shooter, ID3, PID(10, 0, 1.5f)),
    Motor(M2006, POS, supply, ID7, PID(10, .03f, .5f), PID(1, .01f, .02f)),
    Motor(M6020, POS, pantile, ID5, PID(200, .08f, 0), PID(5, 0, .5f, .5f))};
Motor can2_motor[CAN2_MOTOR_NUM] = {
    Motor(M3508, SPD, chassis, ID1, PID(1, 0, 0)),
    Motor(M3508, SPD, chassis, ID2, PID(1, 0, 0)),
    Motor(M3508, SPD, chassis, ID3, PID(1, 0, 0)),
    Motor(M3508, SPD, chassis, ID4, PID(1, 0, 0))};
DMMOTOR DMmotor[2] = {DMMOTOR(9, P_S, Pitch), DMMOTOR(6, SPEED, Yaw)};
CAN can1, can2;
CONTROL ctrl;
PARAMETER para;
IMU imu_pantile, imu_chassis;
DELAY delay;
struct Transmission { uint32_t id; std::array<uint8_t, 8> bytes; };
std::vector<Transmission> transmissions;
HAL_StatusTypeDef CAN::Transmit(uint32_t id, const uint8_t* bytes, uint8_t length) {
    assert(length <= 8); Transmission frame{id, {}};
    std::copy(bytes, bytes + length, frame.bytes.begin());
    transmissions.push_back(frame); return 0;
}

std::array<uint8_t, 8> dji(uint16_t angle, int16_t rpm, uint8_t temperature = 30) {
    return {uint8_t(angle >> 8), uint8_t(angle), uint8_t(uint16_t(rpm) >> 8),
            uint8_t(rpm), 0, 0, temperature, 0};
}
std::array<uint8_t, 8> dm(uint8_t id, float position, uint8_t state = 1) {
    const auto p = uint16_t((position - P_MIN) * 65535.f / (P_MAX - P_MIN));
    return {uint8_t((state << 4) | id), uint8_t(p >> 8), uint8_t(p), 0x80, 0, 0, 30, 30};
}
void update_motor(Motor& motor) { motor.Ontimer(can1.data, can1.temp_data); }
void feed_feedback(int16_t left = 6500, int16_t right = -6500, uint16_t encoder = 1000) {
    const auto l = dji(100, left), r = dji(200, right), f = dji(encoder, 0);
    assert(can1.ReceiveFeedback(0x202, l.data(), 8));
    assert(can1.ReceiveFeedback(0x203, r.data(), 8));
    assert(can1.ReceiveFeedback(0x207, f.data(), 8));
    for (int i = 0; i < 3; ++i) update_motor(can1_motor[i]);
}
void shooter_request(bool fire, bool wheels = true) {
    ctrl.shooter.auto_fire_request_level = fire;
    ctrl.shooter.auto_shoot = fire; ctrl.shooter.openRub = wheels; ctrl.shooter.Update();
}
int main() {
    para.bullet_step = 42130.29f; para.bullet_lead_max = 84260.58f;
    para.yaw_center = 8192; para.yaw_span = 1707;
    para.dm_pitch_min = -.55f; para.dm_pitch_max = .4f;
    ctrl.shooter_motor[0] = &can1_motor[0]; ctrl.shooter_motor[1] = &can1_motor[1];
    ctrl.supply_motor[0] = &can1_motor[2]; ctrl.pantile_motor[0] = &can1_motor[3];
    can1_motor[2].use_sum_angle = true;
    auto& feeder = can1_motor[2]; auto& yaw = can1_motor[3];

    // Only an actual validated arrival renews a feedback lease. Byte buffers
    // may stay nonzero forever, but polling cannot keep them alive.
    actuator_now_ms = 0;
    const auto yaw_frame = dji(500, 0);
    assert(!can1.ReceiveFeedback(0x205, yaw_frame.data(), 7));
    assert(!can1.ReceiveFeedback(0x205, yaw_frame.data(), 8, false));
    assert(!can2.ReceiveFeedback(0x205, yaw_frame.data(), 8)); // unregistered slot
    assert(!yaw.FeedbackFresh());
    assert(can1.ReceiveFeedback(0x205, yaw_frame.data(), 8));
    update_motor(yaw); assert(yaw.has_feedback && yaw.angle_latched && yaw.setangle == 500);
    yaw.setangle = 600; update_motor(yaw); assert(yaw.current != 0);
    actuator_now_ms = 101;
    for (int i = 0; i < 200; ++i) update_motor(yaw);
    assert(!yaw.FeedbackFresh() && !yaw.has_feedback && !yaw.angle_latched);
    assert(yaw.current == 0 && can1.temp_data[8] == 0 && can1.temp_data[9] == 0);
    const auto recovered_yaw = dji(700, 0);
    assert(can1.ReceiveFeedback(0x205, recovered_yaw.data(), 8));
    assert(!yaw.FeedbackFresh()); // new receive epoch must first be decoded
    update_motor(yaw);
    assert(yaw.setangle == 700 && yaw.current == 0); // no old 600 goal or PID debt
    yaw.setangle = 900;
    actuator_now_ms += 101; // task missed the entire outage; receive resumes first
    assert(can1.ReceiveFeedback(0x205, recovered_yaw.data(), 8));
    assert(!yaw.FeedbackFresh()); update_motor(yaw);
    assert(yaw.setangle == 700 && yaw.current == 0);

    // Startup: wait for a real enabled reply to FC. Never send FD merely
    // because the first response has not arrived, and status=0 is not enabled.
    ctrl.mode = CONTROL::STOP;
    transmissions.clear();
    for (int i = 0; i < 8; ++i) {
        actuator_now_ms += 100;
        DMmotor[1].DMmotorinit(can2); DMmotor[1].DMmotor_transmit(can2);
    }
    assert(transmissions.size() == 8);
    for (const auto& tx : transmissions) assert(tx.id == 6 && tx.bytes[7] == 0xfc);
    auto& pitch_motor = DMmotor[0];
    transmissions.clear();
    pitch_motor.DMmotor_transmit(can1); assert(transmissions.empty());
    pitch_motor.DMmotorinit(can1);
    assert(transmissions.size() == 1 && transmissions.back().bytes[7] == 0xfc);
    pitch_motor.DMmotor_transmit(can1); assert(transmissions.size() == 1);
    auto pitch = dm(9, .1f, 0);
    assert(!can1.ReceiveFeedback(0x100, pitch.data(), 6));
    auto bad_state = pitch; bad_state[0] = 0x39;
    assert(!can1.ReceiveFeedback(0x100, bad_state.data(), 8));
    assert(can1.ReceiveFeedback(0x100, pitch.data(), 8));
    pitch_motor.State_Decode(can2.jointidata);
    pitch_motor.DMmotor_Ontimer(10, .6f, can1.jointpdata[0]);
    assert(pitch_motor.decoded && !pitch_motor.DriveReady());
    pitch_motor.DMmotor_transmit(can1); assert(transmissions.size() == 1);
    pitch = dm(9, .1f, 1); assert(can1.ReceiveFeedback(0x100, pitch.data(), 8));
    pitch_motor.State_Decode(can2.jointidata);
    assert(pitch_motor.DriveReady() && std::fabs(pitch_motor.setPos - .1f) < .001f);
    pitch_motor.DMmotor_transmit(can1); assert(transmissions.size() == 1); // not encoded yet
    pitch_motor.DMmotor_Ontimer(10, .6f, can1.jointpdata[0]);
    pitch_motor.DMmotor_transmit(can1); assert(transmissions.back().id == 0x109);

    // Runtime timeout is latched OFF. Neither resumed status=1 nor periodic
    // startup FC calls, even in STOP, can revive the old position command.
    pitch_motor.setPos = .3f;
    actuator_now_ms += 101;
    for (int i = 0; i < 100; ++i) pitch_motor.State_Decode(can2.jointidata);
    assert(!pitch_motor.FeedbackFresh() && !pitch_motor.decoded);
    assert(pitch_motor.FaultLatched() && pitch_motor.safety_fault_reason == DMMOTOR::FEEDBACK_TIMEOUT);
    pitch_motor.DMmotor_transmit(can1);
    assert(transmissions.back().id == 9 && transmissions.back().bytes[7] == 0xfd);
    const auto stopped_transmissions = transmissions.size();
    for (int i = 0; i < 10; ++i) pitch_motor.DMmotorinit(can1);
    assert(transmissions.size() == stopped_transmissions);
    pitch = dm(9, -.2f); assert(can1.ReceiveFeedback(0x100, pitch.data(), 8));
    pitch_motor.State_Decode(can2.jointidata);
    assert(std::fabs(pitch_motor.setPos + .2f) < .001f && !pitch_motor.DriveReady());
    pitch_motor.DMmotor_Ontimer(10, .6f, can1.jointpdata[0]); pitch_motor.DMmotor_transmit(can1);
    assert(transmissions.back().bytes[7] == 0xfd);

    // A fresh object models the DM safety state after a MAIN MCU restart.
    // The physical STOP/RESET position is required again before FC is issued.
    pitch_motor = DMMOTOR(9, P_S, Pitch);
    ctrl.mode = CONTROL::AUTO; transmissions.clear();
    pitch_motor.DMmotorinit(can1); pitch_motor.DMmotor_transmit(can1);
    assert(transmissions.empty());
    ctrl.mode = CONTROL::STOP; pitch_motor.DMmotorinit(can1);
    assert(transmissions.size() == 1 && transmissions.back().bytes[7] == 0xfc);
    assert(can1.ReceiveFeedback(0x100, pitch.data(), 8)); pitch_motor.State_Decode(can2.jointidata);
    assert(pitch_motor.DriveReady() && std::fabs(pitch_motor.setPos + .2f) < .001f);
    pitch_motor.DMmotor_Ontimer(10, .6f, can1.jointpdata[0]); pitch_motor.DMmotor_transmit(can1);
    float commanded_position; std::memcpy(&commanded_position, transmissions.back().bytes.data(), 4);
    assert(transmissions.back().id == 0x109 && std::fabs(commanded_position + .2f) < .001f);

    // A real disabled report after operation is a distinct, readable latch.
    auto disabled = dm(9, -.2f, 0);
    assert(can1.ReceiveFeedback(0x100, disabled.data(), 8)); pitch_motor.State_Decode(can2.jointidata);
    assert(pitch_motor.FaultLatched() && pitch_motor.safety_fault_reason == DMMOTOR::DRIVER_DISABLED);
    const auto before_reenable = transmissions.size(); pitch_motor.DMmotorinit(can1);
    assert(transmissions.size() == before_reenable);
    pitch_motor.DMmotor_transmit(can1); assert(transmissions.back().bytes[7] == 0xfd);

    // Driver faults also require restart; a subsequent healthy frame is not
    // permission to re-enable. Test with the real FC/FD/transmit path.
    pitch_motor = DMMOTOR(9, P_S, Pitch); pitch_motor.DMmotorinit(can1);
    assert(can1.ReceiveFeedback(0x100, pitch.data(), 8)); pitch_motor.State_Decode(can2.jointidata);
    assert(pitch_motor.DriveReady());
    auto fault = dm(9, -.2f, 8); assert(can1.ReceiveFeedback(0x100, fault.data(), 8));
    pitch_motor.State_Decode(can2.jointidata); pitch_motor.DMmotor_transmit(can1);
    assert(pitch_motor.safety_fault_reason == DMMOTOR::DRIVER_FAULT && transmissions.back().bytes[7] == 0xfd);
    pitch_motor.setPos = .3f;
    assert(can1.ReceiveFeedback(0x100, pitch.data(), 8)); pitch_motor.State_Decode(can2.jointidata);
    assert(std::fabs(pitch_motor.setPos + .2f) < .001f && !pitch_motor.DriveReady());
    const auto fault_stop_count = transmissions.size(); pitch_motor.DMmotorinit(can1);
    assert(transmissions.size() == fault_stop_count);

    ctrl.mode = CONTROL::AUTO;
    // Initial wheel spin-up is allowed to wait with a high request: it must
    // not deadlock before the first shot when the vision side has no wheel RPM.
    feed_feedback(3000, -3000); shooter_request(true);
    assert(feeder.setangle == float(feeder.sum_angle) && !ctrl.shooter.feed_rearm_required);
    feed_feedback(); shooter_request(true);
    assert(feeder.setangle < float(feeder.sum_angle));
    const float queued = feeder.setangle;
    shooter_request(false); assert(feeder.setangle == queued); // complete normal one-shot
    update_motor(feeder); assert(feeder.current != 0);

    // STOP cancels a real queued position and controller debt, not just a flag.
    ctrl.mode = CONTROL::STOP; shooter_request(false, false); update_motor(feeder);
    assert(feeder.setangle == float(feeder.sum_angle) && feeder.current == 0);

    // Both wheels must independently meet signed RPM and freshness conditions.
    ctrl.mode = CONTROL::AUTO; feed_feedback(6500, -4000); shooter_request(true);
    assert(feeder.setangle == float(feeder.sum_angle)); // old average rule admitted 5250
    feed_feedback(6500, 6500); shooter_request(true);
    assert(feeder.setangle == float(feeder.sum_angle)); // reverse wheel rejected
    shooter_request(false); feed_feedback(); shooter_request(true);
    assert(feeder.setangle < float(feeder.sum_angle));
    // Loss after ready cancels debt. A still-high old request cannot revive it.
    feed_feedback(6500, -1000); shooter_request(true); update_motor(feeder);
    assert(feeder.setangle == float(feeder.sum_angle) && feeder.current == 0);
    feed_feedback(); shooter_request(true);
    assert(ctrl.shooter.feed_rearm_required && feeder.setangle == float(feeder.sum_angle));
    shooter_request(false); shooter_request(true);
    assert(feeder.setangle < float(feeder.sum_angle));

    // This is the actual cancellation method used by STOP/Aim lease/axis
    // feedback revocation in RC, with the same re-arm requirement.
    ctrl.shooter.CancelPendingFeed();
    ctrl.shooter.auto_shoot = false; // local lease/axis gate is NOT a new host low edge
    ctrl.shooter.Update();
    assert(ctrl.shooter.feed_rearm_required);
    shooter_request(true); update_motor(feeder);
    assert(feeder.setangle == float(feeder.sum_angle) && feeder.current == 0);
    shooter_request(false); shooter_request(true);
    assert(feeder.setangle < float(feeder.sum_angle));
    actuator_now_ms += 101; shooter_request(true); update_motor(feeder);
    assert(feeder.current == 0 && !feeder.FeedbackFresh());
    feed_feedback(); shooter_request(true);
    assert(feeder.setangle == float(feeder.sum_angle));
    shooter_request(false); shooter_request(true);
    assert(feeder.setangle < float(feeder.sum_angle));

    // Real field fault: stationary right wheel at 72--75 C retained +7372
    // command units even with a zero target. OFF and thermal cut must zero the
    // CAN bytes and PID debt, rather than regulate zero RPM with residual drive.
    shooter_request(false, false);
    feed_feedback(0, 0);
    auto& right = can1_motor[1];
    right.current = 7372; right.setspeed = 0;
    right.pid[speed].m_error[0] = 5;
    right.pid[speed].m_error[1] = -10;
    right.pid[speed].m_error[2] = 20;
    can1.temp_data[4] = 0x1c; can1.temp_data[5] = 0xcc;
    update_motor(right);
    assert(right.current == 0 && right.setspeed == 0);
    assert(can1.temp_data[4] == 0 && can1.temp_data[5] == 0);
    for (float error : right.pid[speed].m_error) assert(error == 0);
    // Heat cut remains effective while ControlTask renews its spin request.
    const auto hot = dji(200, 0, 75);
    for (int repeat = 0; repeat < 20; ++repeat) {
      assert(can1.ReceiveFeedback(0x203, hot.data(), 8));
      right.current = 7372; right.setspeed = -6500;
      update_motor(right);
      assert(right.temperature == 75 && right.current == 0 && right.setspeed == 0);
      assert(can1.temp_data[4] == 0 && can1.temp_data[5] == 0);
    }
    // Stopping a still-coasting wheel must not synthesize a braking pulse.
    const auto coast = dji(220, -3000, 30);
    assert(can1.ReceiveFeedback(0x203, coast.data(), 8));
    right.setspeed = 0; right.current = 5000; update_motor(right);
    assert(right.curspeed == -3000 && right.current == 0 && right.has_feedback);
    assert(can1.temp_data[4] == 0 && can1.temp_data[5] == 0);

    // Wraparound does not instantly expire or permanently renew an old frame.
    actuator_now_ms = 0xfffffff0u;
    assert(can1.ReceiveFeedback(0x205, yaw_frame.data(), 8));
    update_motor(yaw);
    actuator_now_ms = 0x20u; assert(yaw.FeedbackFresh());
    actuator_now_ms = 0x80u; assert(!yaw.FeedbackFresh());
    std::cout << "PASS actual CAN routing + Motor/PID + DM + SHOOTER: stale/identity/stop/debt/wheel/recovery guards\n";
}
