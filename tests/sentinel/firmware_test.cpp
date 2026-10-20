#include "io/sentinel/protocol.hpp"
#include "firmware/sentry/STM32F405/xuc.cpp"
#include "firmware/sentry/STM32F405/CRC.cpp"
#include "sentinel_hold.inc" // Production clamp/manual/hold/update, with device stubs only.
#include "sentinel_apply_aim.inc" // Generated verbatim from production RC::ApplyAim.
#include "sentinel_remote.inc" // Actual remote-loss / non-AUTO revocation methods.
#include <cassert>
#include <iostream>
using namespace io::sentinel;
void deliver(UART& port,const Bytes& bytes){port.rx=bytes;xuc.Decode();}
int main(){
  UART port;xuc.Init(&port,nullptr,921600);
  imu_chassis.arrival=998000;port.arrival=12; // host arrival must NOT appear in IMU telemetry.
  xuc.Encode();assert(port.tx.size()==106);
  std::vector<Frame> telemetry;Decoder d;d.feed(port.tx.data(),port.tx.size(),1,[&](const Frame& f){telemetry.push_back(f);});
  assert(telemetry.size()==3);
  assert(u32(telemetry[0].payload.data()+4)==imu_pantile.arrival);
  assert(u32(telemetry[1].payload.data()+4)==imu_chassis.arrival);
  // Merely entering AUTO must not spin wheels before an accepted host lease.
  RC standby;standby.last_rc_tick=xTaskGetTickCount();standby.OnRC();standby.OnPC();
  assert(!ctrl.shooter.openRub && !ctrl.shooter.auto_shoot);
  assert(!(xuc.LinkState() & XUC_ST_FRICTION_ALLOW));
  // Pending targets must be discarded using decoded feedback, including a pose
  // outside the usual soft limits. Missing feedback must never lock to zero.
  Motor halted;halted.has_feedback=false;halted.angle_latched=false;halted.setangle=1700;
  ctrl.pantile_motor[CONTROL::PANTILE::YAW]=&halted;
  ctrl.pantile.hold_position=false;
  ctrl.pantile.mark_pitch=.3f;DMmotor[0].setPos=.3f;
  ctrl.pantile.Hold_Pantile();ctrl.pantile.Update();
  assert(halted.setangle==1700 && ctrl.pantile.mark_pitch==.3f && !ctrl.pantile.pitch_ready);
  halted.has_feedback=true;halted.angle[now]=0; // has_feedback alone cannot establish a latched pose
  ctrl.pantile.Hold_Pantile();assert(halted.setangle==1700 && !ctrl.pantile.hold_yaw_captured);
  halted.angle_latched=true;halted.angle[now]=8192;
  ctrl.pantile.Hold_Pantile();assert(halted.setangle==1700 && !ctrl.pantile.hold_yaw_captured);
  halted.angle[now]=-1;ctrl.pantile.Hold_Pantile();assert(halted.setangle==1700);
  halted.angle[now]=NAN;ctrl.pantile.Hold_Pantile();assert(halted.setangle==1700);
  halted.angle[now]=2500;
  ctrl.pantile.Hold_Pantile();ctrl.pantile.Update();
  assert(halted.setangle==2500 && ctrl.pantile.hold_yaw_captured && !ctrl.pantile.hold_pitch_captured);
  // Late pitch/large-yaw feedback must not cause an already latched axis to
  // acquire a second target while this STOP/hold session is still active.
  halted.angle[now]=2450;
  DMmotor[0].decoded=true;DMmotor[0].status=1;DMmotor[0].pos=-.6f;
  DMmotor[1].decoded=true;DMmotor[1].status=1;DMmotor[1].pos=.2f;
  ctrl.mode=CONTROL::STOP;standby.requested_mode=CONTROL::STOP;
  standby.OnRC();ctrl.pantile.Update();
  assert(halted.setangle==2500 && ctrl.pantile.mark_pitch==-.6f && DMmotor[0].setPos==-.6f);
  assert(DMmotor[1].setSpeed==0 && ctrl.pantile.hold_position);
  // Repeated STOP retains the first measured position so the servo can restore
  // a disturbance or oppose gravity, instead of following the accumulating drift.
  halted.angle[now]=2400;DMmotor[0].pos=-.2f;DMmotor[1].pos=.3f;
  standby.OnRC();ctrl.pantile.Update();assert(halted.setangle==2500 && DMmotor[0].setPos==-.6f);
  assert(ctrl.pantile.yaw_lock_pos==.2f && DMmotor[1].setSpeed<0);
  ctrl.pantile.Control_Pantile(0,0,0); // Leave the preceding hold session.
  ctrl.mode=CONTROL::AUTO;standby.requested_mode=CONTROL::MANUAL;
  halted.setangle=1000;DMmotor[0].pos=-.1f;standby.OnRC();
  assert(halted.setangle==2400 && ctrl.pantile.mark_pitch==-.1f);
  ctrl.pantile.Control_Pantile(100,100,0);
  const auto manual_yaw=halted.setangle,manual_pitch=ctrl.pantile.mark_pitch;
  standby.OnRC();assert(halted.setangle==manual_yaw && ctrl.pantile.mark_pitch==manual_pitch);
  assert(!ctrl.pantile.hold_position);
  ctrl.pantile.Update();assert(halted.setangle==para.yaw_span); // manual limits resume
  // Fault/NaN DM feedback is not a position reference, nor may it initialize
  // pitch through the normal Update path.
  DMmotor[0].status=8;DMmotor[0].pos=0;ctrl.pantile.pitch_ready=false;
  ctrl.pantile.Hold_Pantile();ctrl.pantile.Update();
  assert(ctrl.pantile.mark_pitch==manual_pitch && !ctrl.pantile.pitch_ready);
  DMmotor[0].status=1;DMmotor[0].pos=NAN;
  ctrl.pantile.Hold_Pantile();ctrl.pantile.Update();assert(!ctrl.pantile.pitch_ready);
  DMmotor[0].pos=-.25f;ctrl.pantile.Hold_Pantile();ctrl.pantile.Update();
  assert(ctrl.pantile.pitch_ready && ctrl.pantile.hold_pitch_captured && DMmotor[0].setPos==-.25f);
  DMmotor[0].pos=-.2f;ctrl.pantile.Hold_Pantile();ctrl.pantile.Update();
  assert(DMmotor[0].setPos==-.25f); // Valid late feedback is only captured once.
  ctrl.pantile.Control_Pantile(0,0,0);ctrl.pantile.Hold_Pantile();ctrl.pantile.Update();
  assert(DMmotor[0].setPos==-.2f); // A new hold session captures the new pose.
  standby.requested_mode=CONTROL::RESET;standby.OnRC();assert(!ctrl.pantile.hold_position);
  halted.angle[now]=2380;DMmotor[0].pos=-.15f;
  standby.requested_mode=CONTROL::STOP;standby.OnRC();ctrl.pantile.Update();
  assert(halted.setangle==2380 && DMmotor[0].setPos==-.15f && ctrl.pantile.hold_position);
  ctrl.pantile={};DMmotor[0]={};DMmotor[1]={};
  ctrl.pantile_motor[CONTROL::PANTILE::YAW]=nullptr;
  ctrl.mode=CONTROL::AUTO;standby.requested_mode=CONTROL::AUTO;
  Imu ref;assert(decode_imu(telemetry[0],ref));
  const auto transmitted=port.tx;
  port.tx_idle=false;xuc.Encode();assert(port.tx==transmitted);port.tx_idle=true;
  Aim aim;aim.ref_epoch=ref.epoch;aim.ref_seq=ref.seq;aim.ref_time_us=ref.time_us;
  aim.yaw_error=.1f;aim.pitch_error=-.05f;aim.flags=15;
  auto invisible=aim;++invisible.ref_seq;
  deliver(port,encode_aim(invisible,65534,xuc.boot_id));assert(xuc.last_reject==XUC_BAD_REFERENCE);
  deliver(port,encode_aim(aim,65535,xuc.boot_id));
  assert(xuc.aim_accepted && std::abs(xuc.aim_yaw_target-(ref.yaw+.1f))<1e-6);
#if XUC_ALLOW_CONTROL && (!XUC_REQUIRE_VERIFIED_POSE || (XUC_SAMPLE_TIME_VERIFIED && XUC_AXES_VERIFIED))
  assert(xuc.AimReady() && xuc.last_reject==XUC_OK);
  Motor yaw;ctrl.pantile_motor[CONTROL::PANTILE::YAW]=&yaw;RC rc;
  yaw.angle[now]=300;DMmotor[0].decoded=true;DMmotor[0].status=1;DMmotor[0].pos=-.1f;
  rc.ApplyAim();ctrl.pantile.Update();
  assert(ctrl.shooter.openRub && ctrl.shooter.auto_shoot);
  assert(yaw.setangle==300 && ctrl.pantile.mark_pitch==-.1f); // takeover is measured pose
  const float yaw_lead_max=22.f,pitch_lead_max=.5f*PI/180.f;
  rc.ApplyAim();ctrl.pantile.Update();
  assert(std::abs(yaw.setangle-(300+yaw_lead_max))<1e-4f);
  assert(std::abs(ctrl.pantile.mark_pitch-(-.1f-pitch_lead_max))<1e-6f);
  // Same valid packet, frozen feedback, repeated ControlTask executions: no
  // position goal may walk away. This verifies reference algebra, not dynamics.
  for(int i=0;i<100;++i){
    rc.ApplyAim();ctrl.pantile.Update();
    assert(std::abs(yaw.setangle-yaw.angle[now])<=yaw_lead_max+1e-4f);
    assert(std::abs(ctrl.pantile.mark_pitch-DMmotor[0].pos)<=pitch_lead_max+1e-6f);
  }
  // A new packet (including sequence wrap) must not reset or accumulate a goal;
  // reversing/zeroing the IMU error immediately reverses/removes its bounded lead.
  uint16_t motion_seq=0;
  auto reversed=aim;reversed.yaw_error=-.1f;reversed.pitch_error=.05f;
  deliver(port,encode_aim(reversed,motion_seq++,xuc.boot_id));rc.ApplyAim();ctrl.pantile.Update();
  assert(xuc.last_reject==XUC_OK);
  assert(std::abs(yaw.setangle-(300-yaw_lead_max))<1e-4f);
  assert(std::abs(ctrl.pantile.mark_pitch-(-.1f+pitch_lead_max))<1e-6f);
  auto centered=aim;centered.yaw_error=centered.pitch_error=0;
  deliver(port,encode_aim(centered,motion_seq++,xuc.boot_id));rc.ApplyAim();ctrl.pantile.Update();
  assert(yaw.setangle==yaw.angle[now] && ctrl.pantile.mark_pitch==DMmotor[0].pos);
  auto crossing=aim;imu_pantile.yaw=179;
  crossing.yaw_error=XUC::WrapPi(-179.f*PI/180.f-ref.yaw);
  deliver(port,encode_aim(crossing,motion_seq++,xuc.boot_id));rc.ApplyAim();ctrl.pantile.Update();
  assert(yaw.setangle>yaw.angle[now] && yaw.setangle-yaw.angle[now]<=yaw_lead_max); // +2°, not -358°
  imu_pantile.yaw=20;
  deliver(port,encode_aim(aim,motion_seq++,xuc.boot_id));rc.ApplyAim();
  // A missing/faulted/nonfinite axis blocks new AUTO corrections on both axes.
  DMmotor[0].decoded=false;rc.ApplyAim();ctrl.pantile.Update();
  const auto blocked_pitch=ctrl.pantile.mark_pitch;
  assert(ctrl.pantile.hold_position && yaw.setangle==300 && !ctrl.shooter.auto_shoot);
  DMmotor[0].decoded=true;DMmotor[0].status=8;DMmotor[0].pos=.2f;
  rc.ApplyAim();ctrl.pantile.Update();assert(ctrl.pantile.mark_pitch==blocked_pitch);
  DMmotor[0].status=1;DMmotor[0].pos=NAN;
  rc.ApplyAim();ctrl.pantile.Update();assert(ctrl.pantile.mark_pitch==blocked_pitch);
  DMmotor[0].pos=-.1f;yaw.has_feedback=false;rc.ApplyAim();
  assert(ctrl.pantile.hold_position && !ctrl.shooter.auto_shoot);
  yaw.has_feedback=true;yaw.angle_latched=false;rc.ApplyAim();assert(ctrl.pantile.hold_position);
  yaw.angle_latched=true;yaw.angle[now]=NAN;rc.ApplyAim();assert(ctrl.pantile.hold_position);
  yaw.angle[now]=300;rc.ApplyAim();ctrl.pantile.Update();
  assert(yaw.setangle==300 && ctrl.pantile.mark_pitch==-.1f); // first valid feedback re-anchors
  rc.ApplyAim();assert(yaw.setangle>300 && ctrl.pantile.mark_pitch<-.1f);
  // Out-of-limit feedback is held without snapping to the nearest limit.
  yaw.angle[now]=2500;DMmotor[0].pos=-.6f;
  rc.ApplyAim();ctrl.pantile.Update();
  assert(ctrl.pantile.hold_position && yaw.setangle==2500 && DMmotor[0].setPos==-.6f);
  yaw.angle[now]=300;DMmotor[0].pos=-.1f;rc.ApplyAim();ctrl.pantile.Update();
  assert(yaw.setangle==300 && ctrl.pantile.mark_pitch==-.1f);
  rc.ApplyAim();ctrl.pantile.Control_Pantile(0,0,0); // manual handoff clears AUTO reference state
  yaw.angle[now]=350;DMmotor[0].pos=-.12f;rc.ApplyAim();ctrl.pantile.Update();
  assert(yaw.setangle==350 && ctrl.pantile.mark_pitch==-.12f);
  yaw.angle[now]=1234;DMmotor[0].decoded=true;DMmotor[0].status=1;DMmotor[0].pos=.12f;
  test_now_us=ref.time_us+100001;assert(!xuc.AimReady());rc.ApplyAim();
  assert(!ctrl.shooter.openRub && !ctrl.shooter.auto_shoot);
  assert(yaw.setangle==yaw.angle[now] && ctrl.pantile.hold_position);
  assert(ctrl.pantile.mark_pitch==.12f && DMmotor[0].setPos==.12f);
  yaw.angle[now]=1250;DMmotor[0].pos=.13f;rc.ApplyAim();
  assert(yaw.setangle==1234 && DMmotor[0].setPos==.12f); // Still one expired lease.
  test_now_us=1000000;rc.ApplyAim();ctrl.pantile.Update();assert(!ctrl.pantile.hold_position); // AIM resumes.
  assert(yaw.setangle==1250 && DMmotor[0].setPos==.13f); // first recovered lease is measured pose
  yaw.angle[now]=1300;DMmotor[0].pos=.15f;test_now_us=ref.time_us+100001;rc.ApplyAim();
  assert(yaw.setangle==1300 && DMmotor[0].setPos==.15f); // New revocation re-captures.
  test_now_us=1000000;
  // Keep receiving fresh host commands: physical STOP must still win.
  rc.last_rc_tick=xTaskGetTickCount();rc.requested_mode=CONTROL::STOP;
  rc.OnRC();rc.OnPC();
  assert(ctrl.mode==CONTROL::STOP && !xuc.AimReady() && !xuc.aim_control_en);
  assert(!ctrl.shooter.openRub && !ctrl.shooter.auto_shoot);
  ctrl.mode=CONTROL::AUTO;deliver(port,encode_aim(aim,motion_seq++,xuc.boot_id));assert(xuc.AimReady());
  rc.requested_mode=CONTROL::AUTO;rc.last_rc_tick=xTaskGetTickCount()-101;
  rc.OnRC();rc.OnPC();
  assert(ctrl.mode==CONTROL::STOP && !xuc.AimReady() && !xuc.aim_control_en);
  assert(!ctrl.shooter.openRub && !ctrl.shooter.auto_shoot);
  ctrl.mode=CONTROL::AUTO;deliver(port,encode_aim(aim,motion_seq++,xuc.boot_id));
#else
  assert(!xuc.AimReady() && xuc.last_reject==XUC_NOT_ALLOWED);
#endif
  // Duplicate, CRC, bad reference, stale, reboot identity, epoch and target invalidity.
  deliver(port,encode_aim(aim,xuc.aim_last_seq,xuc.boot_id));assert(xuc.last_reject==XUC_DUPLICATE_OR_OLD);
  const uint16_t seq=uint16_t(xuc.aim_last_seq+1);
  auto damaged=encode_aim(aim,seq,xuc.boot_id);damaged[20]^=1;deliver(port,damaged);assert(xuc.crc_err);
  auto bad=aim;++bad.ref_time_us;deliver(port,encode_aim(bad,seq,xuc.boot_id));assert(xuc.last_reject==XUC_BAD_REFERENCE);
  auto extreme=aim;extreme.yaw_error=1e30f;
  deliver(port,encode_aim(extreme,seq,xuc.boot_id));assert(xuc.last_reject==XUC_INVALID_VALUE);
  assert(std::isfinite(XUC::WrapPi(1e30f))); // must finish, never loop on float precision loss.
  deliver(port,encode_aim(aim,seq,xuc.boot_id+1));assert(xuc.last_reject==XUC_BAD_SESSION);
  test_now_us=ref.time_us+150001;deliver(port,encode_aim(aim,seq,xuc.boot_id));assert(xuc.last_reject==XUC_STALE);
  // Stop is reference-independent and must still work with missing IMU and late data.
  imu_pantile.valid=false;deliver(port,encode_aim({},seq,xuc.boot_id));assert(!xuc.aim_accepted && !xuc.aim_fire_req);
  imu_pantile.valid=true;test_now_us=1000000;
  deliver(port,encode_aim(aim,uint16_t(seq+1),xuc.boot_id));assert(xuc.aim_last_seq==uint16_t(seq+1)); // sequence wrap was accepted.
  ++imu_pantile.epoch;assert(!xuc.AimReady());
  deliver(port,encode_aim(aim,uint16_t(seq+2),xuc.boot_id));assert(xuc.last_reject==XUC_BAD_REFERENCE);
  --imu_pantile.epoch;aim.flags=1;aim.yaw_error=aim.pitch_error=0;deliver(port,encode_aim(aim,uint16_t(seq+2),xuc.boot_id));assert(!xuc.AimReady());
  ctrl.mode=CONTROL::STOP;assert(!xuc.AimReady());
  // A wheel-only heartbeat uses flags=5, fresh references, zero errors and no
  // target-valid bit. It may spin wheels, but must never move a gimbal/feed.
  ctrl.mode=CONTROL::AUTO;xuc.RevokeAim();imu_pantile.valid=true;
  Aim wheel=aim;wheel.flags=5;wheel.yaw_error=wheel.pitch_error=0;
  uint16_t next=uint16_t(xuc.aim_last_seq+1);
  deliver(port,encode_aim(wheel,next++,xuc.boot_id));
  Motor still_yaw;ctrl.pantile_motor[CONTROL::PANTILE::YAW]=&still_yaw;
  still_yaw.setangle=900;still_yaw.angle[now]=700;
  ctrl.pantile.hold_position=false; // New physical fixture starts a new hold entry.
  DMmotor[0].decoded=true;DMmotor[0].status=1;DMmotor[0].pos=.1f;
  ctrl.pantile.mark_pitch=.35f;
  standby.last_rc_tick=xTaskGetTickCount();standby.OnRC();standby.OnPC();
  assert(!xuc.AimReady() && !ctrl.shooter.auto_shoot);
#if XUC_ALLOW_CONTROL
  assert(still_yaw.setangle==700 && ctrl.pantile.mark_pitch==.1f);
#else
  assert(still_yaw.setangle==900 && ctrl.pantile.mark_pitch==.35f);
#endif
#if XUC_ALLOW_CONTROL && (!XUC_REQUIRE_VERIFIED_POSE || (XUC_SAMPLE_TIME_VERIFIED && XUC_AXES_VERIFIED))
  assert(xuc.FrictionReady() && ctrl.shooter.openRub && xuc.last_reject==XUC_OK);
  assert(xuc.LinkState() & XUC_ST_FRICTION_ALLOW);
#else
  assert(!xuc.FrictionReady() && !ctrl.shooter.openRub);
#endif
  assert(!(xuc.LinkState() & (XUC_ST_GIMBAL_ALLOW|XUC_ST_FIRE_ALLOW)));
  auto forbidden=wheel;forbidden.flags|=fire_request;
  deliver(port,encode_aim(forbidden,next,xuc.boot_id));assert(xuc.last_reject==XUC_INVALID_VALUE);
  forbidden=wheel;forbidden.yaw_error=.1f;
  deliver(port,encode_aim(forbidden,next,xuc.boot_id));assert(xuc.last_reject==XUC_INVALID_VALUE);
  // Serial/vision stall expires this lease in 100ms; the 5s hold lives on host.
  test_now_us=ref.time_us+100001;standby.OnPC();
  assert(!xuc.FrictionReady() && !ctrl.shooter.openRub && !ctrl.shooter.auto_shoot);
  test_now_us=1000000;deliver(port,encode_aim(wheel,next++,xuc.boot_id));
  imu_pantile.valid=false;standby.OnPC();assert(!ctrl.shooter.openRub);
  imu_pantile.valid=true;
  deliver(port,encode_aim({},next++,xuc.boot_id));standby.OnPC();
  assert(!ctrl.shooter.openRub && !ctrl.shooter.auto_shoot); // explicit shutdown
  deliver(port,encode_aim(wheel,next++,xuc.boot_id));
  standby.last_rc_tick=xTaskGetTickCount()-101;standby.OnRC();standby.OnPC();
  assert(ctrl.mode==CONTROL::STOP && !ctrl.shooter.openRub && !ctrl.shooter.auto_shoot);
  ctrl.mode=CONTROL::AUTO;deliver(port,encode_aim(wheel,next++,xuc.boot_id));
  standby.last_rc_tick=xTaskGetTickCount();standby.requested_mode=CONTROL::STOP;
  standby.OnRC();standby.OnPC();assert(!ctrl.shooter.openRub && !ctrl.shooter.auto_shoot);
  std::cout<<"PASS actual MCU parser/CRC + host V2 interop; independent IMU arrivals, reference/lease/boot/epoch/seq guards and actual ApplyAim revocation (control="<<XUC_ALLOW_CONTROL<<")\n";
}
