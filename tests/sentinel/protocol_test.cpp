#include "io/sentinel/protocol.hpp"
#include "firmware/sentry/STM32F405/feed_diagnostics.h"
#include <cassert>
#include <iostream>
using namespace io::sentinel;
void feed_diagnostic_checks(){
  FeedDiagnostics d;
  d.Feedback(4,0,0xffffff00u);d.Request(65535,0xffffff10u);
  d.Issue(65535,0xffffff20u,true,-1);
  d.Feedback(1,-99,0xffffff30u);assert(d.motion_count==0);
  d.Feedback(8190,-120,0xffffff40u);assert(d.motion_count==0); // only 6 ticks
  d.Feedback(8188,-120,0x00000010u); // encoder + us clock wrap
  assert(d.motion_count==1 && d.motion_seq==65535 && d.motion_us==16);
  assert(uint32_t(d.motion_us-d.issue_us)==240 && !d.pending);
  d.Feedback(8180,-400,30);assert(d.motion_count==1);
  d.Issue(0,40,true,-1);assert(d.already_moving && !d.pending);
  d.Feedback(8170,0,100);d.Issue(1,110,true,-1);
  d.Feedback(8150,-500,101000);assert(d.motion_count==1 && !d.pending); // CAN gap
  d.Feedback(8150,0,101100);d.Issue(2,101110,true,-1);
  d.Feedback(8170,500,101120);assert(d.motion_count==1); // opposite direction
  d.Feedback(8130,-500,101130);assert(d.motion_count==2 && d.motion_seq==2);
  Frame f{feed_diagnostic_type,3,123,Bytes(52)};
  auto* p=f.payload.data();p[0]=1;p[1]=d.Flags(101140);
  put16(p+2,2);put16(p+8,2);put16(p+10,2);
  put32(p+12,101110);put32(p+16,101130);put32(p+20,4);put32(p+24,2);
  put32(p+28,101140);put16(p+36,uint16_t(-500));put16(p+40,uint16_t(-6500));
  put32(p+44,uint32_t(-1234));put_float(p+48,-2345);
  FeedDiagnostic decoded;assert(decode_feed_diagnostic(f,decoded));
  assert(decoded.feeder_rpm==-500 && decoded.right_rpm==-6500 && decoded.encoder_sum==-1234);
  const auto wire=encode(f);Decoder decoder;int accepted=0;
  decoder.feed(wire.data(),wire.size(),1,[&](const Frame& x){assert(x.type==feed_diagnostic_type);++accepted;});
  assert(accepted==1);
  State state;assert(!state.apply(f,1) && !state.feedback.received); // cannot authorize/renew link
  f.payload[0]=2;assert(!decode_feed_diagnostic(f,decoded));f.payload[0]=1;
  f.payload[42]=1;assert(!decode_feed_diagnostic(f,decoded));f.payload[42]=0;
  f.payload.pop_back();assert(!decode_feed_diagnostic(f,decoded));
}
Frame imu(uint16_t seq,uint32_t time,uint8_t flags=15,uint32_t boot=0x11223344,uint16_t epoch=1){
  Bytes p(32,0);p[0]=1;p[1]=flags;put16(p.data()+2,epoch);put32(p.data()+4,time);
  put_float(p.data()+8,.1f);put_float(p.data()+12,.2f);put_float(p.data()+16,3.1f);
  put_float(p.data()+20,1);put_float(p.data()+24,2);put_float(p.data()+28,3);
  return {gimbal_type,seq,boot,p};
}
Frame status(uint16_t seq,uint16_t accepted=0,uint32_t boot=0x11223344){
  Bytes p(12,0);p[6]=auto_mode;put16(p.data()+4,accepted);put16(p.data()+8,0xffff);
  return {status_type,seq,boot,p};
}
Frame chassis(uint16_t seq,uint32_t time,uint32_t boot=0x11223344){
  auto f=imu(seq,time,orient_valid,boot);f.type=chassis_type;f.payload.resize(20);f.payload[0]=2;return f;
}
void startup_half_range_probe(){
  // The failure's seq=3631 is inferred from a prior captured seq=3630, not a
  // header observed during the failed launch. Boot/source time are recorded.
  State cached;const int64_t t=1000000000;const uint32_t boot=2243602295u;
  cached.apply(imu(3631,72634445,3,boot),t);
  cached.apply(status(40526,0,boot),t);
  const bool first=cached.apply(imu(40527,810550719,3,boot),t+1000000000);
  const bool link=cached.apply(status(40527,0,boot),t+1000000000);
  const bool second=cached.apply(imu(40528,810570719,3,boot),t+1020000000);
  assert(!first && link && second && cached.feedback.gimbal.time_us==810570719);
  assert(cached.pose_valid(t+1020000000) && cached.generation==2);
  assert(!cached.status_valid(t+1020000000) && !cached.can_stop());
  assert(!cached.reference_ready(t+1020000000,false));
  assert(cached.apply(status(40528,700,boot),t+1020000001));
  assert(cached.reference_ready(t+1020000001,false) && cached.take_sequence()==701);
  std::cout<<"STARTUP_HALF_RANGE inferred_cached_seq=3631 cached_source=72634445"
           <<" first_accepted="<<first<<" status_accepted="<<link
           <<" second_accepted="<<second<<" cached_source_after="<<cached.feedback.gimbal.time_us
           <<" pose="<<cached.pose_valid(t+1020000000)<<"\n";
}
void startup_recovery_boundaries(){
  const int64_t t=1000000000;const uint32_t boot=0x11223344;
  const auto initial=[&]{State s;s.apply(imu(3631,72634445),t);
    s.apply(chassis(3631,72634445),t);s.apply(status(3631,123),t);return s;};
  // All three type baselines may be stale, not just the gimbal sequence.
  auto group=initial();
  assert(!group.apply(status(40527,900),t+200000000));
  assert(!group.apply(chassis(40527,810550719),t+200000000));
  assert(!group.apply(imu(40527,810550719),t+200000000));
  assert(group.apply(imu(40528,810570719),t+220000000));
  assert(group.generation==2 && !group.feedback.chassis.received && !group.feedback.link.received);
  assert(!group.can_stop() && !group.reference_ready(t+220000000));
  assert(group.apply(chassis(40528,810570719),t+220000001));
  assert(group.apply(status(40528,900),t+220000002));
  assert(group.reference_ready(t+220000002) && group.take_sequence()==901);
  assert(group.apply(imu(40529,810590719),t+240000000) && group.generation==2);
  // Source high water, not the most recent (possibly regressed) accepted time.
  auto high=initial();assert(high.apply(imu(3632,72634440),t+1));
  assert(!high.apply(imu(40527,72634441),t+200000000));
  assert(!high.apply(imu(40528,72634442),t+220000000));
  assert(high.generation==1 && !high.pose_valid(t+220000000));
  // No frozen-time stream or ordinary old replay can renew freshness/unlock.
  auto frozen=initial();
  for(int i=0;i<8;++i){
    assert(!frozen.apply(imu(uint16_t(40527+i),72634445),t+200000000+i*20000000));
    assert(!frozen.pose_valid(t+200000000+i*20000000));
  }
  auto replay=initial();
  assert(!replay.apply(imu(3630,72614445),t+200000000));
  assert(!replay.apply(imu(3629,72594445),t+220000000));
  assert(replay.generation==1 && replay.feedback.gimbal.seq==3631);
  // Duplicates cannot extend the first candidate's 100ms deadline.
  auto timeout=initial();
  assert(!timeout.apply(imu(40527,810550719),t+200000000));
  assert(!timeout.apply(imu(40527,810550719),t+290000000));
  assert(!timeout.apply(imu(40528,810570719),t+310000000));
  assert(timeout.generation==1 && !timeout.pose_valid(t+310000000));
  assert(timeout.apply(imu(40529,810590719),t+330000000));
  // A nonconsecutive candidate must establish another pair; no single jump.
  auto skipped=initial();
  assert(!skipped.apply(imu(40527,810550719),t+200000000));
  assert(!skipped.apply(imu(40529,810590719),t+220000000));
  assert(!skipped.apply(imu(40528,810570719),t+230000000));
  assert(skipped.generation==1);
  assert(skipped.apply(imu(40530,810610719),t+240000000));
  // Invalid data and frozen source are never a confirming second sample.
  for(int variant=0;variant<4;++variant){
    auto bad=initial();assert(!bad.apply(imu(40527,810550719),t+200000000));
    auto f=imu(40528,variant==0?810550719:810570719,variant==1?0:15,boot,variant==2?2:1);
    if(variant==3)f.payload[0]=2;
    assert(!bad.apply(f,t+220000000));assert(bad.generation==1 && !bad.pose_valid(t+220000000));
  }
  // Accepted normal samples and epoch changes cancel a prior candidate.
  auto normal=initial();assert(!normal.apply(imu(40527,810550719),t+200000000));
  assert(normal.apply(imu(3632,72654445),t+220000000));
  assert(!normal.apply(imu(40528,810570719),t+221000000));
  assert(normal.generation==1);
  assert(!normal.apply(imu(40529,810590719),t+420000000));
  assert(normal.apply(imu(40530,810610719),t+440000000));
  auto epoch=initial();assert(!epoch.apply(imu(40527,810550719),t+200000000));
  assert(epoch.apply(imu(3632,10,15,boot,2),t+220000000));
  assert(!epoch.apply(imu(40528,810570719,15,boot,1),t+420000000));
  assert(epoch.generation==2);
  // Retired boot replay protection survives recovery.
  assert(group.apply(imu(0,10,15,123),t+450000000));
  assert(!group.apply(imu(40530,810610719),t+460000000));
  assert(!group.apply(status(40530,901),t+460000000));
  auto status_only=State{};
  assert(status_only.apply(status(40000),t));
  assert(status_only.status_valid(t) && !status_only.pose_valid(t) && !status_only.reference_ready(t));
  std::cout<<"PASS startup recovery: whole cached group, reseed, frozen/duplicate/replay, source high water, timeout, invalid/epoch/boot guards\n";
}
int main(){
  feed_diagnostic_checks();
  startup_half_range_probe();
  startup_recovery_boundaries();
  assert(crc16(reinterpret_cast<const uint8_t*>("123456789"),9)==0x6f91);
  Aim a;a.ref_epoch=1;a.ref_seq=42;a.ref_time_us=12345678;a.flags=3;a.yaw_error=.01f;a.pitch_error=-.02f;
  const Bytes golden={0xa6,0x6a,2,0x10,0x18,0,7,0,0x44,0x33,0x22,0x11,
    0,0,0,0,1,0,0x2a,0,0x4e,0x61,0xbc,0,0x64,0,3,0,0x0a,0xd7,0x23,0x3c,0x0a,0xd7,0xa3,0xbc,0xb8,0x3c};
  assert(encode_aim(a,7,0x11223344)==golden);
  auto stop=encode_aim({},8,0x11223344);assert(stop.size()==38 && stop[26]==0);
  bool rejected=false;try{a.flags=8;encode_aim(a,1,1);}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
  Decoder d;int count=0;auto accept=[&](const Frame& f){++count;assert(f.type==1 || f.type==0x81);};
  auto bytes=encode(imu(65535,0xfffffff0));auto bad=bytes;bad[20]^=1;
  const uint8_t noise[]={0,0xff,0xa6};d.feed(noise,3,1,accept);d.feed(bad.data(),bad.size(),2,accept);
  for(auto b:bytes)d.feed(&b,1,3,accept);
  auto st=encode(status(1));Bytes pair=bytes;pair.insert(pair.end(),st.begin(),st.end());
  d.feed(pair.data(),pair.size(),4,accept);assert(count==3 && d.crc_errors);
  auto future=encode({0x77,0,1,Bytes(128,0)});d.feed(future.data(),future.size(),5,accept);assert(d.unknown_types==1);
  auto nan=imu(3,123);put_float(nan.payload.data()+8,std::numeric_limits<float>::quiet_NaN());
  auto nbytes=encode(nan);d.feed(nbytes.data(),nbytes.size(),6,accept);assert(d.field_errors==1);
  nan.payload[1]=0;nbytes=encode(nan);d.feed(nbytes.data(),nbytes.size(),7,accept);assert(count==4);
  // A legal-length false header must expire instead of hiding the following real frame forever.
  Bytes partial(12,0);partial[0]=0xa6;partial[1]=0x6a;partial[2]=2;put16(partial.data()+4,128);
  d.feed(partial.data(),partial.size(),100,accept);d.feed(bytes.data(),bytes.size(),101,accept);
  d.feed(nullptr,0,100000102,accept);assert(count==5 && d.buffered()==0);
  Bytes huge(1000000,0x6a);d.feed(huge.data(),huge.size(),100000103,accept);assert(d.buffered()<=max_frame);
  DetectionFrictionHold hold;
  const int64_t tick=10000000000LL;
  assert(!hold.update(true,false,tick)); // no detection yet
  assert(hold.update(true,true,tick));
  assert(hold.update(true,false,tick+4999999999LL));
  assert(!hold.update(true,false,tick+5000000000LL)); // exact 5s boundary
  assert(hold.update(true,true,tick+6000000000LL)); // reacquisition
  assert(hold.update(true,true,tick+9000000000LL)); // another detection restarts loss timer
  assert(hold.update(true,false,tick+13999999999LL));
  assert(!hold.update(true,false,tick+14000000000LL));
  assert(hold.update(true,true,tick+15000000000LL));
  assert(!hold.update(false,false,tick+15000000001LL)); // STOP/fault cancels immediately
  assert(!hold.update(true,false,tick+15000000002LL)); // no stale re-arm
  assert(hold.update(true,true,tick+16000000000LL));hold.reset();
  assert(!hold.update(true,false,tick+16000000001LL)); // shutdown/boot/epoch reset
  assert(hold.update(true,true,tick+17000000000LL));
  assert(!hold.update(true,false,tick+16999999999LL)); // clock regression
  State s;const int64_t t=1000000000;
  assert(s.apply(imu(65535,0xfffffff0),t));assert(s.pose_valid(t) && !s.reference_ready(t));
  s.apply(status(65535,65535),t);assert(s.reference_ready(t));assert(s.take_sequence()==0);
  s.apply(imu(0,20),t+1000);assert(s.reference_ready(t+1000)); // both wraps are legal.
  assert(!s.apply(imu(65535,0xfffffff0),t+2000));
  s.apply(imu(1,20),t+101000001);s.apply(status(0),t+101000001);
  assert(!s.pose_valid(t+101000001)); // same timestamp does not renew the sensor.
  s.apply(imu(2,30,3),t+101000002);assert(s.pose_valid(t+101000002));assert(!s.reference_ready(t+101000002));assert(s.reference_ready(t+101000002,false));
  auto stopped=status(1);stopped.payload[6]=0;s.apply(stopped,t+101000002);
  assert(!s.reference_ready(t+101000002,false));
  s.apply(status(2),t+101000002);
  s.apply(imu(3,40),t+101000003);assert(s.reference_ready(t+101000003));
  s.apply(imu(4,39),t+101000004);assert(!s.pose_valid(t+101000004));
  s.apply(imu(5,50,15,0x11223344,2),t+101000005);assert(s.generation==2);
  s.apply(imu(0,1,15,123),t+101000006);assert(!s.reference_ready(t+101000006));
  assert(!s.apply(imu(6,60),t+101000007)); // delayed packets from retired boot cannot roll back identity.
  s.apply(status(0,321,123),t+101000008);assert(s.reference_ready(t+101000008));assert(s.take_sequence()==322);
  assert(!s.reference_ready(t+250000000));
  std::cout<<"PASS V2 wire golden, fragmentation/noise/CRC, bounded parser/half-frame expiry, validity, seq/time wraps, frozen sensor, epoch/boot resets and host sequence recovery\n";
}
