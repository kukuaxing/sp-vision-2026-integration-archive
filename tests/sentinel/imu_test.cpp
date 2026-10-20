#include "io/sentinel/protocol.hpp"
#include "firmware/sentry/STM32F405/imu.cpp"
#include <array>
#include <cassert>
#include <iostream>
#include <limits>
#include <string>
using namespace io::sentinel;
namespace {
constexpr double radians = 0.017453292519943295769;
using Quat = std::array<double,4>; // Independent forward rotations, WXYZ.
Quat multiply(const Quat& a,const Quat& b){
  return {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
          a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
          a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
          a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
}
Quat axis(int index,double degrees){
  Quat q{std::cos(degrees*radians/2),0,0,0};q[index+1]=std::sin(degrees*radians/2);return q;
}
Quat zyx(double roll,double pitch,double yaw){
  return multiply(multiply(axis(2,yaw),axis(1,pitch)),axis(0,roll));
}
void fix_crc(Bytes& b){
  uint16_t crc=0;
  for(std::size_t i=0;i<b.size();++i){
    if(i==4||i==5)continue;crc^=uint16_t(b[i])<<8;
    for(int bit=0;bit<8;++bit)crc=uint16_t((crc<<1)^((crc&0x8000)?0x1021:0));
  }
  put16(b.data()+4,crc);
}
Bytes sensor(uint32_t time,Quat q=zyx(5,10,20),std::array<float,3> device_euler={71,-22,63}){
  Bytes b(82,0);b[0]=0x5a;b[1]=0xa5;b[2]=76;b[6]=0x91;
  put32(b.data()+14,time);put_float(b.data()+30,1);put_float(b.data()+34,2);put_float(b.data()+38,3);
  for(int i=0;i<3;++i)put_float(b.data()+54+4*i,device_euler[i]);
  for(int i=0;i<4;++i)put_float(b.data()+66+4*i,static_cast<float>(q[i]));
  fix_crc(b);return b;
}
bool close(double a,double b,double epsilon=3e-5){return std::abs(a-b)<epsilon;}
void expect_rotation(IMU& imu,const Quat& input){
  // Reconstruct the emitted Euler angles through forward Hamilton products;
  // do not repeat the production quaternion-to-Euler extraction formula.
  auto actual=zyx(imu.GetAngleRoll(),imu.GetAnglePitch(),imu.GetAngleYaw());
  double norm=0,dot=0;for(int i=0;i<4;++i){norm+=input[i]*input[i];dot+=actual[i]*input[i];}
  assert(std::abs(std::abs(dot/std::sqrt(norm))-1)<2e-7);
}
void check_orientation(Quat q,std::array<float,3> wire_euler={71,-22,63}){
  IMU imu{};UART port;imu.Init(&port,nullptr,115200,CH010);
  port.rx=sensor(100,q,wire_euler);port.arrival=test_now_us;imu.Decode();
  assert(imu.IsValid() && imu.sample_ok==1 && imu.quaternion_bad==0);
  expect_rotation(imu,q);
  const auto raw=imu.GetDeviceEuler();
  assert(raw.roll==wire_euler[0] && raw.pitch==wire_euler[1] && raw.yaw==wire_euler[2]);
  // Body angular velocity remains X/Y/Z and is not relabelled with device Euler.
  assert(imu.GetGyroRoll()==1 && imu.GetGyroPitch()==2 && imu.GetGyroYaw()==3);
}
} // namespace

int main(){
  // Manufacturer ENU/312 calls pure Y rotation "roll". Our control convention
  // must emit ZYX pitch instead, derived from quaternion rather than swapped fields.
  IMU pure_y{};UART pure_port;pure_y.Init(&pure_port,nullptr,115200,CH010);
  pure_port.rx=sensor(1,axis(1,-11),{-11,0,0});pure_port.arrival=900000;pure_y.Decode();
  assert(close(pure_y.GetAnglePitch(),-11) && close(pure_y.GetAngleRoll(),0));
  assert(pure_y.GetDeviceEuler().roll==-11 && pure_y.GetDeviceEuler().pitch==0);
  // Combined ENU/312 rotation Rz(yaw)*Rx(device pitch)*Ry(device roll) is not
  // equivalent to swapping two Euler fields. Compare complete rotations.
  check_orientation(multiply(multiply(axis(2,42),axis(0,13)),axis(1,25)),{25,13,42});
  const auto joint=zyx(25,-17,42);check_orientation(joint);
  Quat negative=joint;for(auto& v:negative)v=-v;check_orientation(negative);
  Quat slightly_scaled=joint;for(auto& v:slightly_scaled)v*=1.005;check_orientation(slightly_scaled);
  for(const auto& q: {zyx(-120,43,179),zyx(80,-65,-179),zyx(20,89.9,30),
                     zyx(20,90,30),zyx(20,-90,30),Quat{1,0,0,0}})check_orientation(q);
  // Published manufacturer example (legacy manual section 4.10), independent
  // of the test forward-angle generator. Its device Euler is not a ZYX oracle.
  check_orientation({-.485922,-.14982,.0380868,.860223},{13.0519f,12.1885f,-122.477f});

  // Actual CRC-complete gimbal/chassis HI91 packets read through attach-only
  // SWD on 2026-10-16, recorded in logs/deployment/sentinel-imu-quaternion-preflash.json.
  // Keep their original CRC and non-perfect unit norms; do not regenerate them
  // using the test encoder or silently substitute manufactured Euler values.
  const std::array<const char*,2> live_hex={
    "5aa54c00a7f69123232c00000000903b1400938b22bd14118b3b27d87e3f23ea3bbc19d9c13ea2e5adbd000000000000000000000000848e1040db97373f0ffbbdbf05f97f3f8259d53b86d0a03c022f52bc",
    "5aa54c004d9791232321000000006c3c14004f205ebc9495c43b2ef87f3f4016193ddf5ca4bee8b08fbe0000000000000000000000005c3e1a3fc755523e59a89bc0c1ce7f3f90f9033bebaca93b3acf2dbd"};
  const std::array<std::array<double,3>,2> live_expected={{{.717462473871142,2.258570580438277,-1.4559810840484644},
    {.20535376351458232,.6025237500234165,-4.862227918462555}}};
  for(std::size_t n=0;n<live_hex.size();++n){
    const std::string hex=live_hex[n];Bytes frame;
    for(std::size_t i=0;i<hex.size();i+=2)frame.push_back(static_cast<uint8_t>(std::stoul(hex.substr(i,2),nullptr,16)));
    assert(frame.size()==82);
    IMU live{};UART live_port;live.Init(&live_port,nullptr,115200,CH010);
    live_port.rx=frame;live_port.arrival=test_now_us;live.Decode();
    assert(live.IsValid() && live.sample_ok==1 && live.crc_fail==0 && live.quaternion_bad==0);
    assert(close(live.GetAngleRoll(),live_expected[n][0]) && close(live.GetAnglePitch(),live_expected[n][1]) && close(live.GetAngleYaw(),live_expected[n][2]));
    Quat q{};for(int i=0;i<4;++i)q[i]=R4(frame.data()+66+4*i);expect_rotation(live,q);
    assert(close(live.GetDeviceEuler().roll,R4(frame.data()+54)) && close(live.GetDeviceEuler().pitch,R4(frame.data()+58)));
  }

  IMU imu{};UART port;imu.Init(&port,nullptr,115200,CH010);
  test_now_us=1000000;
  port.rx=sensor(100);port.arrival=900000;imu.Decode();
  assert(imu.IsValid() && imu.GetArrivalTimeUs()==900000 && imu.GetTimestamp()==100);
  assert(close(imu.GetAngleRoll(),5) && close(imu.GetAnglePitch(),10) && close(imu.GetAngleYaw(),20));
  test_now_us+=10000;port.rx=sensor(100,zyx(80,30,40));port.arrival=950000;imu.Decode();
  assert(imu.GetArrivalTimeUs()==900000 && imu.sample_ok==1 && close(imu.GetAngleRoll(),5));
  test_now_us+=60000;imu.Decode();assert(!imu.IsValid());
  port.rx=sensor(101);port.arrival=1000000;imu.Decode();assert(imu.IsValid());
  const auto old_epoch=imu.GetEpoch();port.rx=sensor(1);port.arrival=1100000;imu.Decode();
  assert(imu.GetEpoch()==uint16_t(old_epoch+1) && imu.GetArrivalTimeUs()==1100000);

  // Every rejection must preserve the accepted attitude, timestamp, epoch,
  // sample count and arrival metadata. They must not keep freshness alive.
  auto reject_without_update=[&](Bytes frame){
    const auto count=imu.sample_ok,arrival=imu.GetArrivalTimeUs(),time=imu.GetTimestamp();
    const auto epoch=imu.GetEpoch();const float roll=imu.GetAngleRoll(),pitch=imu.GetAnglePitch(),yaw=imu.GetAngleYaw();
    test_now_us+=10000;port.rx=frame;port.arrival=test_now_us;imu.Decode();
    assert(imu.sample_ok==count && imu.GetArrivalTimeUs()==arrival && imu.GetTimestamp()==time && imu.GetEpoch()==epoch);
    assert(imu.GetAngleRoll()==roll && imu.GetAnglePitch()==pitch && imu.GetAngleYaw()==yaw);
  };
  // Bad packet timestamps deliberately go backwards; an invalid pose cannot
  // consume a restart epoch before a later valid sample arrives.
  for(Quat bad: {Quat{0,0,0,0},Quat{2,0,0,0},Quat{.98,0,0,0},Quat{1.02,0,0,0}})
    reject_without_update(sensor(0,bad));
  for(int i=0;i<4;++i){Quat bad=joint;bad[i]=std::numeric_limits<double>::quiet_NaN();reject_without_update(sensor(0,bad));}
  reject_without_update(sensor(0,{std::numeric_limits<double>::infinity(),0,0,0}));
  assert(imu.quaternion_bad==9 && !imu.IsValid());
  Bytes truncated=sensor(2);truncated.pop_back();reject_without_update(truncated);
  for(int payload: {10,60,75}){
    Bytes short_packet=sensor(2);short_packet.resize(payload+6);short_packet[2]=uint8_t(payload);fix_crc(short_packet);
    reject_without_update(short_packet);
  }
  assert(imu.short_payload==3);
  Bytes oversized=sensor(2);oversized.push_back(0);oversized[2]=77;fix_crc(oversized);reject_without_update(oversized);
  Bytes damaged=sensor(2);damaged[20]^=1;reject_without_update(damaged);assert(imu.crc_fail==1);
  // Rejected new timestamps were not consumed; a corrected packet can recover.
  port.rx=sensor(2);port.arrival=test_now_us;imu.Decode();assert(imu.IsValid() && imu.GetTimestamp()==2);
  IMU wrap{};UART port2;wrap.Init(&port2,nullptr,115200,CH010);
  port2.rx=sensor(0xfffffff0);wrap.Decode();port2.rx=sensor(5);wrap.Decode();assert(wrap.GetEpoch()==1);
  IMU hi226{};UART port3;hi226.Init(&port3,nullptr,115200,HI226);
  port3.rx=sensor(100,axis(1,-11),{-11,0,0});hi226.Decode();assert(close(hi226.GetAnglePitch(),-11));

  // The unrelated 0x55/IMU601 path keeps its existing Euler wire semantics.
  IMU old{};UART old_port;old.Init(&old_port,nullptr,115200,IMU601);
  Bytes old_frame(23,0);old_frame[0]=old_frame[1]=0x55;old_frame[2]=1;old_frame[3]=18;
  put16(old_frame.data()+4,1024);put16(old_frame.data()+6,2048);put16(old_frame.data()+8,3072);
  for(int i=0;i<22;++i)old_frame[22]=uint8_t(old_frame[22]+old_frame[i]);
  old_port.rx=old_frame;old.Decode();
  assert(old.IsValid() && old.GetAngleRoll()==5.625f && old.GetAnglePitch()==11.25f && old.GetAngleYaw()==16.875f);
  std::cout<<"PASS actual CH010/HI226: full HI91, quaternion WXYZ->ZYX, ENU312 distinction, joint rotations, q/-q, normalization/singularities, invalid quaternion/CRC/length/time guards; IMU601 preserved\n";
}
