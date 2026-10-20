#pragma once
// Supplied Sentinel V2: little-endian, radians, no device I/O.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>
namespace io::sentinel {
using Bytes=std::vector<uint8_t>;
constexpr double pi=3.14159265358979323846, deg_to_rad=pi/180.0;
constexpr std::size_t header_size=12,max_payload=128,max_frame=142;
constexpr uint8_t gimbal_type=1,chassis_type=2,aim_type=0x10,status_type=0x81;
constexpr uint8_t orient_valid=1,gyro_valid=2,sample_valid=4,axes_verified=8;
constexpr uint8_t control_enable=1,target_valid=2,friction_enable=4,fire_request=8;
constexpr uint8_t auto_mode=0x80; // paired firmware extension; telemetry also runs in STOP.
// Call with a monotonic clock. A fault/mode exit resets the hold, rather than
// extending the last target or remembering a detection across re-arming.
class DetectionFrictionHold {
public:
  int64_t timeout_ns=5000000000LL;
  void reset(){seen_=false;last_=0;}
  bool update(bool allowed,bool detected,int64_t now_ns){
    if(!allowed || (seen_ && now_ns<last_)){reset();return false;}
    if(detected){seen_=true;last_=now_ns;}
    return seen_ && (detected || now_ns-last_<timeout_ns);
  }
private:
  bool seen_=false;int64_t last_=0;
};
static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559);
inline uint16_t crc16(const uint8_t* p,std::size_t n) {
  uint16_t c=0xffff;
  for(std::size_t i=0;i<n;++i){c^=p[i];for(int b=0;b<8;++b)c=(c>>1)^((c&1)?0x8408:0);}
  return c;
}
inline uint16_t u16(const uint8_t* p){return p[0]|(uint16_t(p[1])<<8);}
inline uint32_t u32(const uint8_t* p){return uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);}
inline float f32(const uint8_t* p){const auto bits=u32(p);float v;std::memcpy(&v,&bits,4);return v;}
inline void put16(uint8_t* p,uint16_t v){p[0]=uint8_t(v);p[1]=uint8_t(v>>8);}
inline void put32(uint8_t* p,uint32_t v){for(int i=0;i<4;++i)p[i]=uint8_t(v>>(8*i));}
inline void put_float(uint8_t* p,float v){uint32_t bits;std::memcpy(&bits,&v,4);put32(p,bits);}
inline bool newer(uint16_t a,uint16_t b){const uint16_t d=uint16_t(a-b);return d && d<0x8000;}
inline bool newer_time(uint32_t a,uint32_t b){const uint32_t d=a-b;return d && d<0x80000000u;}
struct Frame {uint8_t type=0;uint16_t seq=0;uint32_t boot_id=0;Bytes payload;};
inline Bytes encode(const Frame& f){
  if(f.payload.size()>max_payload)throw std::invalid_argument("Sentinel payload too large");
  Bytes b(header_size+f.payload.size()+2,0);b[0]=0xa6;b[1]=0x6a;b[2]=2;b[3]=f.type;
  put16(b.data()+4,uint16_t(f.payload.size()));put16(b.data()+6,f.seq);put32(b.data()+8,f.boot_id);
  std::copy(f.payload.begin(),f.payload.end(),b.begin()+header_size);
  put16(b.data()+b.size()-2,crc16(b.data(),b.size()-2));return b;
}
struct Imu {
  float roll=0,pitch=0,yaw=0,gx=0,gy=0,gz=0; // rad; sensor-body rad/s, not Euler derivatives.
  uint32_t time_us=0;uint16_t epoch=0,seq=0;uint8_t id=0,flags=0;bool received=false;
  bool valid()const{return received && (flags&orient_valid);}
};
struct LinkStatus {
  uint32_t host_session=0;uint16_t aim_last_seq=0,aim_age_ms=0xffff,seq=0;
  uint8_t state=0,last_reject=0;bool received=false;
};
struct Feedback {uint32_t boot_id=0;bool received=false;Imu gimbal,chassis;LinkStatus link;};
inline bool decode_imu(const Frame& f,Imu& out){
  const bool g=f.type==gimbal_type;
  if((!g && f.type!=chassis_type)||f.payload.size()!=(g?32u:20u))return false;
  const auto* p=f.payload.data();Imu a;
  a.id=p[0];a.flags=p[1];a.epoch=u16(p+2);a.time_us=u32(p+4);a.seq=f.seq;a.received=true;
  a.roll=f32(p+8);a.pitch=f32(p+12);a.yaw=f32(p+16);
  if(g){a.gx=f32(p+20);a.gy=f32(p+24);a.gz=f32(p+28);}
  if((a.flags&~0x0f)||(!g && (a.flags&gyro_valid)))return false;
  if((a.flags&orient_valid)&&(!std::isfinite(a.roll)||!std::isfinite(a.pitch)||!std::isfinite(a.yaw)))return false;
  if((a.flags&gyro_valid)&&(!std::isfinite(a.gx)||!std::isfinite(a.gy)||!std::isfinite(a.gz)))return false;
  out=a;return true;
}
inline bool decode_status(const Frame& f,LinkStatus& out){
  if(f.type!=status_type||f.payload.size()!=12)return false;
  const auto* p=f.payload.data();out={u32(p),u16(p+4),u16(p+8),f.seq,p[6],p[7],true};return true;
}
struct Aim {
  uint16_t ref_epoch=0,ref_seq=0,valid_for_ms=100;uint32_t ref_time_us=0;
  uint8_t flags=0;float yaw_error=0,pitch_error=0;
};
inline Bytes encode_aim(const Aim& a,uint16_t seq,uint32_t boot){
  if((a.flags&~0x0f)||!std::isfinite(a.yaw_error)||!std::isfinite(a.pitch_error)||
     ((a.flags&control_enable)&&(a.valid_for_ms<1||a.valid_for_ms>150))||
     (!(a.flags&control_enable)&&(a.flags||a.yaw_error!=0||a.pitch_error!=0)))
    throw std::invalid_argument("Invalid Sentinel AIM");
  Bytes p(24,0); // host_session/reserved zero. Offsets follow HandleAim, not the old document.
  put16(p.data()+4,a.ref_epoch);put16(p.data()+6,a.ref_seq);put32(p.data()+8,a.ref_time_us);
  put16(p.data()+12,a.valid_for_ms);p[14]=a.flags;
  put_float(p.data()+16,a.yaw_error);put_float(p.data()+20,a.pitch_error);
  return encode({aim_type,seq,boot,p});
}
class Decoder {
public:
  uint64_t headers=0,crc_errors=0,field_errors=0,unknown_types=0;
  template<class Callback>void feed(const uint8_t* p,std::size_t n,int64_t now_ns,Callback accept){
    parse(now_ns,accept,true);
    for(std::size_t i=0;i<n;++i){pending_.push_back(p[i]);parse(now_ns,accept,false);}
  }
  std::size_t buffered()const{return pending_.size();}
private:
  Bytes pending_;int64_t since_=0;
  template<class Callback>void parse(int64_t now_ns,Callback accept,bool expire){
    if(expire && since_ && now_ns-since_>100000000){
      if(!pending_.empty())pending_.erase(pending_.begin());since_=0;
    }
    while(!pending_.empty()){
      if(pending_[0]!=0xa6||(pending_.size()>1 && pending_[1]!=0x6a)){
        pending_.erase(pending_.begin());since_=0;continue;
      }
      if(!since_)since_=now_ns;
      if(pending_.size()<header_size)return;
      const auto n=u16(pending_.data()+4);
      if(pending_[2]!=2||n>max_payload){++field_errors;pending_.erase(pending_.begin());since_=0;continue;}
      const std::size_t total=header_size+n+2;
      if(pending_.size()<total)return;
      ++headers;
      if(crc16(pending_.data(),total-2)!=u16(pending_.data()+total-2)){
        ++crc_errors;pending_.erase(pending_.begin());since_=0;continue;
      }
      Frame f{pending_[3],u16(pending_.data()+6),u32(pending_.data()+8),Bytes(pending_.begin()+12,pending_.begin()+12+n)};
      Imu imu;LinkStatus status;
      if(f.type!=gimbal_type && f.type!=chassis_type && f.type!=status_type)++unknown_types;
      else if(!(decode_imu(f,imu)||decode_status(f,status)))++field_errors;
      else accept(f);
      pending_.erase(pending_.begin(),pending_.begin()+total);since_=0;
    }
  }
};
// Shared production/test state machine; caller holds the link mutex.
class State {
public:
  Feedback feedback;uint64_t generation=0;
  bool apply(const Frame& f,int64_t now_ns){
    Imu imu;LinkStatus status;
    if(!(decode_imu(f,imu)||decode_status(f,status)))return false;
    if(!feedback.received||feedback.boot_id!=f.boot_id){
      for(auto old:retired_)if(old==f.boot_id)return false;
      if(feedback.received){retired_.push_back(feedback.boot_id);if(retired_.size()>8)retired_.erase(retired_.begin());}
      feedback={};feedback.received=true;feedback.boot_id=f.boot_id;
      gimbal_rx_=progress_ns_=status_rx_=0;have_progress_=seeded_=false;
      recovery_pending_=false;++generation;
    }
    if(f.type==status_type){
      if(feedback.link.received && !newer(status.seq,feedback.link.seq))return false;
      feedback.link=status;status_rx_=now_ns;
      // Resume after the MCU's last accepted AIM when the host restarts.
      if(!seeded_){next_seq_=uint16_t(status.aim_last_seq+1);seeded_=true;}
    }else{
      auto& prev=f.type==gimbal_type?feedback.gimbal:feedback.chassis;
      if(prev.received && !newer(imu.seq,prev.seq)){
        if(f.type!=gimbal_type || !recover_gimbal_sequence(imu,now_ns))return false;
      }
      if(f.type==gimbal_type){
        recovery_pending_=false;
        if(prev.received && imu.epoch!=prev.epoch){have_progress_=false;++generation;}
        if(imu.valid() && imu.id==1 && (!have_progress_||newer_time(imu.time_us,progress_time_))){
          progress_time_=imu.time_us;progress_ns_=now_ns;have_progress_=true;
        }
        gimbal_rx_=now_ns;
      }
      prev=imu;
    }
    return true;
  }
  bool pose_valid(int64_t now_ns,int64_t timeout_ns=100000000)const{
    return feedback.received && feedback.gimbal.valid() && feedback.gimbal.id==1 && have_progress_ &&
      gimbal_rx_>0 && now_ns-gimbal_rx_<=timeout_ns && now_ns-progress_ns_<=timeout_ns &&
      feedback.gimbal.time_us==progress_time_;
  }
  bool status_valid(int64_t now_ns)const{return status_rx_>0 && now_ns-status_rx_<=100000000;}
  bool reference_ready(int64_t now_ns,bool require_verified_pose=true)const{
    return pose_valid(now_ns) && status_valid(now_ns) && seeded_ && feedback.boot_id!=0 &&
      (feedback.link.state&auto_mode) &&
      (!require_verified_pose ||
       (feedback.gimbal.flags&(orient_valid|sample_valid|axes_verified))==(orient_valid|sample_valid|axes_verified));
  }
  uint16_t take_sequence(){return next_seq_++;}
  bool can_stop()const{return feedback.received && seeded_;}
private:
  // A closed serial port can retain a complete old packet. After more than
  // half a uint16 sequence cycle, the current stream looks older than that
  // first packet. Only an expired source may recover, using two independently
  // advancing consecutive samples; duplicates never renew either deadline.
  bool recover_gimbal_sequence(const Imu& imu,int64_t now_ns){
    constexpr int64_t timeout_ns=100000000;
    const auto& prev=feedback.gimbal;
    if(!have_progress_ || progress_ns_<=0 || now_ns<progress_ns_ ||
       now_ns-progress_ns_<=timeout_ns || !imu.valid() || imu.id!=1 ||
       imu.epoch!=prev.epoch || imu.seq==prev.seq ||
       !newer_time(imu.time_us,progress_time_)){
      recovery_pending_=false;return false;
    }
    if(recovery_pending_){
      // Do not move the candidate deadline for duplicates, frozen source time,
      // backward time, or out-of-order replay. A fresh later candidate may
      // start a new pair after a timeout or an intervening missing sequence.
      if(now_ns<recovery_ns_ || !newer(imu.seq,recovery_candidate_.seq) ||
         !newer_time(imu.time_us,recovery_candidate_.time_us))return false;
      if(uint16_t(imu.seq-recovery_candidate_.seq)==1 &&
         now_ns-recovery_ns_<=timeout_ns &&
         uint32_t(imu.time_us-recovery_candidate_.time_us)<=100000u){
        recovery_pending_=false;
        // Other cached types can have the same half-cycle ambiguity. Discard
        // their baselines and require the next status to seed TX anew. Keep
        // this boot's identity and retired-boot replay protection intact.
        feedback.chassis={};feedback.link={};status_rx_=0;seeded_=false;
        next_seq_=0;++generation;return true;
      }
    }
    recovery_candidate_=imu;recovery_ns_=now_ns;recovery_pending_=true;
    return false;
  }
  int64_t gimbal_rx_=0,progress_ns_=0,status_rx_=0;uint32_t progress_time_=0;
  bool have_progress_=false,seeded_=false;uint16_t next_seq_=0;std::vector<uint32_t> retired_;
  Imu recovery_candidate_;
  int64_t recovery_ns_=0;bool recovery_pending_=false;
};
} // namespace io::sentinel
