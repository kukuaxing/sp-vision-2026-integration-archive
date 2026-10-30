#pragma once
#include "fixed_target_world.hpp"

namespace fixed_sim {
// Bounded first-order command follower, not a fitted real motor/PID model.
struct AngleServo {
  double angle=0,rate=0,target=0,tau=0,max_rate=2,max_acceleration=12;
  void command(double value) {
    target=value;
    if(tau==0){angle=target;rate=0;}
  }
  void advance(double duration) {
    if(!(duration>=0 && std::isfinite(duration)))throw std::invalid_argument("servo clock moved backwards");
    if(tau==0){angle=target;rate=0;return;}
    if(!(tau>0 && max_rate>0 && max_acceleration>0))throw std::invalid_argument("invalid servo model");
    while(duration>1e-12) {
      const double step=std::min(duration,.001);
      const double error=std::remainder(target-angle,2*pi);
      const double requested=std::clamp(error/tau,-max_rate,max_rate);
      rate+=std::clamp(requested-rate,-max_acceleration*step,max_acceleration*step);
      double change=rate*step;
      if(change*error>=0 && std::abs(change)>=std::abs(error)){change=error;rate=0;}
      angle+=change;duration-=step;
    }
  }
};

struct MuzzleSample {
  double time=0,yaw=0,pitch=0,speed=22;
  Projectile projectile() const {
    Projectile p;p.muzzle_time=time;
    p.velocity=speed*Eigen::Vector3d{std::cos(pitch)*std::cos(yaw),std::cos(pitch)*std::sin(yaw),std::sin(pitch)};
    return p;
  }
};
}
