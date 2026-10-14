#include "io/rmv/held_shot.hpp"
#include <cassert>
#include <limits>
int main(){
  io::rmv::HeldShotStability s;
  assert(!s.settled(1000000));
  // Recorded stationary speed noise crosses the former .03 threshold every
  // 200 ms, so its 300 ms acquisition can never complete, despite steady angles.
  std::uint64_t old_settled=0;bool old_acquired=false;
  for(unsigned i=0;i<=100;++i){
    const auto time=1000000+i*10000;
    const double velocity=i%20==0?-.03296661376953125:-.010990142822265625;
    if(std::abs(velocity)<=.03){
      if(!old_settled)old_settled=time;
      if(time-old_settled>=300000)old_acquired=true;
    }else old_settled=0;
    s.observe(time,.5+(i%2?0.0004:-.0004),-.23);
    if(i>=20)assert(s.settled(time));
  }
  assert(!old_acquired);
  assert(!s.settled(2050001)); // stale feedback
  s.observe(2050001,.5,-.23);assert(!s.settled(2050001)); // discontinuity resets window
  s.clear();
  for(unsigned i=0;i<=30;++i)s.observe(3000000+i*10000,i*.001,0);
  assert(!s.settled(3300000)); // actual motion must not qualify
  s.clear();
  for(unsigned i=0;i<=30;++i)s.observe(4000000+i*10000,std::remainder(3.1414+(i%2?0.0004:0),6.283185307179586),0);
  assert(s.settled(4300000)); // circular yaw crossing is not a jump
  s.observe(4310000,std::numeric_limits<double>::quiet_NaN(),0);
  assert(!s.settled(4310000));
}
