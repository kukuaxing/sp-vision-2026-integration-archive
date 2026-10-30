#include "fixed_target_world.hpp"
#include "fixed_target_plant.hpp"
#include "fixed_target_timing.hpp"
#include "fixed_target_cadence.hpp"
#include <cassert>
#include <iostream>

int main() {
  using namespace fixed_sim;
  const auto regular=capture_offsets(.1,50);
  assert(regular.size()==6&&regular[3]==3/50.);
  const auto uneven=capture_offsets(.1,50,{.02,.03},1);
  assert(uneven.size()==5&&std::abs(uneven[1]-.03)<1e-12&&std::abs(uneven[3]-.08)<1e-12);
  for(double dt:{0.,-.01,.501,static_cast<double>(NAN)}) {
    bool invalid=false;try{capture_offsets(.1,50,{dt});}catch(const std::invalid_argument&){invalid=true;}
    assert(invalid);
  }
  const auto room=horizontal_time_headroom(.02,4.56,.26,.28,3.,0.,.135,.2*pi/180);
  assert(room&&*room>.02);
  assert(std::abs(*room-*horizontal_time_headroom(-.02,-4.56,.26,.28,3.,0.,.135,.2*pi/180))<1e-12);
  assert(*horizontal_time_headroom(.1,4.56,.26,.28,3.,0.,.135,.2*pi/180)<*room);
  assert(*horizontal_time_headroom(.02,4.56,.28,.26,3.,0.,.135,.2*pi/180)==*room);
  assert(*horizontal_time_headroom(.02,4.56,.26,.28,3.,.003,.135,.2*pi/180)<*room);
  assert(!horizontal_time_headroom(NAN,4.56,.26,.28,3.,0.,.135,.2*pi/180));
  assert(!horizontal_time_headroom(0.,0.,.26,.28,3.,0.,.135,.2*pi/180));
  assert(!horizontal_time_headroom(0.,4.56,.26,.28,30.,0.,.135,.02));
  assert(centered_opportunity(NAN,0)); // disabled path preserves previous behavior
  assert(centered_opportunity(.02,3)&&centered_opportunity(-.02,3));
  assert(!centered_opportunity(.1,3)&&!centered_opportunity(-.1,3));
  assert(!centered_opportunity(NAN,3));
  assert(centered_opportunity(2*pi+.02,3));
  for(double bound:{-1.,21.,static_cast<double>(NAN)}) {
    bool invalid=false;try{centered_opportunity(0,bound);}catch(const std::invalid_argument&){invalid=true;}
    assert(invalid);
  }
  assert(preferred_radius_face(0,.26,.28)&&!preferred_radius_face(1,.26,.28));
  assert(preferred_radius_face(1,.28,.26)&&!preferred_radius_face(0,.28,.26));
  assert(preferred_radius_face(0,.26,.261)&&preferred_radius_face(1,.26,.261));
  assert(!preferred_radius_face(-1,.26,.28)&&!preferred_radius_face(4,.26,.28));
  assert(!preferred_radius_face(0,NAN,.28)&&!preferred_radius_face(0,.01,.28));
  assert(directional_time_offset(-4.,.01,0.,.02)==0.);
  assert(directional_time_offset(4.,.01,0.,.02)==.02);
  assert(directional_time_offset(NAN,.01,0.,.02)==.01);
  assert(directional_time_offset(0.,.01,0.,.02)==.01);
  assert(std::abs(*center_crossing_wait(-.04,4.,.015)-.01)<1e-12);
  assert(std::abs(*center_crossing_wait(.04,-4.,.015)-.01)<1e-12);
  assert(!center_crossing_wait(.04,4.,.015)); // do not move a shot into the past
  assert(!center_crossing_wait(-.08,4.,.015)); // never hold past the next capture
  assert(!center_crossing_wait(NAN,4.,.015));
  assert(!center_crossing_wait(-.01,0.,.015));
  assert(!center_crossing_wait(-.01,4.,.2));
  World world;world.omega=0;world.tilt=0;world.gravity=0;world.center.z()=0;
  Projectile shot;shot.velocity={22,0,0};
  const auto stationary=collide(world,shot,.001,.5);
  assert(stationary && stationary->face==0);
  assert(std::abs(stationary->time-(3-.26)/22)<1e-9);
  assert(std::abs(stationary->normal_speed-22)<1e-6);
  // A point outside a finite plate is a miss even if it crosses its plane.
  shot.origin.z()=.04;
  assert(!collide(world,shot,.001,.5));
  shot.origin={4,0,0};shot.velocity={-22,0,0};
  const auto opposite=collide(world,shot,.001,.5);
  assert(opposite && opposite->face==2); // rejects the front plate's back face
  shot.origin={3,0,0};
  assert(!collide(world,shot,.001,.5)); // from inside, front plate is back-facing
  for(int sign:{-1,1}) {
    world.omega=sign*4.56;shot.origin.setZero();shot.velocity={22,0,0};
    // Arrange the front face at the analytic impact time. Truth geometry and
    // interpolation must work in both directions and at either integration step.
    world.initial_phase=-world.omega*(3-.26)/22;
    const auto coarse=collide(world,shot,.002,.5),fine=collide(world,shot,.00025,.5);
    assert(coarse && fine && coarse->face==0 && fine->face==0);
    assert(std::abs(coarse->time-fine->time)<1e-8);
    assert(std::abs(fine->time-(3-.26)/22)<1e-8);
    world.initial_phase+=pi/4;
    assert(!collide(world,shot,.0005,.5));
  }
  // Include gravity and tilt, without using the aiming implementation to make
  // the oracle. Construct a ballistic ray crossing the known stationary center.
  world.omega=0;world.initial_phase=0;world.tilt=15*pi/180;world.gravity=9.7833;
  shot.velocity={22,0,.5*world.gravity*(3-.26)/22};
  const auto ballistic=collide(world,shot,.0005,.5);
  assert(ballistic && ballistic->face==0 && std::abs(ballistic->vertical)<1e-8);
  Score score;
  for(int i=0;i<29;++i){assert(score.issue());assert(score.hit());}
  assert(score.hp==10 && score.may_issue());
  assert(score.issue() && score.hit() && score.hp==0 && score.hits==30);
  assert(!score.hit());
  for(int i=30;i<50;++i)assert(score.issue()); // no oracle HP stop
  assert(!score.issue());
  Score exhausted;
  for(int i=0;i<50;++i)assert(exhausted.issue());
  assert(!exhausted.issue() && exhausted.issued==50);
  bool rejected=false;try{collide(world,shot,0,.5);}catch(const std::invalid_argument&){rejected=true;}
  assert(rejected);
  AngleServo servo;servo.tau=.05;servo.max_rate=1;servo.max_acceleration=2;servo.command(.5);
  servo.advance(.1);assert(servo.angle>0 && servo.angle<=.011 && servo.rate<=.201);
  servo.advance(2);assert(std::abs(servo.angle-.5)<.001);
  servo.command(-.5);const double before=servo.angle;servo.advance(.01);
  assert(std::abs(servo.angle-before)<.01);
  AngleServo ideal;ideal.command(.25);assert(ideal.angle==.25);
  MuzzleSample launch{.33,.1,.04,22};const auto ray=launch.projectile();
  assert(ray.muzzle_time==.33 && std::abs(ray.velocity.norm()-22)<1e-10);
  std::cout<<"fixed target geometry, continuous collision, both directions and scoring passed\n";
}
