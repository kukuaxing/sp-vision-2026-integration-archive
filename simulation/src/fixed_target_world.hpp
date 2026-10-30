#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace fixed_sim {
constexpr double pi = 3.14159265358979323846;
struct Plate {
  Eigen::Vector3d center, tangent, vertical, normal;
  double phase;
};
struct World {
  Eigen::Vector3d center{3, 0, .03};
  std::array<double,4> radii{{.26,.28,.26,.28}};
  double omega=4.56, initial_phase=0, height_delta=0, epoch=0;
  double width=.135, height=.055, tilt=15*pi/180, gravity=9.7833;
  Plate plate(int face, double time) const {
    const double a=std::remainder(initial_phase+omega*(time-epoch)+face*pi/2,2*pi);
    const double c=std::cos(a),s=std::sin(a);
    return {{center.x()-radii.at(face)*c,center.y()-radii.at(face)*s,
             center.z()+(face%2?height_delta:0)},
            {-s,c,0},{std::sin(tilt)*c,std::sin(tilt)*s,std::cos(tilt)},
            {-std::cos(tilt)*c,-std::cos(tilt)*s,std::sin(tilt)},a};
  }
  std::array<Eigen::Vector3d,4> corners(int face,double time,double lightbar_length) const {
    const auto p=plate(face,time);
    return {{p.center+p.tangent*width/2+p.vertical*lightbar_length/2,
             p.center-p.tangent*width/2+p.vertical*lightbar_length/2,
             p.center-p.tangent*width/2-p.vertical*lightbar_length/2,
             p.center+p.tangent*width/2-p.vertical*lightbar_length/2}};
  }
};
struct Projectile {
  double muzzle_time=0;
  Eigen::Vector3d origin=Eigen::Vector3d::Zero(), velocity=Eigen::Vector3d::Zero();
  Eigen::Vector3d position(double elapsed,double gravity) const {
    Eigen::Vector3d result=origin+velocity*elapsed;
    result.z()-=.5*gravity*elapsed*elapsed;
    return result;
  }
};
struct Hit { int face; double time, horizontal, vertical, normal_speed; Eigen::Vector3d point; };
// Independent continuous geometry: bracket then bisect a moving plane crossing.
// Neither the predictor's selected face nor its predicted hit time is consulted.
inline std::optional<Hit> collide(const World& world,const Projectile& shot,double step,double limit) {
  if (!(step>0 && step<=.01 && limit>0 && limit<=2))
    throw std::invalid_argument("invalid collision integration bounds");
  std::optional<Hit> earliest;
  for(int face=0;face<4;++face) {
    auto distance=[&](double dt) {
      const auto p=world.plate(face,shot.muzzle_time+dt);
      return (shot.position(dt,world.gravity)-p.center).dot(p.normal);
    };
    double previous=distance(0), left=0;
    for(double right=std::min(step,limit);left<limit;right=std::min(right+step,limit)) {
      const double current=distance(right);
      if(previous>0 && current<=0) {
        double lo=left,hi=right;
        for(int n=0;n<32;++n) {
          const double mid=(lo+hi)/2;
          if(distance(mid)>0)lo=mid;else hi=mid;
        }
        const double dt=(lo+hi)/2,time=shot.muzzle_time+dt;
        const auto p=world.plate(face,time);const auto point=shot.position(dt,world.gravity);
        const Eigen::Vector3d delta=point-p.center;
        const double u=delta.dot(p.tangent),v=delta.dot(p.vertical);
        if(std::abs(u)<=world.width/2 && std::abs(v)<=world.height/2) {
          const double h=1e-6;
          const double approach=(distance(dt+h)-distance(dt-h))/(2*h);
          Hit hit{face,time,u,v,-approach,point};
          if(!earliest || hit.time<earliest->time)earliest=hit;
          break;
        }
      }
      previous=current;left=right;
      if(right>=limit)break;
    }
  }
  return earliest;
}
struct Score {
  int hp=300, damage=10, ammo=50, issued=0, hits=0;
  // The real host does not receive referee HP. Full-budget evaluation must
  // never feed a truth-only kill event back into the shooting controller.
  bool may_issue() const {return issued<ammo;}
  bool issue() {if(!may_issue())return false;++issued;return true;}
  bool hit() {if(hp<=0)return false;hp=std::max(0,hp-damage);++hits;return true;}
};
}
