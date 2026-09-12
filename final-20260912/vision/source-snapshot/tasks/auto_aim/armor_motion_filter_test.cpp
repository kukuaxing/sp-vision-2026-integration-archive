#include "armor_motion_filter.hpp"

#include <cassert>
#include <iostream>

int main()
{
  auto_aim::ArmorMotionFilter filter;
  constexpr double dt = 0.01;
  const Eigen::Vector3d velocity(1.2, -0.25, 0.05);
  Eigen::Vector3d position(4.0, 0.2, 0.1);
  filter.reset(position);
  for (int i = 1; i <= 100; ++i) {
    filter.predict(dt);
    position += velocity * dt;
    const Eigen::Vector3d noise(
      0.01 * std::sin(i * 0.31), 0.01 * std::cos(i * 0.27),
      0.005 * std::sin(i * 0.19));
    filter.update(position + noise);
  }
  assert(filter.trusted());
  assert((filter.position() - position).norm() < 0.04);
  assert((filter.velocity() - velocity).norm() < 0.20);

  const Eigen::Vector3d before_long_prediction = filter.position();
  const Eigen::Vector3d velocity_before_long_prediction = filter.velocity();
  filter.predict(0.40);
  assert((filter.position() - before_long_prediction -
          velocity_before_long_prediction * 0.40).norm() < 1e-6);
  position += velocity * 0.40;
  filter.update(position);

  const int rejected_before = filter.rejected_updates();
  assert(!filter.update(position + Eigen::Vector3d(3.0, 2.0, 1.0)));
  assert(filter.rejected_updates() == rejected_before + 1);
  assert(!filter.trusted());

  // A face that has not been observed for longer than the prediction horizon
  // must not keep driving the aim point with a stale constant velocity.
  filter.update(position);
  filter.predict(0.76);
  assert(!filter.trusted());

  // Repeated contradictions reinitialize rather than leaving a permanently
  // trusted, runaway state behind.
  auto_aim::ArmorMotionFilter recovery;
  recovery.reset(Eigen::Vector3d::Zero());
  for (int i = 0; i < 20; ++i) {
    recovery.predict(dt);
    assert(recovery.update(Eigen::Vector3d::Zero()));
  }
  const Eigen::Vector3d reacquired(4.0, -3.0, 1.0);
  assert(!recovery.update(reacquired));
  assert(!recovery.update(reacquired));
  assert(!recovery.update(reacquired));
  assert(!recovery.trusted());
  assert((recovery.position() - reacquired).norm() < 1e-9);
  std::cout << "armor_motion_filter_test: PASS position_error="
            << (filter.position() - position).norm() << " velocity_error="
            << (filter.velocity() - velocity).norm() << std::endl;
  return 0;
}
