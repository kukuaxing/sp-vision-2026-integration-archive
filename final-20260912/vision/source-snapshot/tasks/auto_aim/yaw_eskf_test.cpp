#include "yaw_eskf.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace
{
double wrap(double angle) { return std::remainder(angle, 2.0 * M_PI); }
}

int main()
{
  constexpr double dt = 0.01;
  constexpr double true_rate = 4.5;
  constexpr double measurement_variance =
    (8.0 * M_PI / 180.0) * (8.0 * M_PI / 180.0);

  auto_aim::YawESKF filter;
  double true_yaw = 3.10;
  filter.reset(wrap(true_yaw));
  for (int i = 0; i < 500; ++i) {
    true_yaw += true_rate * dt;
    filter.predict(dt);
    const int face_id = (i / 35) % 4;
    const double noise = 0.030 * std::sin(0.37 * static_cast<double>(i));
    const double armor_yaw = wrap(
      true_yaw + face_id * M_PI / 2.0 + noise);
    const double body_measurement = wrap(
      armor_yaw - face_id * M_PI / 2.0);
    filter.update(body_measurement, measurement_variance);
  }

  std::cout << "steady rate=" << filter.rate()
            << " angle_error=" << wrap(filter.angle() - true_yaw)
            << " alpha=" << filter.acceleration() << std::endl;
  assert(filter.trusted());
  assert(std::abs(filter.rate() - true_rate) < 0.20);
  assert(std::abs(wrap(filter.angle() - true_yaw)) < 0.05);

  const double rate_before_outlier = filter.rate();
  filter.predict(dt);
  const auto outlier = filter.update(
    wrap(true_yaw + M_PI / 2.0), measurement_variance);
  assert(!outlier.accepted);
  assert(std::abs(filter.rate() - rate_before_outlier) < 0.5);
  assert(filter.covariance().isApprox(filter.covariance().transpose(), 1e-10));
  assert((filter.covariance().diagonal().array() > 0.0).all());

  // A separate filter must follow a genuine speed ramp without confusing it
  // with a 90-degree armor hand-over.
  auto_aim::YawESKF accelerating;
  double ramp_yaw = 0.2;
  double ramp_rate = 1.0;
  accelerating.reset(ramp_yaw, ramp_rate);
  for (int i = 0; i < 300; ++i) {
    const double acceleration = i < 100 ? 3.0 : 0.0;
    ramp_yaw += ramp_rate * dt + 0.5 * acceleration * dt * dt;
    ramp_rate += acceleration * dt;
    accelerating.predict(dt);
    const int face_id = (i / 40) % 4;
    const double armor_yaw = wrap(ramp_yaw + face_id * M_PI / 2.0);
    accelerating.update(
      wrap(armor_yaw - face_id * M_PI / 2.0),
      measurement_variance);
  }
  assert(accelerating.trusted());
  assert(std::abs(accelerating.rate() - ramp_rate) < 0.25);

  // A brief association error must be treated as an equivalent face, not as a
  // physical quarter-turn of the vehicle body.
  auto_aim::YawESKF handover;
  double handover_yaw = -0.4;
  constexpr double handover_rate = 1.6;
  handover.reset(handover_yaw, handover_rate);
  for (int i = 0; i < 240; ++i) {
    handover_yaw += handover_rate * dt;
    handover.predict(dt);
    const int true_face_id = (i / 45) % 4;
    const int reported_face_id = (i % 47 == 0) ? (true_face_id + 1) % 4 : true_face_id;
    const double armor_yaw = wrap(handover_yaw + true_face_id * M_PI / 2.0);
    const double reported_body_yaw = wrap(
      armor_yaw - reported_face_id * M_PI / 2.0);
    const auto result = handover.update_equivalent(
      reported_body_yaw, measurement_variance, M_PI / 2.0);
    assert(result.accepted);
  }
  assert(std::abs(handover.rate() - handover_rate) < 0.20);
  assert(std::abs(wrap(handover.angle() - handover_yaw)) < 0.05);

  // A low covariance alone must not make an impossible association-induced
  // rotation rate eligible to drive model selection or prediction.
  auto_aim::YawESKF::Config physical_config;
  physical_config.max_trusted_abs_rate_rad_s = 8.0;
  physical_config.max_trusted_abs_acceleration_rad_s2 = 30.0;
  auto_aim::YawESKF impossible(physical_config);
  double impossible_yaw = 0.0;
  impossible.reset(impossible_yaw, 12.0);
  for (int i = 0; i < 200; ++i) {
    impossible_yaw += 12.0 * dt;
    impossible.predict(dt);
    impossible.update(wrap(impossible_yaw), measurement_variance);
  }
  assert(!impossible.trusted());

  // The same physical bound must actively recover a runaway alias and then
  // converge again when plausible rotating-target observations resume.
  auto_aim::YawESKF recovered(physical_config);
  double recovered_yaw = 0.0;
  recovered.reset(recovered_yaw, -100.0);
  for (int i = 0; i < 400; ++i) {
    recovered_yaw += true_rate * dt;
    recovered.predict(dt);
    recovered.update_equivalent(
      wrap(recovered_yaw), measurement_variance, M_PI / 2.0);
  }
  assert(recovered.trusted());
  assert(std::abs(recovered.rate() - true_rate) < 0.25);

  std::cout << "yaw_eskf_test: PASS rate=" << filter.rate()
            << " angle_error=" << wrap(filter.angle() - true_yaw)
            << " rejected=" << filter.rejected_updates() << '\n';
  return 0;
}
