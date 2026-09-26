// Deterministic, synthetic comparison using the actual production headers.
// No detector, PnP, camera, serial I/O, target association or hit-rate model.
#include "tasks/auto_aim/yaw_eskf.hpp"
#include "tasks/auto_aim/armor_motion_filter.hpp"
#include "tasks/auto_aim/robust_range_filter.hpp"
#include "benchmark_config.hpp"  // generated from the sole standard.yaml

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

namespace {
constexpr double dt = 0.01;
constexpr double horizon = 0.40;
constexpr double pi = 3.14159265358979323846;
constexpr double nan = std::numeric_limits<double>::quiet_NaN();

// An explicit generator avoids implementation-dependent normal_distribution
// sequences. Twelve independent uniforms give bounded, near-normal noise.
struct Noise {
  std::uint32_t state;
  double sample(double sigma) {
    double sum = 0;
    for (int i = 0; i < 12; ++i) {
      state = 1664525u * state + 1013904223u;
      sum += (static_cast<double>(state) + 0.5) / 4294967296.0;
    }
    return (sum - 6.0) * sigma;
  }
};

double positive(double x) { return std::max(0.0, x); }
double wrap(double angle) { return std::remainder(angle, 2.0 * pi); }
bool missing(const std::string & scene, int i) {
  return scene == "dropout" && ((i >= 200 && i < 280) || i % 7 == 0);
}
std::string phase(double t) {
  if (t < 1.0) return "warmup";
  if (t < 2.0) return "before";
  if (t < 4.0) return "stress";
  if (t < 6.0) return "recovery";
  return "late";
}
void row(const std::string & family, const std::string & scene, int seed,
         int i, const std::string & method, bool observed, bool accepted,
         bool valid, double error, double rate_error, double forecast_error) {
  std::cout << family << ',' << scene << ',' << seed << ',' << i * dt << ','
            << phase(i * dt) << ',' << method << ',' << observed << ','
            << accepted << ',' << valid << ',' << error << ',' << rate_error
            << ',' << forecast_error << ','
            << (family == "yaw" ? std::remainder(error, pi / 2) : nan) << ','
            << (family == "yaw" ? std::remainder(forecast_error, pi / 2) : nan) << '\n';
}

double yaw_truth(const std::string & scene, double t) {
  if (scene == "rate_ramp")
    return 0.2 + t + 1.5 * std::pow(positive(t - 2), 2) -
      1.5 * std::pow(positive(t - 3), 2);
  return (scene == "angle_wrap" ? 3.0 : 0.2) + 2.0 * t;
}
double yaw_rate(const std::string & scene, double t) {
  return scene == "rate_ramp"
    ? 1.0 + 3.0 * positive(t - 2) - 3.0 * positive(t - 3) : 2.0;
}

void yaw_benchmark(const std::string & scene, int seed) {
  Noise noise{static_cast<std::uint32_t>(seed)};
  auto_aim::YawESKF filter(benchmark_yaw_config());
  bool initialized = false;
  double baseline_angle = 0, baseline_rate = 0;
  for (int i = 0; i < 800; ++i) {
    const double t = i * dt;
    if (initialized) { filter.predict(dt); baseline_angle += baseline_rate * dt; }
    const bool observed = !missing(scene, i);
    bool accepted = false;
    // Generate noise on every tick, even when that tick will be missing.
    double measured = wrap(yaw_truth(scene,
      scene == "measurement_delay" ? t - 0.06 : t) + noise.sample(0.035));
    if (scene == "outliers" && i >= 100 && i % 41 == 0) measured = wrap(measured + 0.8);
    if (observed) {
      if (!initialized) {
        filter.reset(measured);
        baseline_angle = measured;
        initialized = true;
        accepted = true;
      } else {
        accepted = filter.update_equivalent(measured, 0.035 * 0.035, pi / 2).accepted;
        // A basic constant-velocity alpha-beta observer. It gets the same
        // equivalent-face information as the production filter.
        const double innovation = std::remainder(measured - baseline_angle, pi / 2);
        baseline_angle += 0.35 * innovation;
        baseline_rate += 0.04 * innovation / dt;
      }
    }
    if (!initialized) {
      row("yaw", scene, seed, i, "yaw_eskf", observed, false, false, nan, nan, nan);
      row("yaw", scene, seed, i, "alpha_beta", observed, false, false, nan, nan, nan);
      continue;
    }
    auto forecast = filter;
    forecast.predict(horizon);
    row("yaw", scene, seed, i, "yaw_eskf", observed, accepted, filter.trusted(),
        wrap(filter.angle() - yaw_truth(scene, t)), filter.rate() - yaw_rate(scene, t),
        wrap(forecast.angle() - yaw_truth(scene, t + horizon)));
    row("yaw", scene, seed, i, "alpha_beta", observed, observed, true,
        wrap(baseline_angle - yaw_truth(scene, t)), baseline_rate - yaw_rate(scene, t),
        wrap(baseline_angle + horizon * baseline_rate - yaw_truth(scene, t + horizon)));
  }
}

Eigen::Vector3d armor_truth(const std::string & scene, double t) {
  double x = 4.0 + 0.2 * t;
  if (scene == "velocity_ramp")
    x += 0.5 * std::pow(positive(t - 2), 2) - 0.5 * std::pow(positive(t - 3), 2);
  if (scene == "target_switch" && t >= 2.5) x += 2.0;
  return {x, 0.2 + 0.4 * std::sin(0.6 * t), 0.3};
}
Eigen::Vector3d armor_velocity(const std::string & scene, double t) {
  const double extra = scene == "velocity_ramp" ? positive(t - 2) - positive(t - 3) : 0;
  return {0.2 + extra, 0.24 * std::cos(0.6 * t), 0.0};
}
void armor_benchmark(const std::string & scene, int seed) {
  Noise noise{static_cast<std::uint32_t>(seed)};
  auto_aim::ArmorMotionFilter filter;
  bool initialized = false;
  Eigen::Vector3d baseline_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d baseline_velocity = Eigen::Vector3d::Zero();
  for (int i = 0; i < 800; ++i) {
    const double t = i * dt;
    if (initialized) { filter.predict(dt); baseline_position += dt * baseline_velocity; }
    const bool observed = !missing(scene, i);
    bool accepted = false;
    Eigen::Vector3d measured = armor_truth(scene, t);
    for (int axis = 0; axis < 3; ++axis) measured[axis] += noise.sample(0.02);
    if (scene == "outliers" && i >= 100 && i % 41 == 0)
      measured += Eigen::Vector3d(1.5, -1.2, 0.5);
    if (observed) {
      if (!initialized) {
        filter.reset(measured);
        baseline_position = measured;
        initialized = true;
        accepted = true;
      } else {
        accepted = filter.update(measured);
        const Eigen::Vector3d innovation = measured - baseline_position;
        baseline_position += 0.35 * innovation;
        baseline_velocity += 0.04 * innovation / dt;
      }
    }
    if (!initialized) {
      row("armor", scene, seed, i, "armor_cv", observed, false, false, nan, nan, nan);
      row("armor", scene, seed, i, "alpha_beta", observed, false, false, nan, nan, nan);
      continue;
    }
    auto forecast = filter;
    forecast.predict(horizon);
    row("armor", scene, seed, i, "armor_cv", observed, accepted, filter.trusted(),
        (filter.position() - armor_truth(scene, t)).norm(),
        (filter.velocity() - armor_velocity(scene, t)).norm(),
        (forecast.position() - armor_truth(scene, t + horizon)).norm());
    row("armor", scene, seed, i, "alpha_beta", observed, observed, true,
        (baseline_position - armor_truth(scene, t)).norm(),
        (baseline_velocity - armor_velocity(scene, t)).norm(),
        (baseline_position + horizon * baseline_velocity - armor_truth(scene, t + horizon)).norm());
  }
}

double range_truth(const std::string & scene, double t) {
  if (scene == "velocity_ramp")
    return 4.0 + 0.2 * t + 0.5 * std::pow(positive(t - 2), 2) -
      0.5 * std::pow(positive(t - 3), 2);
  return 4.0 + 0.2 * t;
}
void range_benchmark(const std::string & scene, int seed) {
  Noise noise{static_cast<std::uint32_t>(seed)};
  auto_aim::RobustRangeFilter filter(benchmark_range_config());
  bool initialized = false;
  double model_ema = nan, raw_ema = nan, previous_t = 0;
  for (int i = 0; i < 800; ++i) {
    const double t = i * dt;
    const bool observed = !missing(scene, i);
    double model = range_truth(scene, t) + noise.sample(0.05);
    double raw = range_truth(scene, t) + noise.sample(0.12);
    if (scene == "raw_outliers" && i >= 100 && i % 41 == 0) raw += i % 2 ? 2.0 : -2.0;
    if (scene == "model_bias" && i >= 200 && i < 600) model += 0.35;
    const auto result = filter.update(t, observed ? model : nan, observed ? raw : nan);
    if (observed) {
      const double alpha = 1.0 - std::exp(-(t - previous_t) / 0.08);
      model_ema = initialized ? model_ema + alpha * (model - model_ema) : model;
      raw_ema = initialized ? raw_ema + alpha * (raw - raw_ema) : raw;
      initialized = true;
      previous_t = t;
    }
    row("range", scene, seed, i, "robust_range", observed, result.valid, result.valid,
        result.valid ? result.range_m - range_truth(scene, t) : nan, nan, nan);
    row("range", scene, seed, i, "model_ema", observed, observed, observed,
        observed ? model_ema - range_truth(scene, t) : nan, nan, nan);
    row("range", scene, seed, i, "raw_ema", observed, observed, observed,
        observed ? raw_ema - range_truth(scene, t) : nan, nan, nan);
  }
}
}  // namespace

int main() {
  std::cout << std::setprecision(12);
  std::cout << "family,scenario,seed,time_s,phase,method,observed,accepted,valid,error,rate_error,forecast_error,equivalent_error,equivalent_forecast_error\n";
  for (int seed : {17, 2026, 98173}) {
    for (const std::string scene : {"steady", "rate_ramp", "angle_wrap", "dropout", "outliers", "measurement_delay"})
      yaw_benchmark(scene, seed);
    for (const std::string scene : {"steady", "velocity_ramp", "dropout", "outliers", "target_switch"})
      armor_benchmark(scene, seed);
    for (const std::string scene : {"steady", "velocity_ramp", "dropout", "raw_outliers", "model_bias"})
      range_benchmark(scene, seed);
  }
}
