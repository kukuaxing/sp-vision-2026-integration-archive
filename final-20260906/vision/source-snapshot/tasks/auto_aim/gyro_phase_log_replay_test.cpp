#include "gyro_phase_scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

namespace
{
void cross(auto_aim::GyroPhaseScheduler & scheduler, double t, double omega)
{
  scheduler.observe(t - 0.012, -0.70, omega);
  scheduler.observe(t - 0.004, -0.02, omega);
  scheduler.observe(t + 0.004, +0.02, omega);
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: gyro_phase_log_replay_test LOG\n";
    return 2;
  }

  std::ifstream input(argv[1]);
  if (!input) return 2;
  const std::regex pattern(
    R"(\[GYROPLL\].*sample=([0-9.]+) ms.*ekf_omega=([+-]?[0-9.]+))");
  std::vector<std::pair<double, double>> observations;
  std::string line;
  std::smatch match;
  while (std::getline(input, line)) {
    if (std::regex_search(line, match, pattern)) {
      observations.emplace_back(
        std::stod(match[1].str()) / 1000.0, std::stod(match[2].str()));
    }
  }
  if (observations.size() < 8) return 2;

  auto_aim::GyroPhaseScheduler scheduler;
  double t = 1.0;
  cross(scheduler, t, observations.front().second);
  int valid = 0;
  int safe = 0;
  int rejected = 0;
  double uncertainty_sum_ms = 0.0;
  double uncertainty_max_ms = 0.0;
  int uncertainty_count = 0;
  for (const auto & [sample_s, omega] : observations) {
    t += sample_s;
    const auto before_updates = scheduler.lock_updates();
    cross(scheduler, t, omega);
    rejected += scheduler.lock_updates() == before_updates ? 1 : 0;
    if (scheduler.locked() && std::isfinite(scheduler.phase_uncertainty_s())) {
      const double uncertainty_ms = scheduler.phase_uncertainty_s() * 1000.0;
      uncertainty_sum_ms += uncertainty_ms;
      uncertainty_max_ms = std::max(uncertainty_max_ms, uncertainty_ms);
      ++uncertainty_count;
      const auto solution = scheduler.solve(
        t + 0.010, 0.154, 0.134, 0.174, 0.155, 0.005,
        0.28, 0.135, 0.005, 1.0 * M_PI / 180.0);
      if (solution.valid) {
        ++valid;
        safe += solution.interval_safe ? 1 : 0;
      }
    }
  }

  std::cout << "samples=" << observations.size()
            << " locked=" << scheduler.locked()
            << " period_ms=" << scheduler.face_period_s() * 1000.0
            << " rejected=" << rejected
            << " uncertainty_avg_ms="
            << uncertainty_sum_ms / std::max(1, uncertainty_count)
            << " uncertainty_max_ms=" << uncertainty_max_ms
            << " interval_safe=" << safe << "/" << valid << '\n';
  return scheduler.locked() && valid > 0 ? 0 : 1;
}
