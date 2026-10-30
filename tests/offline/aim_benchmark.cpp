// Deterministic benchmark of the production Aimer, with no device/mission I/O.
// Save outputs as well as timing so optimizations can be checked for equivalence.
#include "tasks/auto_aim/aimer.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

using Clock = std::chrono::steady_clock;
int main(int argc, char ** argv) {
  if (argc != 3) { std::cerr << "usage: aim_benchmark CONFIG OUTPUT_JSON\n"; return 2; }
  nlohmann::json report = {{"device_io", false}, {"cases", nlohmann::json::array()}};
  for (double omega : {0., -.4, .4, -4.56, 4.56}) {
    const bool rigid = std::abs(omega) > 1.;
    std::vector<std::list<auto_aim::Target>> inputs;
    for (int k = 0; k < 240; ++k) {
      auto_aim::Target target(3. + (k % 7) * .35, omega, .2, .025);
      if (rigid) {
        auto state = target.ekf().x;
        state[4] = .12; state[6] = k * (2 * M_PI / 240.); state[9] = .025;
        target = *target.with_fixed_center_prediction(state);
      }
      inputs.push_back({target});
    }
    auto_aim::Aimer aimer(argv[1]);
    std::vector<double> times;
    nlohmann::json reference = nlohmann::json::array();
    for (int round = -2; round < 12; ++round) {
      for (const auto & input : inputs) {
        auto_aim::AimPredictionTrace trace;
        const auto start = Clock::now();
        const auto command = aimer.aim(input, Clock::time_point{}, 22., false, &trace,
          rigid ? auto_aim::AimGeometry::whole_ekf : auto_aim::AimGeometry::existing, rigid);
        const auto end = Clock::now();
        if (round >= 0) times.push_back(std::chrono::duration<double, std::micro>(end-start).count());
        if (round == 0) reference.push_back({{"control", command.control}, {"yaw", command.yaw},
          {"pitch", command.pitch}, {"face", trace.face_id}, {"converged", trace.converged},
          {"prediction_ns", trace.prediction_ns}, {"iterations", trace.iterations},
          {"point", {aimer.debug_aim_point.xyza[0], aimer.debug_aim_point.xyza[1],
                     aimer.debug_aim_point.xyza[2], aimer.debug_aim_point.xyza[3]}}});
      }
    }
    std::sort(times.begin(), times.end());
    report["cases"].push_back({{"omega", omega}, {"rigid", rigid}, {"calls", times.size()},
      {"p50_us", times[times.size()/2]}, {"p95_us", times[times.size()*95/100]},
      {"reference", reference}});
  }
  std::ofstream output(argv[2]); output << report.dump(2) << '\n';
  if (!output) return 1;
}
