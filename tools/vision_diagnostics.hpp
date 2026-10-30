#pragma once
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include "tools/logger.hpp"

namespace tools {
class VisionDiagnostics {
public:
  explicit VisionDiagnostics(const YAML::Node & config) {
    stride_ = config["vision_diagnostics_stride"]
      ? std::clamp(config["vision_diagnostics_stride"].as<int>(), 1, 50) : 1;
    if (!config["vision_diagnostics_enabled"] ||
        !config["vision_diagnostics_enabled"].as<bool>()) return;
    limit_ = config["vision_diagnostics_max_frames"]
      ? std::clamp(config["vision_diagnostics_max_frames"].as<int>(), 1, 20000) : 3000;
    std::error_code error;
    std::filesystem::create_directories("logs/diagnostics", error);
    if (error) { logger()->warn("[DIAGNOSTICS] mkdir failed: {}", error.message()); return; }
    const auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    path_ = "logs/diagnostics/vision-" + std::to_string(stamp) + ".jsonl";
    output_.open(path_, std::ios::out);
    if (!output_) { logger()->warn("[DIAGNOSTICS] cannot open {}", path_); return; }
    logger()->info("[DIAGNOSTICS] {} max_frames={}", path_, limit_);
  }
  void begin_frame() { selected_ = (frame_++ % stride_ == 0); }
  bool enabled() const { return selected_ && output_.is_open() && count_ < limit_; }
  void write(const nlohmann::json & frame) {
    if (!enabled()) return;
    output_ << frame.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace) << '\n';
    if (++count_ % 32 == 0) output_.flush();
    if (!output_) { logger()->warn("[DIAGNOSTICS] write failed: {}", path_); output_.close(); }
    else if (count_ == limit_) { output_.flush(); output_.close();
      logger()->info("[DIAGNOSTICS] capture complete: {} frames in {}", count_, path_); }
  }
private:
  std::ofstream output_;
  std::string path_;
  int count_ = 0, limit_ = 0;
  int stride_ = 1;
  std::uint64_t frame_ = 0;
  bool selected_ = true;
};
}  // namespace tools
