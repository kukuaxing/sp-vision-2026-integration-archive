// Explicit file input only: run the production YOLO26 detector without camera or control I/O.
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <openvino/openvino.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "tasks/auto_aim/yolos/yolo26.hpp"

// This executable routes existing production log calls to stderr. The Python launcher
// writes them into logs/replay; no logger file is created beside the captured images.
namespace tools {
std::shared_ptr<spdlog::logger> logger()
{
  static auto value = [] {
    auto sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
    auto result = std::make_shared<spdlog::logger>("detector_replay", sink);
    result->set_level(spdlog::level::info);
    return result;
  }();
  return value;
}
}  // namespace tools

namespace {
namespace fs = std::filesystem;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

std::uint64_t now_us()
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count());
}

int positive(const std::string & value, const std::string & name, int maximum)
{
  std::size_t used = 0;
  const long long result = std::stoll(value, &used);
  if (used != value.size() || result < 1 || result > maximum)
    throw std::invalid_argument(name + " outside allowed range");
  return static_cast<int>(result);
}

struct Arguments {
  fs::path config, input, image_list, output, root;
  int max_frames = 8;
  int stride = 1;
  bool blank = false;
};

Arguments parse(int argc, char ** argv)
{
  Arguments args;
  for (int index = 1; index < argc; ++index) {
    const std::string option(argv[index]);
    if (option == "--blank") {
      args.blank = true;
      continue;
    }
    if (index + 1 >= argc) throw std::invalid_argument("missing value for " + option);
    const std::string value(argv[++index]);
    if (option == "--config") args.config = value;
    else if (option == "--input") args.input = value;
    else if (option == "--image-list") args.image_list = value;
    else if (option == "--output") args.output = value;
    else if (option == "--repo-root") args.root = value;
    else if (option == "--max-frames") args.max_frames = positive(value, option, 1000);
    else if (option == "--stride") args.stride = positive(value, option, 10000);
    else throw std::invalid_argument("unknown option: " + option);
  }
  const int inputs = static_cast<int>(args.blank) + !args.input.empty() + !args.image_list.empty();
  if (args.config.empty() || args.output.empty() || inputs != 1)
    throw std::invalid_argument("required: --config FILE --output NEW_DIR and one of --input FILE / --image-list JSON / --blank");
  args.config = fs::absolute(args.config);
  args.output = fs::absolute(args.output);
  args.root = args.root.empty() ? args.config.parent_path().parent_path() : fs::absolute(args.root);
  if (!args.input.empty()) args.input = fs::absolute(args.input);
  if (!args.image_list.empty()) args.image_list = fs::absolute(args.image_list);
  if (!fs::is_regular_file(args.config) || (!args.input.empty() && !fs::is_regular_file(args.input)) ||
      (!args.image_list.empty() && !fs::is_regular_file(args.image_list)))
    throw std::invalid_argument("config and input must be existing local regular files");
  if (fs::exists(args.output)) throw std::invalid_argument("refusing to replace existing output directory");
  return args;
}

fs::path absolute_asset(const YAML::Node & config, const std::string & key, const fs::path & root)
{
  const fs::path value(config[key].as<std::string>());
  const auto path = fs::absolute(value.is_absolute() ? value : root / value);
  if (!fs::is_regular_file(path)) throw std::runtime_error("missing " + key + ": " + path.string());
  return path;
}

Json detection(const auto_aim::Armor & armor)
{
  if (armor.points.size() != 4 || !std::isfinite(armor.confidence))
    throw std::runtime_error("detector returned non-finite confidence or non-quadrilateral points");
  Json corners = Json::array();
  for (const auto & point : armor.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y))
      throw std::runtime_error("detector returned non-finite keypoint");
    corners.push_back({point.x, point.y});
  }
  return {{"label", auto_aim::ARMOR_NAMES.at(static_cast<std::size_t>(armor.name))},
          {"color", auto_aim::COLORS.at(static_cast<std::size_t>(armor.color))},
          {"armor_type", auto_aim::ARMOR_TYPES.at(static_cast<std::size_t>(armor.type))},
          {"class_id", armor.class_id}, {"confidence", armor.confidence},
          {"keypoint_confidence", armor.keypoint_confidence},
          {"color_score", armor.color_score}, {"color_source", armor.color_source},
          {"corners_px", corners}};
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto args = parse(argc, argv);
    auto config = YAML::LoadFile(args.config.string());
    if (config["yolo_name"].as<std::string>() != "yolo26")
      throw std::invalid_argument("this replay executable requires the formal YOLO26 configuration");
    const auto model = absolute_asset(config, "yolo26_model_path", args.root);
    const auto classifier = absolute_asset(config, "classify_model", args.root);
    config["yolo26_model_path"] = model.generic_string();
    config["classify_model"] = classifier.generic_string();
    config["device"] = "CPU";
    config["infer_threads"] = 1;
    config["classifier_device"] = "CPU";
    config["classifier_infer_threads"] = 1;
    config["yolo26_bind_big_cores"] = false;
    config["yolo26_save_samples"] = false;
    config["detector_debug"] = false;
    config["use_async_inference"] = false;
    fs::create_directories(args.output.parent_path());
    if (!fs::create_directory(args.output)) throw std::runtime_error("cannot create new output directory");
    fs::create_directory(args.output / "images");
    const auto runtime_path = args.output / "runtime.yaml";
    {
      std::ofstream runtime(runtime_path, std::ios::binary);
      if (!(runtime << config << '\n')) throw std::runtime_error("cannot write derived runtime configuration");
    }
    // Keep legacy constructor-created empty sample directories inside this explicit capture.
    fs::current_path(args.output);
    cv::setNumThreads(1);
    cv::ocl::setUseOpenCL(false);
    auto_aim::YOLO26 detector(runtime_path.string(), false);
    if (args.blank && !detector.detect(cv::Mat(), -1).empty())
      throw std::runtime_error("empty-image regression returned a detection");

    cv::Mat still;
    cv::VideoCapture video;
    Json image_list = Json::array();
    const bool list_input = !args.image_list.empty();
    const bool image_input = args.blank || (!list_input && cv::haveImageReader(args.input.string()));
    if (list_input) {
      std::ifstream input(args.image_list, std::ios::binary);
      input >> image_list;
      if (!image_list.is_array() || image_list.empty() || image_list.size() > 1000)
        throw std::invalid_argument("image-list must contain 1..1000 frame objects");
      int previous = -1;
      for (const auto & entry : image_list) {
        const int index = entry.at("source_index").get<int>();
        const fs::path path(entry.at("path").get<std::string>());
        if (index <= previous || !path.is_absolute() || !fs::is_regular_file(path))
          throw std::invalid_argument("image-list paths must be absolute files and indices strictly increasing");
        previous = index;
        if (!entry.at("video_pts_us").is_null() && !entry.at("video_pts_us").is_number_unsigned())
          throw std::invalid_argument("image-list PTS must be unsigned microseconds or null");
      }
    }
    if (args.blank) still = cv::Mat::zeros(640, 640, CV_8UC3);
    else if (image_input) {
      still = cv::imread(args.input.string(), cv::IMREAD_COLOR);
      if (still.empty()) throw std::runtime_error("image decoder returned no pixels");
    } else if (!list_input) {
      // Refuse network/camera backends and implicit unconstrained decoder threading.
#ifdef SP_HAS_VIDEO_THREAD_CONTROL
      if (!video.open(args.input.string(), cv::CAP_FFMPEG, {cv::CAP_PROP_N_THREADS, 1}))
        throw std::runtime_error("FFmpeg file decoder with one thread is unavailable; use an image or a build with FFmpeg");
#else
      throw std::runtime_error("This OpenCV lacks decoder thread control; use --image-list or --input IMAGE");
#endif
    }

    Json models = Json::array({model.generic_string(), classifier.generic_string()});
    if (model.extension() == ".xml") {
      auto weights = model;
      weights.replace_extension(".bin");
      if (!fs::is_regular_file(weights)) throw std::runtime_error("missing OpenVINO weights");
      models.push_back(weights.generic_string());
    }
    const auto ov_version = ov::get_openvino_version();
    std::ofstream output(args.output / "raw.jsonl", std::ios::binary);
    if (!output) throw std::runtime_error("cannot write raw detector output");
    output << Json({{"kind", "manifest"}, {"schema", "sp-yolo26-file-replay-1"},
                    {"input", args.blank ? "generated-black-image" :
                      (list_input ? args.image_list.generic_string() : args.input.generic_string())},
                    {"input_kind", args.blank ? "synthetic_blank" :
                      (list_input ? "decoded_sequence" : (image_input ? "image" : "video"))},
                    {"config_source", args.config.generic_string()}, {"runtime_config", "runtime.yaml"},
                    {"models", models}, {"opencv", CV_VERSION}, {"openvino_build", ov_version.buildNumber},
                    {"openvino_description", ov_version.description},
                    {"inference_device", "CPU"}, {"inference_threads", 1}, {"opencv_threads", cv::getNumThreads()},
                    {"max_frames", args.max_frames}, {"stride", args.stride},
                    {"input_color", "decoded BGR converted to production RGB; saved PNG uses original BGR"},
                    {"timing", "detector_wall_us includes preprocessing, inference and postprocessing"}}).dump() << '\n';

    int count = 0;
    for (int slot = 0; count < args.max_frames; ++slot) {
      cv::Mat bgr;
      int source_index = slot;
      Json pts_us = nullptr;
      if (list_input) {
        if (static_cast<std::size_t>(slot) >= image_list.size()) break;
        const auto & entry = image_list.at(slot);
        source_index = entry.at("source_index").get<int>();
        bgr = cv::imread(entry.at("path").get<std::string>(), cv::IMREAD_COLOR);
        pts_us = entry.at("video_pts_us");
      } else if (image_input) {
        if (source_index != 0) break;
        bgr = still;
      } else if (!video.read(bgr)) break;
      if (!list_input && source_index % args.stride) continue;
      if (bgr.empty() || bgr.type() != CV_8UC3) throw std::runtime_error("decoded frame must be nonempty CV_8UC3");
      const auto decoded_us = now_us();
      cv::Mat rgb;
      cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
      const auto start_us = now_us();
      const auto armors = detector.detect(rgb, source_index);
      const auto finished_us = now_us();
      if (args.blank && !armors.empty()) throw std::runtime_error("black-image regression returned a detection");
      Json objects = Json::array();
      for (const auto & armor : armors) objects.push_back(detection(armor));
      std::ostringstream name;
      name << "images/frame-" << std::setfill('0') << std::setw(6) << source_index << ".png";
      if (!cv::imwrite((args.output / name.str()).string(), bgr)) throw std::runtime_error("image encoding failed");
      const double pts_ms = image_input ? 0.0 : video.get(cv::CAP_PROP_POS_MSEC);
      if (!list_input && std::isfinite(pts_ms) && pts_ms >= 0)
        pts_us = static_cast<std::uint64_t>(std::llround(pts_ms * 1000.0));
      output << Json({{"kind", "frame"}, {"source_index", source_index}, {"image_path", name.str()},
                      {"width", bgr.cols}, {"height", bgr.rows}, {"video_pts_us", pts_us},
                      {"decoded_host_us", decoded_us}, {"finished_host_us", finished_us},
                      {"detector_wall_us", finished_us - start_us}, {"objects", objects}}).dump() << '\n';
      output.flush();
      if (!output) throw std::runtime_error("raw output write failed");
      ++count;
    }
    if (!count) throw std::runtime_error("input contained no decoded frames");
    output << Json({{"kind", "end"}, {"frame_count", count}, {"complete", true}}).dump() << '\n';
    if (!output) throw std::runtime_error("raw footer write failed");
    std::cout << "PRODUCTION YOLO26 FILE REPLAY: " << count
              << " frame(s), CPU inference_threads=1; no pose, trajectory, hardware or accuracy claim\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "DETECTOR REPLAY FAILED: " << error.what() << '\n';
    return 2;
  }
}
