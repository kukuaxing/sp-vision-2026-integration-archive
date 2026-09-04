#include "yolo26.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <thread>
#include <vector>

#include <sched.h>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{

namespace
{
struct BigCoreInfo {
  std::vector<int> ids;
  bool core_type_available = false;
  long max_freq = -1;
};

BigCoreInfo get_big_core_info()
{
  BigCoreInfo info;
  std::vector<int> big_cores;
  const unsigned int cpu_count = std::thread::hardware_concurrency();
  for (unsigned int cpu = 0; cpu < cpu_count; ++cpu) {
    std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/topology/core_type";
    std::ifstream file(path);
    int core_type = -1;
    if (file.good() && (file >> core_type)) {
      info.core_type_available = true;
      if (core_type != 0) {
        big_cores.push_back(static_cast<int>(cpu));
      }
    }
  }
  if (!info.core_type_available) {
    long max_freq = -1;
    std::vector<long> freqs(cpu_count, -1);
    for (unsigned int cpu = 0; cpu < cpu_count; ++cpu) {
      std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/cpufreq/cpuinfo_max_freq";
      std::ifstream file(path);
      long freq = -1;
      if (file.good() && (file >> freq)) {
        freqs[cpu] = freq;
        max_freq = std::max(max_freq, freq);
      }
    }
    info.max_freq = max_freq;
    if (max_freq > 0) {
      for (unsigned int cpu = 0; cpu < cpu_count; ++cpu) {
        if (freqs[cpu] == max_freq) {
          big_cores.push_back(static_cast<int>(cpu));
        }
      }
    }
  }
  info.ids = std::move(big_cores);
  return info;
}

bool bind_current_thread_to_cores(const std::vector<int> & cores)
{
  if (cores.empty()) {
    return false;
  }
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  for (int cpu : cores) {
    CPU_SET(cpu, &cpuset);
  }
  return sched_setaffinity(0, sizeof(cpuset), &cpuset) == 0;
}
}  // namespace
YOLO26::YOLO26(const std::string & config_path, bool debug)
: debug_(debug), detector_(config_path, false)
{
  auto yaml = YAML::LoadFile(config_path);

  model_path_ = yaml["yolo26_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  min_keypoint_confidence_ = yaml["min_keypoint_confidence"]
    ? yaml["min_keypoint_confidence"].as<double>()
    : 0.0;
  letterbox_value_ = yaml["letterbox_value"] ? yaml["letterbox_value"].as<int>() : 114;
  letterbox_value_ = std::clamp(letterbox_value_, 0, 255);
  swap_rb_ = yaml["yolo26_swap_rb"] ? yaml["yolo26_swap_rb"].as<bool>() : false;
  refine_color_ = yaml["yolo26_refine_color"]
    ? yaml["yolo26_refine_color"].as<bool>()
    : false;
  save_samples_ = yaml["yolo26_save_samples"]
    ? yaml["yolo26_save_samples"].as<bool>()
    : false;
  color_min_delta_ = yaml["yolo26_color_min_delta"]
    ? yaml["yolo26_color_min_delta"].as<double>()
    : 12.0;
  color_min_brightness_ = yaml["yolo26_color_min_brightness"]
    ? yaml["yolo26_color_min_brightness"].as<int>()
    : 60;
  color_hold_frames_ = yaml["yolo26_color_hold_frames"]
    ? yaml["yolo26_color_hold_frames"].as<int>()
    : 12;
  color_hold_frames_ = std::max(0, color_hold_frames_);
  if (yaml["detector_debug"]) debug_ = yaml["detector_debug"].as<bool>();

  // 🚀 优化1: 大核绑定，提升推理性能
  std::vector<int> big_cores;
  const auto big_info = get_big_core_info();
  big_cores = big_info.ids;
  const int cpu_cores = static_cast<int>(std::thread::hardware_concurrency());

  tools::logger()->info(
    "[YOLO26] CPU cores: {}, big cores found: {}, core_type: {}, max_freq: {}",
    cpu_cores, big_cores.size(),
    big_info.core_type_available ? "available" : "unavailable",
    big_info.max_freq);

  if (!big_cores.empty()) {
    std::string core_list;
    for (size_t i = 0; i < big_cores.size(); ++i) {
      core_list += std::to_string(big_cores[i]);
      if (i + 1 < big_cores.size()) core_list += ",";
    }
    tools::logger()->info("[YOLO26] Big core ids: [{}]", core_list);

    if (bind_current_thread_to_cores(big_cores)) {
      tools::logger()->info("[YOLO26] Successfully bound to {} big cores", big_cores.size());
    } else {
      tools::logger()->warn("[YOLO26] Failed to bind to big cores, using default affinity");
    }
  }

  // 🚀 优化2: 禁用OpenCV多线程，避免与OpenVINO争抢CPU资源
  cv::setNumThreads(1);
  tools::logger()->info("[YOLO26] Disabled OpenCV threading to avoid CPU contention");

  // ROI配置
  int x = yaml["roi"]["x"].as<int>();
  int y = yaml["roi"]["y"].as<int>();
  int width = yaml["roi"]["width"].as<int>();
  int height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);

  // 读取并配置模型
  auto model = core_.read_model(model_path_);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  // 🚀 优化3: 配置输入为RGB U8格式，利用OpenVINO的硬件加速预处理
  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 640, 640, 3})
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::RGB);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .scale(255.0);

  model = ppp.build();

  // 🚀 优化4: 同步模式最优配置
  // - LATENCY模式：优化单次推理延迟
  // - streams=1：同步推理只需要1个流
  // - num_requests=1：同步模式只需要1个请求
  // - 线程数：根据大核数量自适应（2-4线程最优）
  const int default_threads = big_cores.empty() ? 4 : std::min(4, static_cast<int>(big_cores.size()));
  const int infer_threads = yaml["infer_threads"]
    ? std::max(1, yaml["infer_threads"].as<int>())
    : default_threads;

  tools::logger()->info("[YOLO26] Compiling model with {} threads (LATENCY mode)", infer_threads);

  compiled_model_ = core_.compile_model(
    model, device_,
    ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
    ov::streams::num(1),
    ov::hint::num_requests(1),
    ov::inference_num_threads(infer_threads)
  );

  // 🚀 优化5: 预分配输入图像和tensor，避免每帧重新分配
  infer_request_ = compiled_model_.create_infer_request();
  input_image_ = cv::Mat(
    640, 640, CV_8UC3,
    cv::Scalar(letterbox_value_, letterbox_value_, letterbox_value_));
  input_tensor_ = ov::Tensor(
    ov::element::u8,
    {1, 640, 640, 3},
    input_image_.data
  );

  // 预先绑定tensor到InferRequest
  infer_request_.set_input_tensor(input_tensor_);

  tools::logger()->info("[YOLO26] Initialization complete - SYNC mode optimized");
  tools::logger()->info(
    "[YOLO26] Config: threads={}, streams=1, requests=1, letterbox={}, "
    "min_confidence={:.2f}, min_keypoint_confidence={:.2f}, swap_rb={}, "
    "refine_color={}, color_delta={:.1f}, color_hold_frames={}, save_samples={}, debug={}",
    infer_threads, letterbox_value_, min_confidence_, min_keypoint_confidence_, swap_rb_,
    refine_color_, color_min_delta_, color_hold_frames_, save_samples_, debug_);
}

std::list<Armor> YOLO26::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("[YOLO26] Empty image, camera drop!");
    return std::list<Armor>();
  }

  auto t_total_start = std::chrono::high_resolution_clock::now();

  // ========== 阶段1: 预处理 ==========
  auto t_preprocess_start = std::chrono::high_resolution_clock::now();

  cv::Mat bgr_img;
  tmp_img_ = raw_img;
  cv::Mat tmp_img = raw_img;
  cv::Mat model_img = raw_img;
  if (swap_rb_) {
    cv::cvtColor(raw_img, color_converted_, cv::COLOR_RGB2BGR);
    model_img = color_converted_;
  }

  if (use_roi_) {
    if (roi_.width == -1) roi_.width = raw_img.cols;
    if (roi_.height == -1) roi_.height = raw_img.rows;
    bgr_img = model_img(roi_);
  } else {
    bgr_img = model_img;
  }

  int orig_w = bgr_img.cols;
  int orig_h = bgr_img.rows;

  // 计算letterbox缩放参数
  float scale = std::min(640.0f / orig_w, 640.0f / orig_h);
  int new_w = static_cast<int>(orig_w * scale);
  int new_h = static_cast<int>(orig_h * scale);
  int pad_x = (640 - new_w) / 2;
  int pad_y = (640 - new_h) / 2;

  // 🚀 优化: 直接在预分配的input_image_上操作，避免临时内存分配
  // Match the model export metadata (pad_value=114), and clear every frame so
  // a future ROI/geometry change cannot leave stale pixels in the padding.
  input_image_.setTo(cv::Scalar(letterbox_value_, letterbox_value_, letterbox_value_));
  cv::resize(
    bgr_img,
    input_image_(cv::Rect(pad_x, pad_y, new_w, new_h)),
    cv::Size(new_w, new_h),
    0, 0,
    cv::INTER_LINEAR
  );

  auto t_preprocess_end = std::chrono::high_resolution_clock::now();
  auto duration_preprocess = std::chrono::duration_cast<std::chrono::microseconds>(
    t_preprocess_end - t_preprocess_start).count();

  // ========== 阶段2: 同步推理 ==========
  auto t_infer_start = std::chrono::high_resolution_clock::now();

  // 🚀 优化: tensor已预绑定，无需每帧set_input_tensor
  infer_request_.infer();

  auto t_infer_end = std::chrono::high_resolution_clock::now();
  auto duration_infer = std::chrono::duration_cast<std::chrono::microseconds>(
    t_infer_end - t_infer_start).count();

  // ========== 阶段3: 后处理 ==========
  auto t_postprocess_start = std::chrono::high_resolution_clock::now();

  auto output_tensor = infer_request_.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

  auto result = parse(scale, pad_x, pad_y, output, bgr_img, tmp_img, frame_count);

  auto t_postprocess_end = std::chrono::high_resolution_clock::now();
  auto duration_postprocess = std::chrono::duration_cast<std::chrono::microseconds>(
    t_postprocess_end - t_postprocess_start).count();

  // ========== 性能统计 ==========
  auto t_total_end = std::chrono::high_resolution_clock::now();
  auto duration_total = std::chrono::duration_cast<std::chrono::microseconds>(
    t_total_end - t_total_start).count();

  static int debug_perf_counter = 0;
  if (debug_ && ++debug_perf_counter % 50 == 0) {
    tools::logger()->info("[YOLO26] ==================== Performance ====================");
    tools::logger()->info("[YOLO26] Preprocess:  {:.3f} ms ({:.1f}%)",
                          duration_preprocess / 1000.0,
                          100.0 * duration_preprocess / duration_total);
    tools::logger()->info("[YOLO26] Inference:   {:.3f} ms ({:.1f}%)",
                          duration_infer / 1000.0,
                          100.0 * duration_infer / duration_total);
    tools::logger()->info("[YOLO26] Postprocess: {:.3f} ms ({:.1f}%)",
                          duration_postprocess / 1000.0,
                          100.0 * duration_postprocess / duration_total);
    tools::logger()->info("[YOLO26] Total:       {:.3f} ms ({:.1f} FPS)",
                          duration_total / 1000.0,
                          1000000.0 / duration_total);
    tools::logger()->info("[YOLO26] ======================================================");
  }

  return result;
}
// 🚀 优化后的parse函数：使用指针访问、预分配空间、减少类型转换
std::list<Armor> YOLO26::parse(
  double scale, int pad_x, int pad_y, cv::Mat & output, const cv::Mat & bgr_img,
  const cv::Mat & tmp_img, int frame_count)
{
  const int num_detections = output.rows;
  const int num_cols = output.cols;
  const int img_width = bgr_img.cols;
  const int img_height = bgr_img.rows;

  // 🚀 优化1: 预先计算缩放因子的倒数（乘法比除法快）
  const float inv_scale = 1.0f / static_cast<float>(scale);
  const float pad_x_f = static_cast<float>(pad_x);
  const float pad_y_f = static_cast<float>(pad_y);

  // 🚀 优化2: 预分配空间，避免动态扩容
  const int estimated_valid = std::max(32, num_detections / 4);
  std::vector<int> ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;
  std::vector<float> keypoint_confidences;

  ids.reserve(estimated_valid);
  confidences.reserve(estimated_valid);
  boxes.reserve(estimated_valid);
  armors_key_points.reserve(estimated_valid);
  keypoint_confidences.reserve(estimated_valid);

  // 🚀 优化3: 获取原始数据指针，避免at<float>()的边界检查开销
  const float* data_ptr = output.ptr<float>(0);
  const bool is_standard_format = (num_cols == 14 || num_cols == 18);
  const int kpt_start = 6;
  // 模型输出每点 3 值 (x,y,visibility)：18 列 = 4(xyxy)+1(conf)+1(cls)+12(kpts)
  // 旧 14 列模型为 2 值/点；步长按列数定，避免把 visibility 当坐标
  const int kpt_step = (num_cols == 18) ? 3 : 2;
  float max_output_conf = 0.0f;
  int max_output_class = -1;

  for (int r = 0; r < num_detections; ++r) {
    // 🚀 优化4: 使用指针偏移访问，比at<float>(r, c)快约2-3倍
    const float* row_ptr = data_ptr + r * num_cols;

    const float x1_raw = row_ptr[0];
    const float y1_raw = row_ptr[1];
    const float x2_raw = row_ptr[2];
    const float y2_raw = row_ptr[3];

    float conf;
    int cls;

    if (is_standard_format) {
      // cols=14或18: [xyxy(4), conf(1), cls(1), kpts...]
      conf = row_ptr[4];
      cls = static_cast<int>(row_ptr[5]);
    } else {
      // 兜底：按[xyxy + obj + class scores + kpts]解析
      const float obj_conf = row_ptr[4];
      const int cls_start = 5;
      float cls_score = 0.0f;
      cls = 0;
      for (int c = 0; c < class_num_; ++c) {
        const float score = row_ptr[cls_start + c];
        if (score > cls_score) {
          cls_score = score;
          cls = c;
        }
      }
      conf = obj_conf * cls_score;
    }
    if (std::isfinite(conf) && conf > max_output_conf) {
      max_output_conf = conf;
      max_output_class = cls;
    }

    // 🚀 优化5: 提前过滤低置信度检测
    if (conf < score_threshold_) {
      continue;
    }

    // 🚀 优化6: bbox坐标转换，使用乘法替代除法
    const int x1 = std::clamp(static_cast<int>((x1_raw - pad_x_f) * inv_scale), 0, img_width);
    const int y1 = std::clamp(static_cast<int>((y1_raw - pad_y_f) * inv_scale), 0, img_height);
    const int x2 = std::clamp(static_cast<int>((x2_raw - pad_x_f) * inv_scale), 0, img_width);
    const int y2 = std::clamp(static_cast<int>((y2_raw - pad_y_f) * inv_scale), 0, img_height);

    const int width  = x2 - x1;
    const int height = y2 - y1;

    // 跳过无效的box
    if (width <= 0 || height <= 0) continue;

    // 🚀 优化7: keypoints，预分配4个点，使用指针访问
    std::vector<cv::Point2f> armor_key_points;
    armor_key_points.reserve(4);
    const float* kpt_ptr = row_ptr + kpt_start;
    float min_kpt_conf = 1.0f;
    bool keypoints_valid = true;

    for (int i = 0; i < 4; ++i) {
      const float kx = (kpt_ptr[i * kpt_step] - pad_x_f) * inv_scale;
      const float ky = (kpt_ptr[i * kpt_step + 1] - pad_y_f) * inv_scale;
      if (!std::isfinite(kx) || !std::isfinite(ky) ||
          kx < 0.0f || kx >= img_width || ky < 0.0f || ky >= img_height) {
        keypoints_valid = false;
      }
      if (kpt_step == 3) {
        const float visibility = kpt_ptr[i * kpt_step + 2];
        if (!std::isfinite(visibility)) keypoints_valid = false;
        min_kpt_conf = std::min(min_kpt_conf, visibility);
      }
      armor_key_points.emplace_back(kx, ky);
    }
    if (!keypoints_valid || min_kpt_conf < min_keypoint_confidence_) continue;

    ids.emplace_back(cls);
    confidences.emplace_back(conf);
    boxes.emplace_back(x1, y1, width, height);
    armors_key_points.emplace_back(std::move(armor_key_points));
    keypoint_confidences.emplace_back(min_kpt_conf);
  }

  // NMS (非极大值抑制)
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  // 生成Armor对象
  std::list<Armor> armors;
  for (int i : indices) {
    sort_keypoints(armors_key_points[i]);

    if (use_roi_) {
      armors.emplace_back(
        ids[i],
        confidences[i],
        boxes[i],
        armors_key_points[i],
        offset_,
        YOLOVersion::YOLO26
      );
    } else {
      armors.emplace_back(
        ids[i],
        confidences[i],
        boxes[i],
        armors_key_points[i],
        YOLOVersion::YOLO26
      );
    }
    armors.back().keypoint_confidence = keypoint_confidences[i];
    if (refine_color_) refine_color(armors.back(), tmp_img, frame_count);
  }

  // 过滤
  int filtered_count = 0;
  for (auto it = armors.begin(); it != armors.end();) {
    bool name_ok = check_name(*it);
    bool type_ok = check_type(*it);

    if (!name_ok || !type_ok) {
      filtered_count++;
      it = armors.erase(it);
      continue;
    }

    it->center_norm = get_center_norm(tmp_img, it->center);
    ++it;
  }

  if (debug_) {
    float max_kpt_conf = 0.0f;
    if (!keypoint_confidences.empty()) {
      max_kpt_conf = *std::max_element(
        keypoint_confidences.begin(), keypoint_confidences.end());
    }
    const int refined_color = armors.empty() ? -1 : static_cast<int>(armors.front().color);
    const double color_score = armors.empty() ? 0.0 : armors.front().color_score;
    tools::logger()->info(
      "[YOLO26][DET] above_0p2={} after_nms={} accepted={} max_conf={:.4f} "
      "max_class={} max_min_kpt={:.4f} refined_color={} color_score={:.2f} color_source={}",
      ids.size(), indices.size(), armors.size(), max_output_conf, max_output_class,
      max_kpt_conf, refined_color, color_score, armors.empty() ? -1 : armors.front().color_source);
  }

  // 绘制检测结果
  if (debug_ && !tmp_img.empty()) {
    draw_detections(tmp_img, armors, frame_count);
  }

  return armors;
}

bool YOLO26::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  // Optional offline dataset collection. Disabled by default so a normal
  // real-time run cannot silently fill the robot's system disk.
  if (save_samples_ && armor.confidence > 0.5 && armor.confidence < 0.7) save(armor);

  return name_ok && confidence_ok;
}

bool YOLO26::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);

  // 不在check_type中保存，统一在check_name中处理
  // if (!name_ok) save(armor);

  return name_ok;
}

cv::Point2f YOLO26::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void YOLO26::sort_keypoints(std::vector<cv::Point2f> & keypoints)
{
  if (keypoints.size() != 4) {
    tools::logger()->warn("[YOLO26] Invalid keypoints size: {}", keypoints.size());
    return;
  }

  // 按y坐标排序
  std::sort(keypoints.begin(), keypoints.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.y < b.y;
  });

  // 分为上下两组
  std::vector<cv::Point2f> top_points = {keypoints[0], keypoints[1]};
  std::vector<cv::Point2f> bottom_points = {keypoints[2], keypoints[3]};

  // 各组内按x坐标排序
  std::sort(top_points.begin(), top_points.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;
  });

  std::sort(bottom_points.begin(), bottom_points.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;
  });

  // 重新排列：左上、右上、右下、左下
  keypoints[0] = top_points[0];     // top-left
  keypoints[1] = top_points[1];     // top-right
  keypoints[2] = bottom_points[1];  // bottom-right
  keypoints[3] = bottom_points[0];  // bottom-left
}

void YOLO26::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  cv::Mat detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});

  for (const auto & armor : armors) {
    auto info = fmt::format(
      "conf={:.2f} kpt={:.2f} color={:.1f}/{} {} {} {}", armor.confidence,
      armor.keypoint_confidence, armor.color_score, armor.color_source, COLORS[armor.color],
      ARMOR_NAMES[armor.name], ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }

  if (use_roi_) {
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, roi_, green, 2);
  }

  cv::resize(detection, detection, {}, 0.5, 0.5);

  // 相机输出是RGB格式，imshow需要BGR格式，所以需要转换
  cv::Mat bgr_detection;
  cv::cvtColor(detection, bgr_detection, cv::COLOR_RGB2BGR);
  static int debug_image_counter = 0;
  if (++debug_image_counter % 50 == 0) {
    cv::imwrite("imgs/yolo26_debug_latest.jpg", bgr_detection);
  }
  cv::imshow("YOLO26 Detection", bgr_detection);
}

void YOLO26::save(const Armor & armor) const
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);

  // 相机输出是RGB格式，需要转换为BGR才能正确保存
  cv::Mat bgr_img;
  cv::cvtColor(tmp_img_, bgr_img, cv::COLOR_RGB2BGR);
  cv::imwrite(img_path, bgr_img);
}

bool YOLO26::refine_color(Armor & armor, const cv::Mat & rgb_img, int frame_count)
{
  if (rgb_img.empty() || rgb_img.type() != CV_8UC3 || armor.points.size() != 4) return false;

  const double left_length = cv::norm(armor.points[3] - armor.points[0]);
  const double right_length = cv::norm(armor.points[2] - armor.points[1]);
  const double top_width = cv::norm(armor.points[1] - armor.points[0]);
  const double bottom_width = cv::norm(armor.points[2] - armor.points[3]);
  const double armor_width = std::max(top_width, bottom_width);
  // The network keypoints lie near the lightbar center lines.  Scale the
  // sampling width from armor width so slightly drifting keypoints still cover
  // the complete illuminated bar, while avoiding the number plate center.
  const int thickness = std::max(
    5, static_cast<int>(std::lround(std::min(
      0.12 * armor_width, 0.45 * std::max(left_length, right_length)))));

  cv::Mat mask(rgb_img.rows, rgb_img.cols, CV_8UC1, cv::Scalar(0));
  const auto pixel = [](const cv::Point2f & p) {
    return cv::Point(static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y)));
  };
  cv::line(mask, pixel(armor.points[0]), pixel(armor.points[3]), cv::Scalar(255), thickness, cv::LINE_AA);
  cv::line(mask, pixel(armor.points[1]), pixel(armor.points[2]), cv::Scalar(255), thickness, cv::LINE_AA);

  int64_t signed_chroma_sum = 0;
  int chroma_count = 0;
  const cv::Rect roi = cv::boundingRect(armor.points) & cv::Rect(0, 0, rgb_img.cols, rgb_img.rows);
  for (int y = roi.y; y < roi.y + roi.height; ++y) {
    const auto * rgb = rgb_img.ptr<cv::Vec3b>(y);
    const auto * m = mask.ptr<uint8_t>(y);
    for (int x = roi.x; x < roi.x + roi.width; ++x) {
      if (m[x] == 0) continue;
      const int red = rgb[x][0];
      const int blue = rgb[x][2];
      if (std::max(red, blue) < color_min_brightness_) continue;
      const int chroma = red - blue;
      if (std::abs(chroma) < 4) continue;
      signed_chroma_sum += chroma;
      ++chroma_count;
    }
  }
  armor.color_score = chroma_count == 0
    ? 0.0
    : static_cast<double>(signed_chroma_sum) / chroma_count;

  color_memories_.erase(
    std::remove_if(
      color_memories_.begin(), color_memories_.end(),
      [this, frame_count](const ColorMemory & memory) {
        return frame_count - memory.frame_count > color_hold_frames_;
      }),
    color_memories_.end());

  const double match_radius = std::max(20.0, 0.35 * armor_width);
  auto nearest = color_memories_.end();
  double nearest_distance = std::numeric_limits<double>::max();
  for (auto it = color_memories_.begin(); it != color_memories_.end(); ++it) {
    const double distance = cv::norm(armor.center - it->center);
    if (distance < nearest_distance && distance <= match_radius) {
      nearest = it;
      nearest_distance = distance;
    }
  }

  const bool current_image_reliable =
    chroma_count >= 6 && std::abs(armor.color_score) >= color_min_delta_;
  if (current_image_reliable) {
    armor.color = armor.color_score > 0.0 ? Color::red : Color::blue;
    armor.color_source = 1;
    if (nearest == color_memories_.end()) {
      color_memories_.push_back({armor.center, armor.color, frame_count});
    } else {
      nearest->center = armor.center;
      nearest->color = armor.color;
      nearest->frame_count = frame_count;
    }
    return true;
  }

  // Armor LEDs are PWM driven.  At 150 fps some frames capture their dark
  // phase, so retain only a recent, spatially matched image classification.
  if (nearest != color_memories_.end()) {
    armor.color = nearest->color;
    armor.color_source = 2;
    nearest->center = armor.center;
    return true;
  }
  return false;
}

std::list<Armor> YOLO26::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // postprocess 假设没有 letterbox padding（外部调用场景）
  // 使用bgr_img作为tmp_img（因为外部调用没有额外的原始图像）
  return parse(scale, 0, 0, output, bgr_img, bgr_img, frame_count);
}

}  // namespace auto_aim
