#ifndef AUTO_AIM__YOLO26_HPP
#define AUTO_AIM__YOLO26_HPP

#ifdef ENABLE_OPENVINO

#include <cstdint>
#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/adaptive_corner_refiner.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace YAML { class Node; }

namespace auto_aim
{

class YOLO26 : public YOLOBase
{
public:
  YOLO26(const std::string & config_path, bool debug);

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  std::list<Armor> detect(const cv::Mat & img, int count, bool full_frame) override;
  DetectionDiagnostics diagnostics() const override { return diagnostics_; }

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  struct BarColorStats
  {
    double score = 0.0;  // normalized red-vs-blue chroma, range [-100, 100]
    double agreement = 0.0;
    int count = 0;
  };

  BarColorStats sample_lightbar(
    const cv::Mat & rgb_img, const cv::Point2f & p0, const cv::Point2f & p1,
    int thickness) const;

  void load_detection_settings(const YAML::Node & yaml);
  void configure_corner_refiner(const YAML::Node & yaml);
  std::vector<int> configure_runtime_cpu(const YAML::Node & yaml);
  void configure_input_area(const YAML::Node & yaml);
  int initialize_model(const YAML::Node & yaml, const std::vector<int> & big_cores);
  void log_runtime_settings(int infer_threads) const;

  std::string device_, model_path_;
  std::string save_path_;
  bool debug_, use_roi_, swap_rb_, refine_color_, refine_corners_, save_samples_;

  const int class_num_ = 16;
  const float nms_threshold_ = 0.3;
  double min_confidence_, candidate_confidence_, binary_threshold_;
  double min_keypoint_confidence_;
  double color_confirmed_min_confidence_;
  double memory_confirmed_min_confidence_;
  double color_confirmed_min_keypoint_confidence_;
  double color_min_delta_;
  double color_single_bar_min_delta_;
  int color_min_brightness_;
  int color_min_pixels_per_bar_;
  int color_hold_frames_;
  int letterbox_value_;

  struct ColorMemory
  {
    cv::Point2f center;
    double armor_width;
    Color color;
    double score;
    int frame_count;
  };
  std::vector<ColorMemory> color_memories_;

  ov::Core core_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;  // 同步模式只需1个InferRequest

  // 预分配的输入图像和tensor，避免每帧重新分配
  cv::Mat input_image_;       // 640x640 RGB图像
  ov::Tensor input_tensor_;   // 预绑定的tensor

  cv::Rect roi_, active_roi_;
  DetectionDiagnostics diagnostics_;
  cv::Point2f offset_;
  cv::Mat tmp_img_, color_converted_;

  AdaptiveCornerRefiner corner_refiner_;
  std::uint64_t corner_refine_attempted_ = 0;
  std::uint64_t corner_refine_accepted_ = 0;

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;
  bool has_plausible_geometry(const Armor & armor) const;

  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  std::list<Armor> parse(double scale, int pad_x, int pad_y, cv::Mat & output,
                         const cv::Mat & bgr_img, const cv::Mat & tmp_img, int frame_count);

  void save(const Armor & armor) const;
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
  void sort_keypoints(std::vector<cv::Point2f> & keypoints);
  bool refine_color(Armor & armor, const cv::Mat & rgb_img, int frame_count);
  bool refine_corners(Armor & armor, const cv::Mat & image);
};

}  // namespace auto_aim

#endif  // ENABLE_OPENVINO

#endif  //AUTO_AIM__YOLO26_HPP
