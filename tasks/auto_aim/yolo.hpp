#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <opencv2/opencv.hpp>

#include "armor.hpp"

namespace auto_aim
{
struct DetectionDiagnostics {
  cv::Rect roi;
  int rows = 0, candidates = 0, nms = 0, accepted = 0;
  int rejected_keypoints = 0, rejected_label_shape = 0, rejected_refinement = 0;
};

class YOLOBase
{
public:
  virtual ~YOLOBase() = default;
  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;

  virtual std::list<Armor> detect(const cv::Mat & img, int count, bool full_frame)
  { return detect(img, count); }
  virtual DetectionDiagnostics diagnostics() const { return {}; }

  virtual std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) = 0;
};

class YOLO
{
public:
  YOLO(const std::string & config_path, bool debug = true);

  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1, bool full_frame = false);
  DetectionDiagnostics diagnostics() const { return yolo_->diagnostics(); }

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

private:
  std::unique_ptr<YOLOBase> yolo_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP
