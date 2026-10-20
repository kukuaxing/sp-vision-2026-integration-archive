#pragma once
#include <cmath>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace auto_aim {
inline cv::Rect clipped_detection_roi(cv::Rect configured, cv::Size image)
{
  if (image.width <= 0 || image.height <= 0) return {};
  if (configured.width == -1) configured.width = image.width - configured.x;
  if (configured.height == -1) configured.height = image.height - configured.y;
  const auto clipped = configured & cv::Rect(0, 0, image.width, image.height);
  return clipped.empty() ? cv::Rect(0, 0, image.width, image.height) : clipped;
}
// Search mode never changes the tracker. All observations remain full-image pixels.
class DetectionSearch {
public:
  bool full_frame() const { return full_frame_; }
  void reset() { full_frame_ = true; }
  void observe(const std::string & state, int missed, bool rebuilt,
               const std::vector<cv::Point2f> * corners, cv::Rect roi)
  {
    if (rebuilt || state == "lost" || state == "detecting" ||
        state == "switching" || (state == "temp_lost" && missed >= 3)) {
      reset(); return;
    }
    if (state != "tracking" || corners == nullptr) return;
    const cv::Rect2f inner(roi.x + 12, roi.y + 12,
      std::max(0, roi.width - 24), std::max(0, roi.height - 24));
    full_frame_ = corners->size() != 4;
    for (const auto & p : *corners)
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !inner.contains(p)) full_frame_ = true;
  }
private:
  bool full_frame_ = true;
};
}  // namespace auto_aim
