#ifndef AUTO_AIM__ADAPTIVE_CORNER_REFINER_HPP
#define AUTO_AIM__ADAPTIVE_CORNER_REFINER_HPP

#include <array>
#include <optional>

#include <opencv2/core.hpp>

namespace auto_aim
{

// Refines neural-network keypoints on the two physical light bars.  The
// network result remains the fallback whenever the image evidence is weak.
class AdaptiveCornerRefiner
{
public:
  struct Config
  {
    int binary_threshold = 100;
    double roi_length_scale = 1.8;
    double roi_width_scale = 0.9;
    double max_center_error_ratio = 0.65;
    double max_angle_error_deg = 25.0;
    double max_pair_angle_error_deg = 8.0;
    double min_pair_length_ratio = 0.65;
    double min_contour_fill = 0.30;
    double min_light_aspect = 2.0;
    double max_endpoint_shift_ratio = 0.80;
  };

  struct Result
  {
    // Current project order: left-top, right-top, right-bottom, left-bottom.
    std::array<cv::Point2f, 4> points;
    double score = 0.0;
  };

  AdaptiveCornerRefiner() = default;
  explicit AdaptiveCornerRefiner(const Config & config) : config_(config) {}

  void set_config(const Config & config) { config_ = config; }
  const Config & config() const { return config_; }

  std::optional<Result> refine(
    const cv::Mat & image, const std::array<cv::Point2f, 4> & network_points) const;

private:
  struct LightCandidate
  {
    cv::Point2f top;
    cv::Point2f bottom;
    cv::Point2f center;
    double length = 0.0;
    double angle_deg = 0.0;
    double score = 0.0;
  };

  std::optional<LightCandidate> refine_light(
    const cv::Mat & gray, const cv::Point2f & expected_top,
    const cv::Point2f & expected_bottom) const;

  Config config_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ADAPTIVE_CORNER_REFINER_HPP
