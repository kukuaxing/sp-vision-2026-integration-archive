#include "adaptive_corner_refiner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{
namespace
{
double line_angle_deg(const cv::Point2f & top, const cv::Point2f & bottom)
{
  const cv::Point2f d = bottom - top;
  return std::atan2(d.x, d.y) * 180.0 / CV_PI;
}

double angle_distance_deg(double a, double b)
{
  double d = std::remainder(a - b, 180.0);
  return std::abs(d);
}

cv::Rect clipped_roi(
  const cv::Point2f & center, double width, double height, const cv::Size & size)
{
  const int x0 = std::max(0, static_cast<int>(std::floor(center.x - width * 0.5)));
  const int y0 = std::max(0, static_cast<int>(std::floor(center.y - height * 0.5)));
  const int x1 = std::min(size.width, static_cast<int>(std::ceil(center.x + width * 0.5)));
  const int y1 = std::min(size.height, static_cast<int>(std::ceil(center.y + height * 0.5)));
  return cv::Rect(x0, y0, std::max(0, x1 - x0), std::max(0, y1 - y0));
}
}  // namespace

std::optional<AdaptiveCornerRefiner::Result> AdaptiveCornerRefiner::refine(
  const cv::Mat & image, const std::array<cv::Point2f, 4> & p) const
{
  if (image.empty()) return std::nullopt;
  for (const auto & point : p) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return std::nullopt;
  }

  cv::Mat gray;
  if (image.channels() == 1) {
    gray = image;
  } else {
    // Max-channel intensity is insensitive to whether the camera buffer is RGB
    // or BGR and preserves saturated red/blue light bars in short exposures.
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    cv::max(channels[0], channels[1], gray);
    if (channels.size() > 2) cv::max(gray, channels[2], gray);
  }

  const auto left = refine_light(gray, p[0], p[3]);
  const auto right = refine_light(gray, p[1], p[2]);
  if (!left || !right) return std::nullopt;

  const double length_ratio =
    std::min(left->length, right->length) / std::max(left->length, right->length);
  const double pair_angle_error = angle_distance_deg(left->angle_deg, right->angle_deg);
  if (length_ratio < config_.min_pair_length_ratio ||
      pair_angle_error > config_.max_pair_angle_error_deg) {
    return std::nullopt;
  }

  const double expected_width = cv::norm((p[1] + p[2] - p[0] - p[3]) * 0.5F);
  const double refined_width = cv::norm(right->center - left->center);
  if (expected_width < 3.0 || refined_width < 0.55 * expected_width ||
      refined_width > 1.45 * expected_width) {
    return std::nullopt;
  }

  Result result;
  result.points = {left->top, right->top, right->bottom, left->bottom};
  result.score = 0.5 * (left->score + right->score) +
    0.25 * pair_angle_error / std::max(1.0, config_.max_pair_angle_error_deg) +
    0.25 * (1.0 - length_ratio);
  return result;
}

std::optional<AdaptiveCornerRefiner::LightCandidate> AdaptiveCornerRefiner::refine_light(
  const cv::Mat & gray, const cv::Point2f & expected_top,
  const cv::Point2f & expected_bottom) const
{
  const cv::Point2f expected_center = (expected_top + expected_bottom) * 0.5F;
  const double expected_length = cv::norm(expected_bottom - expected_top);
  if (expected_length < 4.0) return std::nullopt;

  const cv::Rect roi = clipped_roi(
    expected_center,
    std::max(12.0, expected_length * config_.roi_width_scale),
    std::max(12.0, expected_length * config_.roi_length_scale), gray.size());
  if (roi.width < 4 || roi.height < 4) return std::nullopt;

  cv::Mat blurred;
  cv::GaussianBlur(gray(roi), blurred, cv::Size(3, 3), 0.0);
  cv::Mat otsu_binary;
  const double otsu = cv::threshold(
    blurred, otsu_binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  const double threshold = std::clamp(
    0.65 * otsu + 0.35 * config_.binary_threshold, 25.0, 230.0);
  cv::Mat binary;
  cv::threshold(blurred, binary, threshold, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  const double expected_angle = line_angle_deg(expected_top, expected_bottom);
  LightCandidate best;
  best.score = std::numeric_limits<double>::infinity();

  for (const auto & contour : contours) {
    if (contour.size() < 4 || cv::contourArea(contour) < 3.0) continue;
    const cv::RotatedRect rr = cv::minAreaRect(contour);
    const double major = std::max(rr.size.width, rr.size.height);
    const double minor = std::max(0.5F, std::min(rr.size.width, rr.size.height));
    if (major / minor < config_.min_light_aspect) continue;
    const double rr_area = std::max(1.0, major * minor);
    if (cv::contourArea(contour) / rr_area < config_.min_contour_fill) continue;

    std::vector<cv::Point2f> points;
    points.reserve(contour.size());
    for (const auto & point : contour) {
      points.emplace_back(
        static_cast<float>(point.x + roi.x), static_cast<float>(point.y + roi.y));
    }
    cv::Vec4f line;
    cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);
    cv::Point2f direction(line[0], line[1]);
    const double norm = cv::norm(direction);
    if (norm < 1e-6) continue;
    direction *= static_cast<float>(1.0 / norm);
    const cv::Point2f origin(line[2], line[3]);

    double min_projection = std::numeric_limits<double>::infinity();
    double max_projection = -std::numeric_limits<double>::infinity();
    for (const auto & point : points) {
      const double projection = (point - origin).dot(direction);
      min_projection = std::min(min_projection, projection);
      max_projection = std::max(max_projection, projection);
    }
    cv::Point2f a = origin + direction * static_cast<float>(min_projection);
    cv::Point2f b = origin + direction * static_cast<float>(max_projection);
    if (a.y > b.y) std::swap(a, b);
    const double length = cv::norm(b - a);
    const cv::Point2f center = (a + b) * 0.5F;
    const double center_error = cv::norm(center - expected_center) / expected_length;
    const double length_error = std::abs(length - expected_length) / expected_length;
    const double angle = line_angle_deg(a, b);
    const double angle_error = angle_distance_deg(angle, expected_angle);
    const double endpoint_shift =
      std::max(cv::norm(a - expected_top), cv::norm(b - expected_bottom)) /
      expected_length;

    if (center_error > config_.max_center_error_ratio ||
        angle_error > config_.max_angle_error_deg || length_error > 0.65 ||
        endpoint_shift > config_.max_endpoint_shift_ratio) {
      continue;
    }
    const double score = 0.45 * center_error + 0.35 * length_error +
      0.15 * angle_error / std::max(1.0, config_.max_angle_error_deg) +
      0.05 * endpoint_shift;
    if (score < best.score) {
      best = {a, b, center, length, angle, score};
    }
  }

  if (!std::isfinite(best.score)) return std::nullopt;
  return best;
}

}  // namespace auto_aim
