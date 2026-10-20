#include "adaptive_corner_refiner.hpp"

#include <array>
#include <cassert>
#include <iostream>

#include <opencv2/imgproc.hpp>

int main()
{
  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(8, 8, 8));
  cv::line(image, {104, 74}, {98, 166}, cv::Scalar(255, 255, 255), 8);
  cv::line(image, {214, 78}, {220, 170}, cv::Scalar(255, 255, 255), 8);

  const std::array<cv::Point2f, 4> noisy{{
    {110, 68}, {207, 71}, {226, 177}, {91, 174}}};
  const std::array<cv::Point2f, 4> truth{{
    {104, 74}, {214, 78}, {220, 170}, {98, 166}}};

  auto_aim::AdaptiveCornerRefiner refiner;
  const auto refined = refiner.refine(image, noisy);
  assert(refined.has_value());

  double before = 0.0;
  double after = 0.0;
  for (std::size_t i = 0; i < truth.size(); ++i) {
    before += cv::norm(noisy[i] - truth[i]);
    after += cv::norm(refined->points[i] - truth[i]);
  }
  assert(after < before * 0.45);

  // A neutral panel joins both bars in intensity, but must not change the
  // physical endpoints or force a switch to shorter network keypoints.
  cv::Mat panel(image.size(), image.type(), cv::Scalar::all(8));
  cv::rectangle(panel, {98, 70}, {220, 174}, cv::Scalar::all(180), -1);
  cv::line(panel, {104, 74}, {98, 166}, cv::Scalar(15, 55, 245), 8);
  cv::line(panel, {214, 78}, {220, 170}, cv::Scalar(15, 55, 245), 8);
  using Mode = auto_aim::AdaptiveCornerRefiner::ChannelMode;
  assert(!refiner.refine(panel, noisy).has_value());
  const auto colored = refiner.refine(panel, noisy, Mode::red_blue_contrast);
  assert(colored.has_value());
  for (std::size_t i = 0; i < truth.size(); ++i)
    assert(cv::norm(colored->points[i] - truth[i]) < 7.0);
  cv::Mat swapped;
  cv::cvtColor(panel, swapped, cv::COLOR_RGB2BGR);
  const auto same = refiner.refine(swapped, noisy, Mode::red_blue_contrast);
  assert(same.has_value());
  for (std::size_t i = 0; i < truth.size(); ++i)
    assert(cv::norm(same->points[i] - colored->points[i]) < 1e-4);

  cv::Mat blank(image.size(), image.type(), cv::Scalar::all(0));
  assert(!refiner.refine(blank, noisy).has_value());
  assert(!refiner.refine(blank, noisy, Mode::red_blue_contrast).has_value());
  std::cout << "adaptive_corner_refiner_test: PASS before=" << before
            << " after=" << after << std::endl;
  return 0;
}
