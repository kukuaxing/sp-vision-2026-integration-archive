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
  // Metrology must remain stable when only global brightness changes. This
  // uses raster light bars rather than target pose/impact truth to select edges.
  auto_aim::AdaptiveCornerRefiner::Config invariant_config;
  invariant_config.brightness_invariant_subpixel=true;
  auto_aim::AdaptiveCornerRefiner invariant(invariant_config);
  const auto reference=invariant.refine(panel,noisy,Mode::red_blue_contrast);
  assert(reference);
  for(double brightness:{.35,.65,1.0}) {
    cv::Mat scaled;panel.convertTo(scaled,panel.type(),brightness);
    const auto result=invariant.refine(scaled,noisy,Mode::red_blue_contrast);
    assert(result);
    for(std::size_t i=0;i<4;++i)assert(cv::norm(result->points[i]-reference->points[i])<1.25);
    cv::Mat reverse;cv::cvtColor(scaled,reverse,cv::COLOR_RGB2BGR);
    const auto reversed=invariant.refine(reverse,noisy,Mode::red_blue_contrast);
    assert(reversed);
    for(std::size_t i=0;i<4;++i)assert(cv::norm(reversed->points[i]-result->points[i])<1e-4);
  }
  assert(!invariant.refine(blank,noisy,Mode::red_blue_contrast));
  assert(!invariant.refine(cv::Mat(image.size(),image.type(),cv::Scalar::all(160)),noisy,Mode::red_blue_contrast));
  // Field regression: one blooming frontal lamp had minAreaRect aspect 1.91
  // despite correct endpoints; its reliable companion must corroborate it.
  const std::array<cv::Point2f,4> frontal{{{100,75},{210,75},{210,165},{100,165}}};
  cv::Mat bloom(image.size(),image.type(),cv::Scalar::all(8));
  cv::rectangle(bloom,{75,75},{125,165},cv::Scalar(15,55,245),-1);
  cv::rectangle(bloom,{205,75},{215,165},cv::Scalar(15,55,245),-1);
  const auto recovered=refiner.refine(bloom,frontal,Mode::red_blue_contrast);
  assert(recovered && recovered->single_broad_light);
  for(std::size_t i=0;i<4;++i) assert(cv::norm(recovered->points[i]-frontal[i])<3.0);
  assert(!refiner.refine(bloom,frontal)); // intensity blobs do not use this exception
  cv::Mat reverse_bloom;cv::cvtColor(bloom,reverse_bloom,cv::COLOR_RGB2BGR);
  const auto recovered_reverse=refiner.refine(reverse_bloom,frontal,Mode::red_blue_contrast);
  assert(recovered_reverse && recovered_reverse->single_broad_light);
  for(std::size_t i=0;i<4;++i)
    assert(cv::norm(recovered_reverse->points[i]-recovered->points[i])<1e-4);
  cv::Mat double_bloom=bloom.clone();
  cv::rectangle(double_bloom,{185,75},{235,165},cv::Scalar(15,55,245),-1);
  assert(!refiner.refine(double_bloom,frontal,Mode::red_blue_contrast));
  cv::Mat missing=bloom.clone();
  cv::rectangle(missing,{180,40},{250,190},cv::Scalar::all(8),-1);
  assert(!refiner.refine(missing,frontal,Mode::red_blue_contrast));
  auto displaced=frontal;displaced[0].y+=22;displaced[3].y+=22;
  assert(!refiner.refine(bloom,displaced,Mode::red_blue_contrast));
  assert(!refined->single_broad_light && !colored->single_broad_light);
  std::cout << "adaptive_corner_refiner_test: PASS before=" << before
            << " after=" << after << std::endl;
  return 0;
}
