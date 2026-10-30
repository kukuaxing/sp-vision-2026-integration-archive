// File-only benchmark of the actual production detector. Never links/opens a
// camera, UART or mission bridge. Input images decoded before timed iterations.
#include "tasks/auto_aim/yolos/yolo26.hpp"
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <opencv2/videoio.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <unistd.h>

namespace tools {
std::shared_ptr<spdlog::logger> logger() {
  static auto value=spdlog::stderr_color_mt("vision_benchmark");return value;
}
}
static double percentile(std::vector<double> values,double fraction) {
  std::sort(values.begin(),values.end());return values[static_cast<size_t>(fraction*(values.size()-1))];
}
static nlohmann::json process_resources() {
  nlohmann::json result;
  std::ifstream status("/proc/self/status");
  std::string line;
  while (std::getline(status,line)) {
    for (const auto * key : {"Threads", "VmRSS", "VmHWM"}) {
      if (line.rfind(std::string(key)+":",0) != 0) continue;
      long value = 0;
      std::istringstream(line.substr(line.find(':')+1)) >> value;
      result[key] = value; // Linux RSS/HWM in KiB; Threads is a count.
    }
  }
  return result;
}
int main(int argc,char**argv) {
 try {
  if(argc!=6)throw std::runtime_error("usage: vision_benchmark CONFIG IMAGE_OR_VIDEO CPU_OR_GPU THREADS OUTPUT_JSON");
  const int threads=std::stoi(argv[4]);
  if(threads<1||threads>16)throw std::runtime_error("threads must be 1..16");
  const std::string device=argv[3];
  if(device!="CPU"&&device!="GPU")throw std::runtime_error("explicit CPU or GPU required");
  auto yaml=YAML::LoadFile(argv[1]);
  yaml["device"]=device;yaml["infer_threads"]=threads;
  yaml["detector_debug"]=false;yaml["debug"]=false;
  const auto config=std::filesystem::temp_directory_path()/("rm27_benchmark_"+std::to_string(getpid())+".yaml");
  {std::ofstream out(config);out<<yaml;}
  cv::setNumThreads(1); // Same policy as the production main, before model load.
  const auto resources_before = process_resources();
  const auto init_start = std::chrono::steady_clock::now();
  std::unique_ptr<auto_aim::YOLO26> detector;
  try {detector=std::make_unique<auto_aim::YOLO26>(config.string(),false);}
  catch(...) {std::filesystem::remove(config);throw;}
  const auto initialization_ms = std::chrono::duration<double,std::milli>(
    std::chrono::steady_clock::now()-init_start).count();
  const auto resources_after_model = process_resources();
  std::filesystem::remove(config);
  std::vector<cv::Mat> frames;
  auto image=cv::imread(argv[2]);
  if(!image.empty()) {cv::cvtColor(image,image,cv::COLOR_BGR2RGB);frames.push_back(image);}
  else {
    cv::VideoCapture video(argv[2],cv::CAP_FFMPEG);
    cv::Mat frame;
    for(int k=0;k<24&&video.read(frame);++k) {
      cv::Mat rgb;cv::cvtColor(frame,rgb,cv::COLOR_BGR2RGB);frames.push_back(rgb);
    }
  }
  if(frames.empty())throw std::runtime_error("No local file frames decoded");
  cv::setNumThreads(1);
  for(int k=0;k<20;++k)detector->detect(frames[k%frames.size()],k,true);
  const auto resources_after_warmup = process_resources();
  std::vector<double> total,pre,infer,post;std::vector<int> counts;
  nlohmann::json reference_detections=nlohmann::json::array();
  for(int k=0;k<150;++k) {
    const auto start=std::chrono::steady_clock::now();
    const auto objects=detector->detect(frames[k%frames.size()],k+20,true);
    total.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
    const auto d=detector->diagnostics();
    pre.push_back(d.preprocess_ms);infer.push_back(d.inference_ms);post.push_back(d.postprocess_ms);
    counts.push_back(static_cast<int>(objects.size()));
    if(k<static_cast<int>(frames.size())) {
      nlohmann::json detections=nlohmann::json::array();
      for(const auto & armor:objects) {
        nlohmann::json points=nlohmann::json::array();
        for(const auto & point:armor.points)points.push_back({point.x,point.y});
        detections.push_back({{"name",static_cast<int>(armor.name)}, {"color",static_cast<int>(armor.color)},
          {"confidence",armor.confidence},{"points",points}});
      }
      reference_detections.push_back(detections);
    }
  }
  const nlohmann::json result={{"schema",1},{"device",device},{"threads",threads},
    {"samples",total.size()},{"source",argv[2]},{"decoded_frames",frames.size()},
    {"model_initialization_ms",initialization_ms}, {"resources_before_model",resources_before},
    {"resources_after_model",resources_after_model}, {"resources_after_warmup",resources_after_warmup},
    {"full_frame",true},{"camera_or_uart_opened",false},{"warming_iterations",20},
    {"detector_p50_ms",percentile(total,.5)},{"detector_p95_ms",percentile(total,.95)},
    {"preprocess_p50_ms",percentile(pre,.5)},{"infer_p50_ms",percentile(infer,.5)},
    {"postprocess_p50_ms",percentile(post,.5)}, {"detections",counts},
    {"reference_detections",reference_detections}};
  {std::ofstream out(argv[5]);out<<result.dump(2)<<'\n';if(!out)throw std::runtime_error("Cannot write report");}
  std::cout<<result.dump()<<'\n';return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}
}
