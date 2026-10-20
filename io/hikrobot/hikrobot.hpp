#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "MvCameraControl.h"
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class HikRobot : public CameraBase
{
public:
  explicit HikRobot(const std::string & config_path);
  ~HikRobot() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
  std::uint64_t get_last_frame_id() const { return last_frame_id_; }

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
    std::uint64_t frame_id;
  };

  double exposure_us_;
  double timestamp_transport_delay_us_ = 0.0;
  std::uint64_t last_frame_id_ = 0;
  double gain_;
  double frame_rate_;
  int64_t width_;
  int64_t height_;
  unsigned int white_balance_auto_;
  bool trigger_enable_;
  unsigned int trigger_source_;
  unsigned int trigger_activation_;
  std::string pixel_format_;

  std::thread daemon_thread_;
  std::atomic<bool> daemon_quit_;

  void * handle_;
  bool device_open_;
  bool grabbing_;
  std::thread capture_thread_;
  std::atomic<bool> capturing_;
  std::atomic<bool> capture_quit_;
  // Preview consumers can stall in detection/GUI: retain the newest frame.
  tools::ThreadSafeQueue<CameraData, true> queue_;

  int vid_, pid_;

  void capture_start();
  bool open_camera();
  void configure_camera();
  void capture_loop();
  void capture_stop();

  bool set_float_value(const std::string & name, double value);
  bool set_enum_value(const std::string & name, unsigned int value);
  bool set_enum_value(const std::string & name, const std::string & value);
  bool set_int_value(const std::string & name, int64_t value);
  bool set_bool_value(const std::string & name, bool value);
  void log_camera_state() const;

  void set_vid_pid(const std::string & vid_pid);
  void reset_usb() const;
};

}  // namespace io

#endif  // IO__HIKROBOT_HPP
