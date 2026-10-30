#include "io/camera.hpp"

#include <opencv2/opencv.hpp>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/standard.yaml | yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  // Preview only: construct no CBoard, XucSender, serial port or controller.
  cv::setNumThreads(1);
  tools::Exiter exiter;
  const auto config_path = cli.get<std::string>("config-path");
  const bool display = cli.has("display");
  const std::string title = "rm27_1018 - Camera Preview (NO CONTROL)";
  if (display) {
    cv::namedWindow(title, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow(title, 1100, 825);
    cv::moveWindow(title, 150, 80);
  }
  io::Camera camera(config_path);
  cv::Mat img, preview;
  std::chrono::steady_clock::time_point timestamp;
  auto last_report = std::chrono::steady_clock::now();
  auto last_display = last_report - std::chrono::seconds(1);
  unsigned frames = 0;
  double fps = 0.0;
  while (!exiter.exit()) {
    camera.read(img, timestamp);
    if (img.empty()) continue;
    const auto now = std::chrono::steady_clock::now();
    ++frames;
    const double elapsed = std::chrono::duration<double>(now - last_report).count();
    if (elapsed >= 1.0) {
      fps = frames / elapsed;
      tools::logger()->info("[PREVIEW] received={:.1f} fps; display<=30 fps; NO SERIAL/CONTROL", fps);
      frames = 0;
      last_report = now;
    }
    if (!display || now - last_display < std::chrono::milliseconds(33)) continue;
    last_display = now;
    // Camera::read supplies RGB; HighGUI expects BGR.
    cv::cvtColor(img, preview, cv::COLOR_RGB2BGR);
    const auto label = cv::format("Camera only | %.1f fps | NO SERIAL / CONTROL | q or Esc: close", fps);
    cv::putText(preview, label, cv::Point(16, 34), cv::FONT_HERSHEY_SIMPLEX,
                0.65, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::imshow(title, preview);
    const int key = cv::waitKey(1) & 0xff;
    if (key == 'q' || key == 27) break;
  }
  if (display) cv::destroyWindow(title);
}
