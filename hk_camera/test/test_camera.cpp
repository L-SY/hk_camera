#include "hk_camera/camera_manager.h"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

int main() {
  CameraManager cam_mgr;
  if (!cam_mgr.init()) {
    std::cerr << "❌ Failed to initialize camera(s)." << std::endl;
    return -1;
  }

  cam_mgr.start();
  std::cout << "✅ Camera started. Press ESC to exit.\n";

  const int N = cam_mgr.numCameras();
  // 用于打印首次取到图像
  std::vector<bool> had_image(N, false);

  // 帧计数和 fps 存储
  std::vector<int> frame_count(N, 0);
  std::vector<double> fps(N, 0.0);

  // 记录上次打印 fps 的时间点
  auto last_time = std::chrono::steady_clock::now();

  while (true) {
    bool got_any = false;

    for (int i = 0; i < N; ++i) {
      cv::Mat img;
      if (cam_mgr.getImage(i, img)) {
        // 首次打印分辨率
        if (!had_image[i]) {
//          std::cout << "✅ Got image from Camera " << i
//                    << " | Resolution: " << img.cols << "x" << img.rows << std::endl;
          had_image[i] = true;
        }
        // 增加帧计数
        frame_count[i]++;
        got_any = true;
      } else {
        had_image[i] = false;
      }
    }

    // 计算是否到达 1 秒间隔
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_time).count();
    if (elapsed >= 1.0) {
      // 打印并重置计数
      std::cout << "[FPS] ";
      for (int i = 0; i < N; ++i) {
        fps[i] = frame_count[i] / elapsed;
        std::cout << "Cam" << i << ": " << fps[i] << "  ";
        frame_count[i] = 0;
      }
      std::cout << std::endl;
      last_time = now;
    }

    // 如果不使用 OpenCV 窗口，只需小延迟即可
//    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    // ESC 键退出
    if (cv::waitKey(1) == 27) break;
  }

  cam_mgr.stop();
  std::cout << "🛑 Camera stopped.\n";
  return 0;
}
