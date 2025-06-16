#include "hk_camera/camera_manager.h"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
  CameraManager cam_mgr;

  if (!cam_mgr.init()) {
    std::cerr << "❌ Failed to initialize camera(s)." << std::endl;
    return -1;
  }

  cam_mgr.start();
  std::cout << "✅ Camera started. Press ESC to exit.\n";

  // 上一次是否有图像可用，用于控制是否打印
  std::vector<bool> had_image(cam_mgr.numCameras(), false);

  while (true) {
    bool got_any = false;

    for (int i = 0; i < cam_mgr.numCameras(); ++i) {
      cv::Mat img;
      if (cam_mgr.getImage(i, img)) {
        if (!had_image[i]) {
          std::cout << "✅ Got image from Camera " << i
                    << " | Resolution: " << img.cols << "x" << img.rows << std::endl;
          had_image[i] = true;
        }
        // 如需可视化，可启用：
        // cv::imshow("Camera " + std::to_string(i), img);
        got_any = true;
      } else {
        had_image[i] = false;
      }
    }

    // 如果不使用 OpenCV 窗口，加入小延迟即可
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    // ESC 键退出
    if (cv::waitKey(1) == 27) break;
  }

  cam_mgr.stop();
  std::cout << "🛑 Camera stopped.\n";
  return 0;
}
