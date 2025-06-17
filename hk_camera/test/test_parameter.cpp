// test_parameter.cpp
#include "hk_camera/camera_manager.h"
#include <iostream>

int main(int argc, char** argv) {
  CameraManager cam_mgr;
  // 1) 初始化
  if (!cam_mgr.init()) {
    std::cerr << "❌ Failed to initialize camera(s)." << std::endl;
    return -1;
  }

  int N = static_cast<int>(cam_mgr.numCameras());
  if (N == 0) {
    std::cerr << "❌ No cameras found." << std::endl;
    return -1;
  }

  // 3) 配置初始参数
  CameraParams params;
  // —— 曝光 ——
  params.exposure_auto = true;
  params.auto_exposure_time_min = 500;    // µs
  params.auto_exposure_time_max = 20000;  // µs
  params.exposure_time = 10000;           // 当手动模式下使用

  // —— 增益 ——
  params.gain_auto = false;
  params.auto_gain_min = 0.0f;
  params.auto_gain_max = 5.0f;
  params.gain_value = 1.5f;

  // —— 白平衡 ——
  params.balance_white_auto = true;

  // —— Gamma ——
  params.gamma_selector = 1;    // 1=User,2=sRGB
  params.gamma_value    = 2.2f; // 仅在 User 模式下生效

  // —— ROI/分辨率 ——
  params.width    = 640;
  params.height   = 480;
  params.offset_x = 0;
  params.offset_y = 0;

  // 4) 对每个相机执行 ParameterSet，并打印设置前后的结果
  for (int i = 0; i < N; ++i) {
    std::cout << "\n=== Testing Camera " << i << " ===\n";
    void* handle = cam_mgr.getHandle(i);
    // 读取一份本地拷贝，用于对比
    CameraParams before = params;

    // 设置参数
    cam_mgr.setParameter(handle, params);

    // 打印对比
    std::cout << "Exposure: "
              << (before.exposure_auto ? "Auto" : "Manual")
              << " -> "
              << params.exposure_time << " µs\n";

    std::cout << "Gain: "
              << (before.gain_auto ? "Auto" : "Manual")
              << " -> "
              << params.gain_value << "\n";

    std::cout << "ROI: "
              << before.width << "x" << before.height
              << "@" << before.offset_x << "," << before.offset_y
              << " -> "
              << params.width << "x" << params.height
              << "@" << params.offset_x << "," << params.offset_y
              << "\n";
  }
  // 2) 启动采集
  cam_mgr.start();
  std::cout << "✅ Camera(s) started.\n";

  // 5) 停止采集并退出
  cam_mgr.stop();
  std::cout << "\n🛑 Camera(s) stopped.\n";
  return 0;
}
