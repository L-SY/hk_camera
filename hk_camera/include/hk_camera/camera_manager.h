//
// Created by lsy on 25-6-16.
//

#pragma once

#include <opencv2/opencv.hpp>
#include <hk_camera/libMVSapi/MvCameraControl.h>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>

class CameraManager {
public:
  CameraManager();
  ~CameraManager();

  bool init();              // 初始化并打开所有相机
  bool start();             // 启动统一软触发线程
  void stop();              // 停止采图与释放资源
  void triggerAll();        // 手动触发所有相机一次
  bool getImage(int idx, cv::Mat& image); // 获取指定相机图像帧
  size_t numCameras();
private:
  struct CameraContext {
    void* handle = nullptr;
    std::queue<cv::Mat> image_queue;
    std::atomic<bool> running{false};
    std::mutex mtx;
    std::string serial_number;

    CameraContext() = default;
    CameraContext(const CameraContext&) = delete;
    CameraContext& operator=(const CameraContext&) = delete;

    // 手动定义移动构造函数
    CameraContext(CameraContext&& other) noexcept {
      handle = other.handle;
      image_queue = std::move(other.image_queue);
      running.store(other.running.load());
      // mtx 不可移动，使用默认初始化
    }

    CameraContext& operator=(CameraContext&& other) noexcept {
      if (this != &other) {
        handle = other.handle;
        image_queue = std::move(other.image_queue);
        running.store(other.running.load());
        // mtx 不可移动，使用默认初始化
      }
      return *this;
    }

    ~CameraContext() = default;
  };

  std::vector<CameraContext> cameras_;
  std::thread trigger_thread_;
  std::atomic<bool> running_{false};

  static void __stdcall imageCallback(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
  static void enqueueImage(CameraContext& ctx, unsigned char* data, MV_FRAME_OUT_INFO_EX* info);
};

