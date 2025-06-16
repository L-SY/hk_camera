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

struct CameraParams
{
  int exposure_mode;
  float exposure_time;
  int exposure_auto;
  int64_t auto_exposure_time_min;
  int64_t auto_exposure_time_max;

  float gain_value;
  int gain_auto;
  float auto_gain_min;
  float auto_gain_max;

  // Gamma Notice cs050 and cs020 is not suppose this config!!!
  int gamma_selector;
  float gamma_value;
  bool gamma_enable;

  // ROI Notice do not suppose dynamic config!!!
  int64_t width;
  int64_t height;
  int64_t offset_x;
  int64_t offset_y;

  // Color
  int balance_white_auto;
};

class CameraManager {
public:
  CameraManager();
  ~CameraManager();

  bool init();
  bool start();
  void stop();
  void triggerAll();
  bool getImage(int idx, cv::Mat& image);
  int numCameras();
  void* getHandle(size_t index) const;
  int setParameter(void* dev_handle_, CameraParams& config);

private:
  // 相机初始化参数结构体
  struct CameraContext {
    void* handle = nullptr;
    std::queue<cv::Mat> image_queue;
    std::atomic<bool> running{false};
    std::mutex mtx;
    std::string serial_number;
    CameraParams params;

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

