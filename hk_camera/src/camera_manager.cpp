// camera_manager.cpp
#include "hk_camera/camera_manager.h"
#include <iostream>
#include <cstring>

CameraManager::CameraManager() {}

CameraManager::~CameraManager() {
  stop();
  for (auto& cam : cameras_) {
    if (cam.handle) {
      MV_CC_DestroyHandle(cam.handle);
      cam.handle = nullptr;
    }
  }
  MV_CC_Finalize();
}

bool CameraManager::init() {
  int nRet = MV_CC_Initialize();
  if (nRet != MV_OK) {
    std::cerr << "MV_CC_Initialize failed: 0x" << std::hex << nRet << std::endl;
    return false;
  }

  MV_CC_DEVICE_INFO_LIST dev_list;
  cameras_.reserve(dev_list.nDeviceNum);
  memset(&dev_list, 0, sizeof(dev_list));
  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &dev_list);
  if (nRet != MV_OK || dev_list.nDeviceNum == 0) {
    std::cerr << "No camera found or MV_CC_EnumDevices error: 0x" << std::hex << nRet << std::endl;
    return false;
  }

  std::cout << "Detected " << dev_list.nDeviceNum << " camera(s)." << std::endl;

  for (unsigned i = 0; i < dev_list.nDeviceNum; ++i) {
    cameras_.emplace_back();
    CameraContext& ctx = cameras_.back();

    // 创建并打开句柄
    nRet = MV_CC_CreateHandle(&ctx.handle, dev_list.pDeviceInfo[i]);
    if (nRet != MV_OK) {
      std::cerr << "CreateHandle failed for camera " << i << ": 0x" << std::hex << nRet << std::endl;
      continue;
    }
    nRet = MV_CC_OpenDevice(ctx.handle);
    if (nRet != MV_OK) {
      std::cerr << "OpenDevice failed for camera " << i << ": 0x" << std::hex << nRet << std::endl;
      MV_CC_DestroyHandle(ctx.handle);
      continue;
    }

    // 配置触发模式
    MV_CC_SetEnumValue(ctx.handle, "TriggerMode", 1);
    MV_CC_SetEnumValue(ctx.handle, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE);
    MV_CC_SetBoolValue(ctx.handle, "AcquisitionFrameRateEnable", false);

    // 从设备信息中读取 USB3.0 相机序列号
    if (dev_list.pDeviceInfo[i]->nTLayerType == MV_USB_DEVICE) {
      auto& usbInfo = dev_list.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo;
      ctx.serial_number = std::string(reinterpret_cast<char*>(usbInfo.chSerialNumber));
    } else {
      ctx.serial_number = "UNKNOWN";
    }
    ctx.running = true;

    // 注册回调并开始取流
    CameraContext& ref = cameras_.back();
    MV_CC_RegisterImageCallBackEx(ref.handle, imageCallback, &ref);
    MV_CC_StartGrabbing(ref.handle);
    std::cout << "Camera " << i << " (S/N: " << ref.serial_number << ") initialized and grabbing." << std::endl;
  }

  std::cout << "Total initialized cameras: " << cameras_.size() << std::endl;
  return !cameras_.empty();
}

bool CameraManager::start() {
  running_ = true;
  trigger_thread_ = std::thread([this]() {
    while (running_) {
      triggerAll();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });
  return true;
}

void CameraManager::stop() {
  running_ = false;
  if (trigger_thread_.joinable()) trigger_thread_.join();
  for (auto& cam : cameras_) {
    cam.running = false;
    if (cam.handle) MV_CC_StopGrabbing(cam.handle);
  }
}

void CameraManager::triggerAll() {
  for (auto& cam : cameras_) {
    MV_CC_SetCommandValue(cam.handle, "TriggerSoftware");
  }
}

bool CameraManager::getImage(int cam_idx, cv::Mat& image) {
  if (cam_idx < 0 || cam_idx >= static_cast<int>(cameras_.size())) return false;
  auto& cam = cameras_[cam_idx];
  std::lock_guard<std::mutex> lock(cam.mtx);
  if (cam.image_queue.empty()) return false;
  image = cam.image_queue.front();
  cam.image_queue.pop();
  return true;
}

size_t CameraManager::numCameras() {
  return cameras_.size();
}

void __stdcall CameraManager::imageCallback(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
  if (!pData || !pFrameInfo || !pUser) return;
  CameraContext* ctx = static_cast<CameraContext*>(pUser);
  std::cout << "[Callback] Received image from camera (S/N: " << ctx->serial_number << ")" << std::endl;
  enqueueImage(*ctx, pData, pFrameInfo);
}

void CameraManager::enqueueImage(CameraContext& ctx, unsigned char* data, MV_FRAME_OUT_INFO_EX* info) {
  cv::Mat img(info->nHeight, info->nWidth, CV_8UC1, data);
  std::lock_guard<std::mutex> lock(ctx.mtx);
  ctx.image_queue.push(img.clone());
}
