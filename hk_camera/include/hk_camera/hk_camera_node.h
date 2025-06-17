#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "hk_camera/camera_manager.h"
#include <vector>
#include <string>

#include <dynamic_reconfigure/server.h>
#include <hk_camera/CameraConfig.h>

/**
 * ROS node to manage multiple cameras via hk_camera::CameraManager
 * - loads parameters from ROS param server
 * - initializes CameraManager with configs
 * - publishes images on separate topics per camera
 * - supports dynamic_reconfigure for exposure, gain, white balance, gamma
 */
class HKCameraNode {
public:
  HKCameraNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void spin();

private:
  bool loadConfigs();
  void setupPublishers();
  void initDynamicReconfigure();
  void reconfigCallback(size_t cam_idx, hk_camera::CameraConfig& config, uint32_t level);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  std::vector<image_transport::Publisher> pubs_;
  std::vector<CameraParams> configs_;
  CameraManager cam_mgr_;
  int loop_rate_hz_;

  std::vector<std::shared_ptr<dynamic_reconfigure::Server<hk_camera::CameraConfig>>> dyn_servers_;
};