//
// Created by lsy on 25-6-17.
//

#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "hk_camera/camera_manager.h"
#include <vector>
#include <string>

/**
 * ROS node to manage multiple cameras via hk_camera::CameraManager
 * - loads parameters from ROS param server
 * - initializes CameraManager with configs
 * - publishes images on separate topics per camera
 */
class HKCameraROSNode {
public:
  HKCameraROSNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void spin();

private:
  bool loadConfigs();
  void setupPublishers();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  std::vector<image_transport::Publisher> pubs_;
  std::vector<CameraParams> configs_;
  CameraManager cam_mgr_;
  int loop_rate_hz_;
};
