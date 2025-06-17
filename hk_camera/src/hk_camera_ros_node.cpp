//
// Created by lsy on 25-6-17.
//

#include "hk_camera/hk_camera_ros_node.h"

HKCameraROSNode::HKCameraROSNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), it_(nh_) {
  // load loop rate
  pnh_.param("loop_rate_hz", loop_rate_hz_, 30);

  if (!loadConfigs()) {
    ROS_FATAL("Failed to load camera configs");
    ros::shutdown();
  }
  // initialize camera manager
  if (!cam_mgr_.init(configs_)) {
    ROS_FATAL("CameraManager init failed");
    ros::shutdown();
  }
  cam_mgr_.start();
  setupPublishers();
}

bool HKCameraROSNode::loadConfigs() {
  XmlRpc::XmlRpcValue cam_list;
  if (!pnh_.getParam("cameras", cam_list) ||
      cam_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("Param 'cameras' missing or not a list");
    return false;
  }
  configs_.clear();
  configs_.reserve(cam_list.size());

  // Helper to extract numeric values safely
  auto getDouble = [&](const XmlRpc::XmlRpcValue& v) -> double {
    switch (v.getType()) {
    case XmlRpc::XmlRpcValue::TypeDouble:
      return static_cast<double>(v);
    case XmlRpc::XmlRpcValue::TypeInt:
      return static_cast<int>(v);
    case XmlRpc::XmlRpcValue::TypeBoolean:
      // true→1.0, false→0.0
      return v ? 1.0 : 0.0;
    default:
      ROS_WARN("Unexpected XmlRpcValue type (%d) for numeric parameter", v.getType());
      return 0.0;
    }
  };

  for (int i = 0; i < cam_list.size(); ++i) {
    const auto& c = cam_list[i];
    CameraParams cfg;

    cfg.name = static_cast<std::string>(c["name"]);
    // Serial number
    cfg.serial_number = static_cast<std::string>(c["serial_number"]);

    // Exposure
    cfg.exposure_mode             = static_cast<int>(getDouble(c["exposure"]["mode"]));
    cfg.exposure_time             = static_cast<float>(getDouble(c["exposure"]["time_us"]));
    cfg.exposure_auto             = static_cast<int>(getDouble(c["exposure"]["auto"]));
    cfg.auto_exposure_time_min    = static_cast<int64_t>(getDouble(c["exposure"]["ae_lower_us"]));
    cfg.auto_exposure_time_max    = static_cast<int64_t>(getDouble(c["exposure"]["ae_upper_us"]));

    // Gain
    cfg.gain_value                = static_cast<float>(getDouble(c["gain"]["value_db"]));
    cfg.gain_auto                 = static_cast<int>(getDouble(c["gain"]["auto"]));
    cfg.auto_gain_min             = static_cast<float>(getDouble(c["gain"]["gain_lower_db"]));
    cfg.auto_gain_max             = static_cast<float>(getDouble(c["gain"]["gain_upper_db"]));

    // White balance auto
    cfg.balance_white_auto        = static_cast<int>(getDouble(c["white_balance"]["auto"]));

    // ROI
    cfg.width                = static_cast<int>(getDouble(c["roi"]["width"]));
    cfg.height                 = static_cast<int>(getDouble(c["roi"]["height"]));
    cfg.offset_x             = static_cast<int>(getDouble(c["roi"]["offset_x"]));
    cfg.offset_y             = static_cast<int>(getDouble(c["roi"]["offset_y"]));

    configs_.push_back(cfg);
  }

  return !configs_.empty();
}

void HKCameraROSNode::setupPublishers() {
  pubs_.clear();
  for (const auto& cfg : configs_) {
    std::string topic = "/hk_camera/" + cfg.name + "/image_raw";
    pubs_.push_back(it_.advertise(topic, 1));
    ROS_INFO("Advertising on %s", topic.c_str());
  }
}

void HKCameraROSNode::spin() {
  ros::Rate rate(loop_rate_hz_);
  while (ros::ok()) {
    for (int i = 0; i < cam_mgr_.numCameras(); ++i) {
      cv::Mat img;
      if (!cam_mgr_.getImage(i, img)) {
        continue;
      }
      if (img.empty()) {
        std::cerr << "[publish] Camera " << i << " -> empty image, skip\n";
        continue;
      }

      std::string encoding;
      if (img.channels() == 1) {
        encoding = "mono8";
      } else if (img.channels() == 3) {
        encoding = "bgr8";
      } else {
        std::cerr << "[publish] Camera " << i << ": unsupported channels="
                  << img.channels() << ", skip\n";
        continue;
      }

      sensor_msgs::ImagePtr msg =
          cv_bridge::CvImage(std_msgs::Header(), encoding, img).toImageMsg();
      pubs_[i].publish(msg);
    }

    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera_manager_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  HKCameraROSNode node(nh, pnh);
  node.spin();
  return 0;
}