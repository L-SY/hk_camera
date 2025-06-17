//
// Created by lsy on 25-6-17.
//

#include "hk_camera/hk_camera_node.h"

HKCameraNode::HKCameraNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
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
  initDynamicReconfigure();
}

void HKCameraNode::initDynamicReconfigure() {
  dyn_servers_.resize(configs_.size());
  initialize_flags_.resize(configs_.size());
  for (size_t i = 0; i < configs_.size(); ++i) {
    // NodeHandle in camera-specific namespace
    ros::NodeHandle cam_pnh(pnh_, configs_[i].name);
    auto server = std::make_shared<dynamic_reconfigure::Server<hk_camera::CameraConfig>>(cam_pnh);
    dynamic_reconfigure::Server<hk_camera::CameraConfig>::CallbackType cb;
    cb = boost::bind(&HKCameraNode::reconfigCallback, this, i, _1, _2);
    server->setCallback(cb);
    dyn_servers_[i] = server;

    ROS_INFO("Dynamic reconfigure server created for camera: %s", configs_[i].name.c_str());
  }
}

void HKCameraNode::reconfigCallback(size_t cam_idx, hk_camera::CameraConfig& config, uint32_t level) {
  const auto& name = configs_[cam_idx].name;

  CameraParams params = configs_[cam_idx];
  if (!initialize_flags_[cam_idx])
  {
    config.exposure_auto = params.exposure_auto;
    config.exposure_value = params.exposure_value;
    config.auto_exposure_min = params.auto_exposure_min;
    config.auto_exposure_max = params.auto_exposure_max;
    config.gain_auto = params.gain_auto;
    config.gain_value = params.gain_value;
    config.gamma_selector = params.gamma_selector;
    config.gamma_value = params.gamma_value;
    config.balance_white_auto = params.balance_white_auto;
    initialize_flags_[cam_idx] = true;
  }
  ROS_INFO_STREAM(params.balance_white_auto);
  ROS_INFO("[%s] Reconfigure: exp=%.1f (auto=%s), gain=%.1f (auto=%s), balance_white_auto=%s, gamma_sel=%d, gamma=%.2f",
           name.c_str(),
           config.exposure_value,
           config.exposure_auto ? "true" : "false",
           config.gain_value,
           config.gain_auto ? "true" : "false",
           config.balance_white_auto ? "true" : "false",
           config.gamma_selector,
           config.gamma_value);

  // exposure
  params.exposure_auto = config.exposure_auto;
  params.exposure_value = static_cast<float>(config.exposure_value);
  params.auto_exposure_min = static_cast<int64_t>(config.auto_exposure_min);
  params.auto_exposure_max = static_cast<int64_t>(config.auto_exposure_max);
  // gain
  params.gain_auto = config.gain_auto;
  params.gain_value = static_cast<float>(config.gain_value);
  params.auto_gain_min = static_cast<float>(config.auto_gain_min);
  params.auto_gain_max = static_cast<float>(config.auto_gain_max);
  // white balance
  params.balance_white_auto = config.balance_white_auto;
  // gamma
  params.gamma_selector = config.gamma_selector;
  params.gamma_value = static_cast<float>(config.gamma_value);
  // roi can not change will use!!!

  // 获取相机句柄并调用统一 setParameter
  void* handle = cam_mgr_.getHandle(cam_idx);
  int ret = cam_mgr_.setParameter(handle, params);
  if (ret != MV_OK) {
    ROS_WARN("[%s] setParameter failed: 0x%X", name.c_str(), ret);
  }

  // 保存更新备份
  configs_[cam_idx] = params;
}

bool HKCameraNode::loadConfigs() {
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
    cfg.exposure_value             = static_cast<float>(getDouble(c["exposure"]["value"]));
    cfg.exposure_auto             = static_cast<int>(getDouble(c["exposure"]["auto"]));
    cfg.auto_exposure_min    = static_cast<int64_t>(getDouble(c["exposure"]["min"]));
    cfg.auto_exposure_max    = static_cast<int64_t>(getDouble(c["exposure"]["max"]));

    // Gain
    cfg.gain_value                = static_cast<float>(getDouble(c["gain"]["value"]));
    cfg.gain_auto                 = static_cast<int>(getDouble(c["gain"]["auto"]));
    cfg.auto_gain_min             = static_cast<float>(getDouble(c["gain"]["min"]));
    cfg.auto_gain_max             = static_cast<float>(getDouble(c["gain"]["max"]));

    // White balance auto
    cfg.balance_white_auto        = static_cast<int>(getDouble(c["white_balance"]["auto"]));
    ROS_INFO_STREAM(cfg.balance_white_auto);
    // ROI
    cfg.width                = static_cast<int>(getDouble(c["roi"]["width"]));
    cfg.height                 = static_cast<int>(getDouble(c["roi"]["height"]));
    cfg.offset_x             = static_cast<int>(getDouble(c["roi"]["offset_x"]));
    cfg.offset_y             = static_cast<int>(getDouble(c["roi"]["offset_y"]));

    configs_.push_back(cfg);
  }

  return !configs_.empty();
}

void HKCameraNode::setupPublishers() {
  pubs_.clear();
  for (const auto& cfg : configs_) {
    std::string topic = "/hk_camera/" + cfg.name + "/image_raw";
    pubs_.push_back(it_.advertise(topic, 1));
    ROS_INFO("Advertising on %s", topic.c_str());
  }
}

void HKCameraNode::spin() {
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
  HKCameraNode node(nh, pnh);
  node.spin();
  return 0;
}