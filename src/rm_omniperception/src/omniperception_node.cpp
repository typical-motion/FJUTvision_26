#include "rm_omniperception/omniperception_node.hpp"

#include <algorithm>
#include <chrono>

#include "rm_utils/common.hpp"

namespace rm_omniperception {

OmniPerceptionNode::OmniPerceptionNode(const rclcpp::NodeOptions &options)
    : Node("rm_omniperception", options) {
  RCLCPP_INFO(this->get_logger(), "Starting OmniPerceptionNode...");

  camera_count_ = this->declare_parameter("camera_count", 3);
  camera_count_ = std::clamp(camera_count_, 1, 3);

  camera_devices_ = this->declare_parameter<std::vector<std::string>>(
      "camera_devices", {"video0", "video2", "video4"});
  if (camera_devices_.size() < static_cast<std::size_t>(camera_count_)) {
    camera_devices_.resize(static_cast<std::size_t>(camera_count_), "video0");
  }

  initCameras();
  perceptron_ = std::make_unique<Perceptron>(
      cameras_, readDeciderConfig(), readDetectorConfig());

  gimbal_cmd_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>(
      "omniperception/cmd_gimbal", rclcpp::SensorDataQoS());
  set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
      "rm_omniperception/set_mode",
      std::bind(&OmniPerceptionNode::setModeCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(8),
      std::bind(&OmniPerceptionNode::onTimer, this));
}

OmniPerceptionNode::~OmniPerceptionNode() {
  perceptron_.reset();
  cameras_.clear();
}

void OmniPerceptionNode::initCameras() {
  USBCamera::Config cam_cfg;
  cam_cfg.width = this->declare_parameter("usb.width", 1280);
  cam_cfg.height = this->declare_parameter("usb.height", 1024);
  cam_cfg.fps = this->declare_parameter("usb.fps", 120);
  cam_cfg.exposure = this->declare_parameter("usb.exposure", 100);
  cam_cfg.gamma = this->declare_parameter("usb.gamma", 80);
  cam_cfg.gain = this->declare_parameter("usb.gain", 15);

  cameras_.clear();
  cameras_.reserve(static_cast<std::size_t>(camera_count_));
  for (int i = 0; i < camera_count_; ++i) {
    cameras_.push_back(std::make_shared<USBCamera>(camera_devices_[i], cam_cfg));
  }
}

Perceptron::DetectorConfig OmniPerceptionNode::readDetectorConfig() {
  Perceptron::DetectorConfig cfg;
  const std::string color = this->declare_parameter("enemy_color", "red");
  cfg.enemy_color =
      (color == "blue" || color == "BLUE") ? fyt::EnemyColor::BLUE : fyt::EnemyColor::RED;

  cfg.binary_thres = this->declare_parameter("binary_thres", 160);
  cfg.light_params.min_ratio = this->declare_parameter("light.min_ratio", 0.08);
  cfg.light_params.max_ratio = this->declare_parameter("light.max_ratio", 0.4);
  cfg.light_params.max_angle = this->declare_parameter("light.max_angle", 40.0);
  cfg.light_params.color_diff_thresh =
      this->declare_parameter("light.color_diff_thresh", 25);

  cfg.armor_params.min_light_ratio = this->declare_parameter("armor.min_light_ratio", 0.6);
  cfg.armor_params.min_small_center_distance =
      this->declare_parameter("armor.min_small_center_distance", 0.8);
  cfg.armor_params.max_small_center_distance =
      this->declare_parameter("armor.max_small_center_distance", 3.2);
  cfg.armor_params.min_large_center_distance =
      this->declare_parameter("armor.min_large_center_distance", 3.2);
  cfg.armor_params.max_large_center_distance =
      this->declare_parameter("armor.max_large_center_distance", 5.0);
  cfg.armor_params.max_angle = this->declare_parameter("armor.max_angle", 35.0);

  cfg.use_classifier = this->declare_parameter("use_classifier", true);
  cfg.classifier_threshold = this->declare_parameter("classifier_threshold", 0.7);
  cfg.ignore_classes =
      this->declare_parameter<std::vector<std::string>>("ignore_classes", {"negative"});
  cfg.use_pca = this->declare_parameter("use_pca", true);
  cfg.model_url =
      this->declare_parameter("classifier_model", "package://armor_detector/model/lenet.onnx");
  cfg.label_url =
      this->declare_parameter("classifier_label", "package://armor_detector/model/label.txt");

  return cfg;
}

Decider::Config OmniPerceptionNode::readDeciderConfig() {
  Decider::Config cfg;
  cfg.camera_count = camera_count_;
  cfg.camera_yaw_offsets_deg = this->declare_parameter<std::vector<double>>(
      "camera_yaw_offsets_deg", {62.0, -62.0, 170.0});
  cfg.camera_pitch_offsets_deg = this->declare_parameter<std::vector<double>>(
      "camera_pitch_offsets_deg", {0.0, 0.0, 0.0});
  cfg.fov_h_deg = this->declare_parameter("fov_h_deg", 54.2);
  cfg.fov_v_deg = this->declare_parameter("fov_v_deg", 44.5);
  cfg.new_fov_h_deg = this->declare_parameter("new_fov_h_deg", 54.2);
  cfg.new_fov_v_deg = this->declare_parameter("new_fov_v_deg", 44.5);
  cfg.ignore_outpost = this->declare_parameter("ignore_outpost", true);
  cfg.ignore_base = this->declare_parameter("ignore_base", true);
  cfg.ignore_numbers = this->declare_parameter<std::vector<std::string>>(
      "ignore_numbers", std::vector<std::string>{"negative"});

  const int priority_mode = this->declare_parameter("priority_mode", 1);
  cfg.priority_mode = (priority_mode == 2) ? PriorityMode::MODE_TWO : PriorityMode::MODE_ONE;

  const std::string enemy_color = this->declare_parameter("enemy_color", "red");
  cfg.enemy_color = enemy_color;

  return cfg;
}

void OmniPerceptionNode::onTimer() {
  if (!perceptron_) {
    return;
  }

  DetectionResult result;
  if (!perceptron_->popLatestDetection(result)) {
    return;
  }

  rm_interfaces::msg::GimbalCmd cmd;
  cmd.header.stamp = this->now();
  cmd.yaw_diff = result.delta_yaw;
  cmd.pitch_diff = result.delta_pitch;
  cmd.yaw = 0.0;
  cmd.pitch = 0.0;
  cmd.distance = -1.0;
  cmd.fire_advice = !result.armors.empty();
  gimbal_cmd_pub_->publish(cmd);
}

void OmniPerceptionNode::setModeCallback(
    const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
  response->success = true;
  response->message = "0";

  const fyt::VisionMode mode = static_cast<fyt::VisionMode>(request->mode);

  switch (mode) {
  case fyt::VisionMode::AUTO_AIM_RED: {
    perceptron_->setEnemyColor(fyt::EnemyColor::RED);
    RCLCPP_INFO(this->get_logger(), "OmniPerception set to RED");
    break;
  }
  case fyt::VisionMode::AUTO_AIM_BLUE: {
    perceptron_->setEnemyColor(fyt::EnemyColor::BLUE);
    RCLCPP_INFO(this->get_logger(), "OmniPerception set to BLUE");
    break;
  }
  default: {
    // Rune modes — no action needed for omniperception
    response->success = true;
    break;
  }
  }
}

} // namespace rm_omniperception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_omniperception::OmniPerceptionNode)
