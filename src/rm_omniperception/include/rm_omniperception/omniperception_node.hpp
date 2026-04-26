#ifndef RM_OMNIPERCEPTION__OMNIPERCEPTION_NODE_HPP_
#define RM_OMNIPERCEPTION__OMNIPERCEPTION_NODE_HPP_

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_omniperception/omniperception/perceptron.hpp"
#include "rm_omniperception/usbcamera/usbcamera.hpp"

namespace rm_omniperception {

class OmniPerceptionNode : public rclcpp::Node {
public:
  explicit OmniPerceptionNode(const rclcpp::NodeOptions &options);
  ~OmniPerceptionNode() override;

private:
  void onTimer();
  void initCameras();
  void setModeCallback(
    const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);
  Perceptron::DetectorConfig readDetectorConfig();
  Decider::Config readDeciderConfig();

  int camera_count_ = 1;
  std::vector<std::string> camera_devices_;
  std::vector<std::shared_ptr<USBCamera>> cameras_;
  std::unique_ptr<Perceptron> perceptron_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
};

} // namespace rm_omniperception

#endif // RM_OMNIPERCEPTION__OMNIPERCEPTION_NODE_HPP_
