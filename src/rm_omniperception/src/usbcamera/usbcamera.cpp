#include "rm_omniperception/usbcamera/usbcamera.hpp"

#include <chrono>

#include <rclcpp/rclcpp.hpp>

namespace rm_omniperception {
namespace {
constexpr auto kRetryInterval = std::chrono::milliseconds(200);
constexpr auto kReadFailInterval = std::chrono::milliseconds(30);
}

USBCamera::USBCamera(const std::string &device_name, const Config &config)
    : device_name_(device_name), config_(config) {
  capture_thread_ = std::thread(&USBCamera::captureLoop, this);
}

USBCamera::~USBCamera() {
  running_.store(false);
  frame_cv_.notify_all();
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  closeCamera();
}

bool USBCamera::read(
    cv::Mat &img,
    std::chrono::steady_clock::time_point &timestamp,
    std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(frame_mutex_);
  const bool ok = frame_cv_.wait_for(lock, timeout, [this] {
    return has_frame_ || !running_.load();
  });
  if (!ok || !has_frame_) {
    return false;
  }
  img = latest_frame_.clone();
  timestamp = latest_stamp_;
  return !img.empty();
}

std::string USBCamera::deviceName() const {
  return device_name_;
}

bool USBCamera::isOpened() const {
  std::lock_guard<std::mutex> lock(cap_mutex_);
  return cap_.isOpened();
}

bool USBCamera::openCamera() {
  std::lock_guard<std::mutex> lock(cap_mutex_);
  if (cap_.isOpened()) {
    return true;
  }

  const bool opened = cap_.open(getDevicePath(), cv::CAP_V4L2);
  if (!opened) {
    return false;
  }
  configureCamera();
  RCLCPP_INFO(rclcpp::get_logger("rm_omniperception.usbcamera"), "Opened USB camera: %s", getDevicePath().c_str());
  return true;
}

void USBCamera::closeCamera() {
  std::lock_guard<std::mutex> lock(cap_mutex_);
  if (cap_.isOpened()) {
    cap_.release();
  }
}

void USBCamera::configureCamera() {
  cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cap_.set(cv::CAP_PROP_FPS, config_.fps);
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
  cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
  cap_.set(cv::CAP_PROP_EXPOSURE, config_.exposure);
  cap_.set(cv::CAP_PROP_GAMMA, config_.gamma);
  cap_.set(cv::CAP_PROP_GAIN, config_.gain);
}

void USBCamera::captureLoop() {
  while (running_.load()) {
    if (!openCamera()) {
      std::this_thread::sleep_for(kRetryInterval);
      continue;
    }

    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(cap_mutex_);
      if (!cap_.read(frame) || frame.empty()) {
        cap_.release();
        std::this_thread::sleep_for(kReadFailInterval);
        continue;
      }
    }

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_frame_ = frame;
      latest_stamp_ = std::chrono::steady_clock::now();
      has_frame_ = true;
    }
    frame_cv_.notify_all();
  }
}

std::string USBCamera::getDevicePath() const {
  if (device_name_.rfind("/dev/", 0) == 0) {
    return device_name_;
  }
  return "/dev/" + device_name_;
}

}  // namespace rm_omniperception
