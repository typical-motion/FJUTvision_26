#ifndef RM_OMNIPERCEPTION__USBCAMERA__USBCAMERA_HPP_
#define RM_OMNIPERCEPTION__USBCAMERA__USBCAMERA_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

namespace rm_omniperception {

class USBCamera {
public:
  struct Config {
    int width = 1280;
    int height = 1024;
    int fps = 120;
    int exposure = 100;
    int gamma = 80;
    int gain = 15;
  };

  explicit USBCamera(const std::string &device_name, const Config &config);
  ~USBCamera();

  bool read(
      cv::Mat &img,
      std::chrono::steady_clock::time_point &timestamp,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(40));

  std::string deviceName() const;
  bool isOpened() const;

private:
  bool openCamera();
  void closeCamera();
  void configureCamera();
  void captureLoop();
  std::string getDevicePath() const;

  const std::string device_name_;
  const Config config_;

  mutable std::mutex cap_mutex_;
  cv::VideoCapture cap_;

  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  cv::Mat latest_frame_;
  std::chrono::steady_clock::time_point latest_stamp_;
  bool has_frame_ = false;

  std::atomic_bool running_{true};
  std::thread capture_thread_;
};

}  // namespace rm_omniperception

#endif  // RM_OMNIPERCEPTION__USBCAMERA__USBCAMERA_HPP_