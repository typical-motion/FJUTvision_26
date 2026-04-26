#ifndef RM_OMNIPERCEPTION__OMNIPERCEPTION__PERCEPTRON_HPP_
#define RM_OMNIPERCEPTION__OMNIPERCEPTION__PERCEPTRON_HPP_

#include <atomic>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "armor_detector/armor_detector.hpp"
#include "rm_utils/common.hpp"
#include "rm_omniperception/omniperception/decider.hpp"
#include "rm_omniperception/omniperception/detection.hpp"
#include "rm_omniperception/usbcamera/usbcamera.hpp"

namespace rm_omniperception {

class Perceptron {
public:
  struct DetectorConfig {
    int binary_thres = 160;
    fyt::EnemyColor enemy_color = fyt::EnemyColor::RED;
    fyt::auto_aim::Detector::LightParams light_params{0.08, 0.4, 40.0, 25};
    fyt::auto_aim::Detector::ArmorParams armor_params{0.6, 0.8, 3.2, 3.2, 5.0, 35.0};

    bool use_classifier = true;
    std::string model_url = "package://armor_detector/model/lenet.onnx";
    std::string label_url = "package://armor_detector/model/label.txt";
    double classifier_threshold = 0.7;
    std::vector<std::string> ignore_classes{"negative"};
    bool use_pca = true;
  };

  Perceptron(
      std::vector<std::shared_ptr<USBCamera>> cameras,
      Decider::Config decider_config,
      DetectorConfig detector_config);
  ~Perceptron();

  // 取经过优先级排序后的最新检测结果
  bool popLatestDetection(DetectionResult &result);

  void setEnemyColor(fyt::EnemyColor color);

  // 接收无敌装甲板 ID
  void receiveInvincibleArmor(const std::vector<int8_t> &invincible_enemy_ids);

  Decider &decider() { return decider_; }

private:
  std::unique_ptr<fyt::auto_aim::Detector> createDetector() const;
  void processCamera(std::size_t index);
  void pushDetection(const DetectionResult &result);

  std::vector<std::shared_ptr<USBCamera>> cameras_;
  std::vector<std::unique_ptr<fyt::auto_aim::Detector>> detectors_;
  std::vector<std::thread> threads_;

  Decider decider_;
  DetectorConfig detector_config_;

  std::mutex queue_mutex_;
  std::deque<DetectionResult> detection_queue_;
  const std::size_t queue_capacity_ = 128;

  std::atomic_bool running_{true};
};

}  // namespace rm_omniperception

#endif  // RM_OMNIPERCEPTION__OMNIPERCEPTION__PERCEPTRON_HPP_