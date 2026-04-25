#include "rm_omniperception/omniperception/perceptron.hpp"

#include <filesystem>

#include <rclcpp/rclcpp.hpp>

#include "armor_detector/light_corner_corrector.hpp"
#include "armor_detector/number_classifier.hpp"
#include "rm_utils/url_resolver.hpp"

namespace rm_omniperception {

Perceptron::Perceptron(
    std::vector<std::shared_ptr<USBCamera>> cameras,
    Decider::Config decider_config,
    DetectorConfig detector_config)
    : cameras_(std::move(cameras)),
      decider_(std::move(decider_config)),
      detector_config_(std::move(detector_config)) {
  detectors_.reserve(cameras_.size());
  threads_.reserve(cameras_.size());

  for (std::size_t i = 0; i < cameras_.size(); ++i) {
    detectors_.push_back(createDetector());
    threads_.emplace_back(&Perceptron::processCamera, this, i);
  }
}

Perceptron::~Perceptron() {
  running_.store(false);
  for (auto &thread : threads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
}

bool Perceptron::popLatestDetection(DetectionResult &result) {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (detection_queue_.empty()) {
    return false;
  }
  result = detection_queue_.back();
  detection_queue_.clear();
  return true;
}

std::unique_ptr<fyt::auto_aim::Detector> Perceptron::createDetector() const {
  auto detector = std::make_unique<fyt::auto_aim::Detector>(
      detector_config_.binary_thres,
      detector_config_.enemy_color,
      detector_config_.light_params,
      detector_config_.armor_params);

  if (detector_config_.use_classifier) {
    try {
      const auto model_path =
          fyt::utils::URLResolver::getResolvedPath(detector_config_.model_url);
      const auto label_path =
          fyt::utils::URLResolver::getResolvedPath(detector_config_.label_url);

      if (std::filesystem::exists(model_path) && std::filesystem::exists(label_path)) {
        detector->classifier = std::make_unique<fyt::auto_aim::NumberClassifier>(
            model_path.string(),
            label_path.string(),
            detector_config_.classifier_threshold,
            detector_config_.ignore_classes);
      } else {
        RCLCPP_WARN(
            rclcpp::get_logger("rm_omniperception.perceptron"),
            "Classifier model not found, fallback to unclassified detector.");
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(
          rclcpp::get_logger("rm_omniperception.perceptron"),
          "Failed to initialize classifier: %s",
          e.what());
    }
  }

  if (detector_config_.use_pca) {
    detector->corner_corrector = std::make_unique<fyt::auto_aim::LightCornerCorrector>();
  }

  return detector;
}

void Perceptron::processCamera(std::size_t index) {
  if (index >= cameras_.size() || index >= detectors_.size()) {
    return;
  }

  auto &camera = cameras_.at(index);
  auto &detector = detectors_.at(index);

  while (running_.load()) {
    cv::Mat frame;
    std::chrono::steady_clock::time_point stamp;
    if (!camera->read(frame, stamp)) {
      continue;
    }

    auto armors = detector->detect(frame);
    auto result = decider_.makeDetection(armors, frame.size(), index, stamp);
    if (result.has_value()) {
      pushDetection(result.value());
    }
  }
}

void Perceptron::pushDetection(const DetectionResult &result) {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (detection_queue_.size() >= queue_capacity_) {
    detection_queue_.pop_front();
  }
  detection_queue_.push_back(result);
}

}  // namespace rm_omniperception
