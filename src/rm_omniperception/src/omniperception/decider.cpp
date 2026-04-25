#include "rm_omniperception/omniperception/decider.hpp"

#include <algorithm>
#include <cmath>

namespace rm_omniperception {
namespace {
constexpr double kDegToRad = M_PI / 180.0;
}

Decider::Decider(Config config) : config_(std::move(config)) {}

std::optional<DetectionResult> Decider::makeDetection(
    const std::vector<fyt::auto_aim::Armor> &armors,
    const cv::Size &img_size,
    std::size_t camera_index,
    std::chrono::steady_clock::time_point timestamp) const {
  if (armors.empty() || img_size.width <= 0 || img_size.height <= 0) {
    return std::nullopt;
  }

  std::vector<fyt::auto_aim::Armor> filtered;
  filtered.reserve(armors.size());
  for (const auto &armor : armors) {
    if (!shouldIgnore(armor)) {
      filtered.push_back(armor);
    }
  }
  if (filtered.empty()) {
    return std::nullopt;
  }

  const auto *best = pickBest(filtered, img_size);
  if (best == nullptr) {
    return std::nullopt;
  }

  const double nx = static_cast<double>(best->center.x) / static_cast<double>(img_size.width);
  const double ny = static_cast<double>(best->center.y) / static_cast<double>(img_size.height);

  const double yaw_deg = getOffset(config_.camera_yaw_offsets_deg, camera_index) +
                         (0.5 - nx) * config_.fov_h_deg;
  const double pitch_deg = getOffset(config_.camera_pitch_offsets_deg, camera_index) +
                           (ny - 0.5) * config_.fov_v_deg;

  DetectionResult result;
  result.armors = std::move(filtered);
  result.timestamp = timestamp;
  result.delta_yaw = yaw_deg * kDegToRad;
  result.delta_pitch = pitch_deg * kDegToRad;
  result.camera_index = camera_index;
  return result;
}

bool Decider::shouldIgnore(const fyt::auto_aim::Armor &armor) const {
  if (config_.ignore_outpost && armor.number == "outpost") {
    return true;
  }
  if (config_.ignore_base && armor.number == "base") {
    return true;
  }

  return std::find(
             config_.ignore_numbers.begin(), config_.ignore_numbers.end(), armor.number) !=
         config_.ignore_numbers.end();
}

const fyt::auto_aim::Armor *Decider::pickBest(
    const std::vector<fyt::auto_aim::Armor> &armors,
    const cv::Size &img_size) const {
  if (armors.empty()) {
    return nullptr;
  }

  const cv::Point2f center(
      static_cast<float>(img_size.width) / 2.0F,
      static_cast<float>(img_size.height) / 2.0F);

  auto cmp = [&center](const fyt::auto_aim::Armor &lhs, const fyt::auto_aim::Armor &rhs) {
    const float dl = cv::norm(lhs.center - center);
    const float dr = cv::norm(rhs.center - center);
    return dl < dr;
  };

  return &(*std::min_element(armors.begin(), armors.end(), cmp));
}

double Decider::getOffset(
    const std::vector<double> &offsets,
    std::size_t camera_index) const {
  if (offsets.empty()) {
    return 0.0;
  }
  if (camera_index >= offsets.size()) {
    return offsets.back();
  }
  return offsets[camera_index];
}

}  // namespace rm_omniperception
