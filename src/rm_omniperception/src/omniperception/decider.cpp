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

  std::vector<fyt::auto_aim::Armor> filtered = armors;
  if (armor_filter(filtered)) {
    return std::nullopt;
  }

  set_priority(filtered);
  std::sort(filtered.begin(), filtered.end(),
    [this](const fyt::auto_aim::Armor &a, const fyt::auto_aim::Armor &b) {
      return getPriority(a.number) < getPriority(b.number);
    });

  const auto &best = filtered.front();
  const double nx = static_cast<double>(best.center.x) / static_cast<double>(img_size.width);
  const double ny = static_cast<double>(best.center.y) / static_cast<double>(img_size.height);

  // USB 辅瞄与背部相机使用不同 FOV
  const double fov_h = getFovH(camera_index);
  const double fov_v = getFovV(camera_index);

  const double yaw_deg = getOffset(config_.camera_yaw_offsets_deg, camera_index) +
                         (0.5 - nx) * fov_h;
  const double pitch_deg = getOffset(config_.camera_pitch_offsets_deg, camera_index) +
                           (ny - 0.5) * fov_v;

  DetectionResult result;
  result.armors = std::move(filtered);
  result.timestamp = timestamp;
  result.delta_yaw = yaw_deg * kDegToRad;
  result.delta_pitch = pitch_deg * kDegToRad;
  result.camera_index = camera_index;
  return result;
}

bool Decider::armor_filter(std::vector<fyt::auto_aim::Armor> &armors) const {
  if (armors.empty()) return true;

  if (config_.ignore_outpost) {
    armors.erase(std::remove_if(armors.begin(), armors.end(),
      [](const auto &a) { return a.number == "outpost"; }), armors.end());
  }
  if (config_.ignore_base) {
    armors.erase(std::remove_if(armors.begin(), armors.end(),
      [](const auto &a) { return a.number == "base"; }), armors.end());
  }
  if (!config_.ignore_numbers.empty()) {
    armors.erase(std::remove_if(armors.begin(), armors.end(),
      [this](const auto &a) {
        return std::find(config_.ignore_numbers.begin(), config_.ignore_numbers.end(),
                         a.number) != config_.ignore_numbers.end();
      }), armors.end());
  }

  // 25赛季没有5号装甲板
  armors.erase(std::remove_if(armors.begin(), armors.end(),
    [](const auto &a) { return a.number == "5"; }), armors.end());

  // 过滤无敌状态的装甲板
  if (!invincible_armors_.empty()) {
    armors.erase(std::remove_if(armors.begin(), armors.end(),
      [this](const auto &a) {
        return std::find(invincible_armors_.begin(), invincible_armors_.end(),
                         a.number) != invincible_armors_.end();
      }), armors.end());
  }

  return armors.empty();
}

void Decider::set_priority(std::vector<fyt::auto_aim::Armor> &armors) const {
  (void)armors;
}

void Decider::sort(std::vector<DetectionResult> &detection_queue) const {
  if (detection_queue.empty()) return;

  for (auto &dr : detection_queue) {
    if (armor_filter(dr.armors)) {
      continue;
    }
    set_priority(dr.armors);
    std::sort(dr.armors.begin(), dr.armors.end(),
      [this](const fyt::auto_aim::Armor &a, const fyt::auto_aim::Armor &b) {
        return getPriority(a.number) < getPriority(b.number);
      });
  }

  detection_queue.erase(
    std::remove_if(detection_queue.begin(), detection_queue.end(),
      [](const DetectionResult &dr) { return dr.armors.empty(); }),
    detection_queue.end());

  std::sort(detection_queue.begin(), detection_queue.end(),
    [this](const DetectionResult &a, const DetectionResult &b) {
      return getPriority(a.armors.front().number) < getPriority(b.armors.front().number);
    });
}

void Decider::get_invincible_armor(const std::vector<int8_t> &invincible_enemy_ids) {
  invincible_armors_.clear();
  if (invincible_enemy_ids.empty()) return;
  for (const auto &id : invincible_enemy_ids) {
    if (id <= 0 || id > 9) continue;
    invincible_armors_.push_back(std::to_string(id));
  }
}

void Decider::get_auto_aim_target(
    std::vector<fyt::auto_aim::Armor> &armors,
    const std::vector<int8_t> &auto_aim_target) const {
  if (auto_aim_target.empty()) return;

  std::vector<std::string> target_strs;
  for (const auto &target : auto_aim_target) {
    if (target <= 0) continue;
    target_strs.push_back(std::to_string(target));
  }
  if (target_strs.empty()) return;

  armors.erase(std::remove_if(armors.begin(), armors.end(),
    [&target_strs](const fyt::auto_aim::Armor &a) {
      return std::find(target_strs.begin(), target_strs.end(), a.number) == target_strs.end();
    }), armors.end());
}

void Decider::setInvincibleArmors(std::vector<std::string> armors) {
  invincible_armors_ = std::move(armors);
}

bool Decider::shouldIgnore(const fyt::auto_aim::Armor &armor) const {
  if (config_.ignore_outpost && armor.number == "outpost") return true;
  if (config_.ignore_base && armor.number == "base") return true;
  return std::find(config_.ignore_numbers.begin(), config_.ignore_numbers.end(),
                   armor.number) != config_.ignore_numbers.end();
}

const fyt::auto_aim::Armor *Decider::pickBest(
    const std::vector<fyt::auto_aim::Armor> &armors,
    const cv::Size &img_size) const {
  if (armors.empty()) return nullptr;

  const cv::Point2f center(
      static_cast<float>(img_size.width) / 2.0F,
      static_cast<float>(img_size.height) / 2.0F);

  auto cmp = [&center](const fyt::auto_aim::Armor &lhs, const fyt::auto_aim::Armor &rhs) {
    return cv::norm(lhs.center - center) < cv::norm(rhs.center - center);
  };
  return &(*std::min_element(armors.begin(), armors.end(), cmp));
}

double Decider::getOffset(const std::vector<double> &offsets, std::size_t camera_index) const {
  if (offsets.empty()) return 0.0;
  if (camera_index >= offsets.size()) return offsets.back();
  return offsets[camera_index];
}

ArmorPriority Decider::getPriority(const std::string &number) const {
  const auto &map = (config_.priority_mode == PriorityMode::MODE_ONE)
                        ? kPriorityMode1 : kPriorityMode2;
  auto it = map.find(number);
  return it != map.end() ? it->second : ArmorPriority::FIFTH;
}

bool Decider::isBackCamera(std::size_t camera_index) const {
  if (config_.camera_count <= 1) return true;
  return camera_index >= static_cast<std::size_t>(config_.camera_count) - 1;
}

double Decider::getFovH(std::size_t camera_index) const {
  return isBackCamera(camera_index) ? config_.fov_h_deg : config_.new_fov_h_deg;
}

double Decider::getFovV(std::size_t camera_index) const {
  return isBackCamera(camera_index) ? config_.fov_v_deg : config_.new_fov_v_deg;
}

}  // namespace rm_omniperception
