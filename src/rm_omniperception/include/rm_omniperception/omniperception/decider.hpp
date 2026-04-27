#ifndef RM_OMNIPERCEPTION__OMNIPERCEPTION__DECIDER_HPP_
#define RM_OMNIPERCEPTION__OMNIPERCEPTION__DECIDER_HPP_

#include <chrono>
#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/core/types.hpp>

#include "armor_detector/types.hpp"
#include "rm_omniperception/omniperception/detection.hpp"

namespace rm_omniperception {

enum class ArmorPriority { FIRST = 1, SECOND, THIRD, FOURTH, FIFTH };

enum class PriorityMode { MODE_ONE = 1, MODE_TWO };

class Decider {
public:
  struct Config {
    int camera_count = 3;
    std::vector<double> camera_yaw_offsets_deg{62.0, -62.0, 170.0};
    std::vector<double> camera_pitch_offsets_deg{0.0, 0.0, 0.0};
    double fov_h_deg = 54.2;
    double fov_v_deg = 44.5;
    double new_fov_h_deg = 54.2;
    double new_fov_v_deg = 44.5;
    bool ignore_outpost = true;
    bool ignore_base = true;
    bool ignore_enemy_two = true;
    std::vector<std::string> ignore_numbers{"negative"};
    PriorityMode priority_mode = PriorityMode::MODE_ONE;
    std::string enemy_color = "red";
  };

  explicit Decider(Config config);

  std::optional<DetectionResult> makeDetection(
      const std::vector<fyt::auto_aim::Armor> &armors,
      const cv::Size &img_size,
      std::size_t camera_index,
      std::chrono::steady_clock::time_point timestamp) const;

  bool armor_filter(std::vector<fyt::auto_aim::Armor> &armors) const;
  void sort(std::vector<DetectionResult> &detection_queue) const;

  void get_invincible_armor(const std::vector<int8_t> &invincible_enemy_ids);
  void get_auto_aim_target(
      std::vector<fyt::auto_aim::Armor> &armors,
      const std::vector<int8_t> &auto_aim_target) const;
  void setInvincibleArmors(std::vector<std::string> armors);

  bool isBackCamera(std::size_t camera_index) const;

private:
  using PriorityMap = std::unordered_map<std::string, ArmorPriority>;

  bool shouldIgnore(const fyt::auto_aim::Armor &armor) const;
  const fyt::auto_aim::Armor *pickBest(
      const std::vector<fyt::auto_aim::Armor> &armors,
      const cv::Size &img_size) const;
  double getOffset(const std::vector<double> &offsets, std::size_t camera_index) const;
  ArmorPriority getPriority(const std::string &number) const;
  double getFovH(std::size_t camera_index) const;
  double getFovV(std::size_t camera_index) const;

  Config config_;
  std::vector<std::string> invincible_armors_;

  const PriorityMap kPriorityMode1 = {
    {"1", ArmorPriority::SECOND}, {"2", ArmorPriority::FOURTH},
    {"3", ArmorPriority::FIRST},  {"4", ArmorPriority::FIRST},
    {"5", ArmorPriority::THIRD},  {"sentry", ArmorPriority::THIRD},
    {"outpost", ArmorPriority::FIFTH}, {"base", ArmorPriority::FIFTH},
    {"negative", ArmorPriority::FIFTH},
  };

  const PriorityMap kPriorityMode2 = {
    {"2", ArmorPriority::FIRST},  {"1", ArmorPriority::SECOND},
    {"3", ArmorPriority::SECOND}, {"4", ArmorPriority::SECOND},
    {"5", ArmorPriority::SECOND}, {"sentry", ArmorPriority::THIRD},
    {"outpost", ArmorPriority::THIRD}, {"base", ArmorPriority::THIRD},
    {"negative", ArmorPriority::THIRD},
  };
};

}  // namespace rm_omniperception

#endif  // RM_OMNIPERCEPTION__OMNIPERCEPTION__DECIDER_HPP_