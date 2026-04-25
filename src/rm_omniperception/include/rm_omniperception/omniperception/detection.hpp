#ifndef RM_OMNIPERCEPTION__OMNIPERCEPTION__DETECTION_HPP_
#define RM_OMNIPERCEPTION__OMNIPERCEPTION__DETECTION_HPP_

#include <chrono>
#include <cstddef>
#include <vector>

#include "armor_detector/types.hpp"

namespace rm_omniperception {

struct DetectionResult {
  std::vector<fyt::auto_aim::Armor> armors;
  std::chrono::steady_clock::time_point timestamp;
  double delta_yaw = 0.0;
  double delta_pitch = 0.0;
  std::size_t camera_index = 0;
};

}  // namespace rm_omniperception

#endif  // RM_OMNIPERCEPTION__OMNIPERCEPTION__DETECTION_HPP_