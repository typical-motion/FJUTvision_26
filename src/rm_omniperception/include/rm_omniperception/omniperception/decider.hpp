#ifndef RM_OMNIPERCEPTION__OMNIPERCEPTION__DECIDER_HPP_
#define RM_OMNIPERCEPTION__OMNIPERCEPTION__DECIDER_HPP_

#include <chrono>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core/types.hpp>

#include "armor_detector/types.hpp"
#include "rm_omniperception/omniperception/detection.hpp"

namespace rm_omniperception {

class Decider {
public:
  struct Config {
    std::vector<double> camera_yaw_offsets_deg{62.0, -62.0, 170.0};
    std::vector<double> camera_pitch_offsets_deg{0.0, 0.0, 0.0};
    double fov_h_deg = 54.2;
    double fov_v_deg = 44.5;
    bool ignore_outpost = true;
    bool ignore_base = true;
    std::vector<std::string> ignore_numbers{"negative"};
  };

  explicit Decider(Config config);

  std::optional<DetectionResult> makeDetection(
      const std::vector<fyt::auto_aim::Armor> &armors,
      const cv::Size &img_size,
      std::size_t camera_index,
      std::chrono::steady_clock::time_point timestamp) const;

private:
  bool shouldIgnore(const fyt::auto_aim::Armor &armor) const;
  const fyt::auto_aim::Armor *pickBest(
      const std::vector<fyt::auto_aim::Armor> &armors,
      const cv::Size &img_size) const;
  double getOffset(
      const std::vector<double> &offsets,
      std::size_t camera_index) const;

  Config config_;
};

}  // namespace rm_omniperception

#endif  // RM_OMNIPERCEPTION__OMNIPERCEPTION__DECIDER_HPP_