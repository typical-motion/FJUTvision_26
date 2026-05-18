// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "armor_solver/armor_tracker.hpp"
// std
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
// ros2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// third party
#include <angles/angles.h>
// project
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {
Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
: tracker_state(LOST)
, jumped(false)
, isinit(false)
, position_diff(DBL_MAX)
, yaw_diff(DBL_MAX)
, tracked_id(std::string(""))
, measurement(Eigen::VectorXd::Zero(4))
, target_state(Eigen::VectorXd::Zero(9))
, outpost_step_height_(0.1)
, max_match_distance_(max_match_distance)
, max_match_yaw_diff_(max_match_yaw_diff)
, detect_count_(0)
, lost_count_(0)
, update_count_(0)
, switch_count_(0)
, is_switch_(false)
, is_converged_(false)
, last_yaw_(0) {}

void Tracker::init(const Armors::SharedPtr &armors_msg) noexcept {
  if (armors_msg->armors.empty()) {
    return;
  }

  // Simply choose the armor that is closest to image center
  double min_distance = DBL_MAX;
  tracked_armor = armors_msg->armors[0];
  for (const auto &armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  initEKF(tracked_armor);
  FYT_INFO("armor_solver", "Init EKF!");

  tracked_id = tracked_armor.number;
  tracker_state = DETECTING;
  jumped = false;
  position_diff = DBL_MAX;
  yaw_diff = DBL_MAX;
  update_count_ = 0;
  switch_count_ = 0;
  is_switch_ = false;
  is_converged_ = false;

  if (tracked_armor.type == "large" &&
      (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
    tracked_armors_num = ArmorsNum::BALANCE_2;
  } else if (tracked_id == "outpost") {
    tracked_armors_num = ArmorsNum::OUTPOST_3;
  } else {
    tracked_armors_num = ArmorsNum::NORMAL_4;
  }
}

void Tracker::update(const Armors::SharedPtr &armors_msg) noexcept {
  // KF predict
  Eigen::VectorXd ekf_prediction = ekf->predict();

  bool matched = false;
  // Use KF prediction as default target state if no matched armor is found
  target_state = ekf_prediction;
  position_diff = DBL_MAX;
  yaw_diff = DBL_MAX;

  if (!armors_msg->armors.empty()) {
    // Find the closest armor with the same id
    Armor same_id_armor;
    int same_id_armors_count = 0;
    bool is_outpost = (tracked_id == "outpost");

    if (is_outpost) {
      // ===== OUTPOST SPIRAL PATH =====
      // EKF always tracks plate 0 (lowest plate). d_zc = plate-0 z offset.
      // matchOutpostPlateIndex identifies which plate (0-2) was observed.
      // Plate-k measurements are converted to plate-0 equivalents so the
      // EKF Measure model (which expects plate-0 data) receives consistent input.
      int outpost_plate_idx = 0;
      double min_position_diff = DBL_MAX;
      double min_yaw_diff = DBL_MAX;
      for (const auto &armor : armors_msg->armors) {
        if (armor.number == tracked_id) {
          same_id_armor = armor;
          same_id_armors_count++;
          auto p = armor.pose.position;
          Eigen::Vector3d position_vec(p.x, p.y, p.z);
          int best_plate = matchOutpostPlateIndex(ekf_prediction, position_vec, outpost_step_height_);
          auto best_pred = getArmorPositionFromState(ekf_prediction, best_plate, outpost_step_height_);
          double position_diff = (best_pred - position_vec).norm();
          double measured_yaw = orientationToYaw(armor.pose.orientation);
          double plate0_yaw = angles::normalize_angle(measured_yaw - best_plate * (2.0 * M_PI / 3.0));
          double this_yaw_diff = std::abs(
            angles::shortest_angular_distance(ekf_prediction(6), plate0_yaw));
          if (position_diff < min_position_diff) {
            min_position_diff = position_diff;
            min_yaw_diff = this_yaw_diff;
            outpost_plate_idx = best_plate;
            tracked_armor = armor;
            tracked_armors_num = ArmorsNum::OUTPOST_3;
          }
        }
      }

      if (same_id_armors_count > 0) {
        position_diff = min_position_diff;
        yaw_diff = min_yaw_diff;
      }

      if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
        matched = true;
        auto p = tracked_armor.pose.position;
        double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
        double measured_x = p.x;
        double measured_y = p.y;
        double measured_z = p.z;
        // Convert plate-k measurement to plate-0 equivalents for EKF
        if (outpost_plate_idx != 0) {
          const double k_angle = outpost_plate_idx * (2.0 * M_PI / 3.0);
          measured_z -= outpost_plate_idx * outpost_step_height_;
          measured_yaw = angles::normalize_angle(measured_yaw - k_angle);
          // Back-project through center: xc = xa_k + r*cos(yaw_k)
          double r = ekf_prediction(8);
          double yaw_k = measured_yaw + k_angle; // original observed yaw
          double xc_est = measured_x + r * std::cos(yaw_k);
          double yc_est = measured_y + r * std::sin(yaw_k);
          measured_x = xc_est - r * std::cos(measured_yaw);
          measured_y = yc_est - r * std::sin(measured_yaw);
        }
        measurement = Eigen::Vector4d(measured_x, measured_y, measured_z, measured_yaw);
        target_state = ekf->update(measurement);
      } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
        handleArmorJump(same_id_armor);
      } else {
        FYT_WARN("armor_solver", "No matched armor found!");
      }

    } else {
      // ===== ORIGINAL PATH for non-outpost (unchanged) =====
      auto predicted_position = getArmorPositionFromState(ekf_prediction);
      double min_position_diff = DBL_MAX;
      double min_yaw_diff = DBL_MAX;
      for (const auto &armor : armors_msg->armors) {
        // Only consider armors with the same id
        if (armor.number == tracked_id) {
          same_id_armor = armor;
          same_id_armors_count++;
          // Calculate the difference between the predicted position and the
          // current armor position
          auto p = armor.pose.position;
          Eigen::Vector3d position_vec(p.x, p.y, p.z);
          double position_diff = (predicted_position - position_vec).norm();
          if (position_diff < min_position_diff) {
            // Find the closest armor
            min_position_diff = position_diff;
            min_yaw_diff = std::abs(
              angles::shortest_angular_distance(ekf_prediction(6), orientationToYaw(armor.pose.orientation)));
            tracked_armor = armor;
            // Update tracked armor type
            if (tracked_armor.type == "large" &&
                (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
              tracked_armors_num = ArmorsNum::BALANCE_2;
            } else {
              tracked_armors_num = ArmorsNum::NORMAL_4;
            }
          }
        }
      }

      if (same_id_armors_count > 0) {
        position_diff = min_position_diff;
        yaw_diff = min_yaw_diff;
      }

      // Check if the distance and yaw difference of closest armor are within the
      // threshold
      if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
        // Matched armor found
        matched = true;
        auto p = tracked_armor.pose.position;
        // Update EKF
        double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
        measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
        target_state = ekf->update(measurement);
      } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
        // Matched armor not found, but there is only one armor with the same id
        // and yaw has jumped, take this case as the target is spinning and armor
        // jumped
        handleArmorJump(same_id_armor);
      } else {
        // No matched armor found
        FYT_WARN("armor_solver", "No matched armor found!");
      }
    }
  }

  // Prevent radius from spreading
  // Outpost has fixed radius (distance from center to armor plate)
  // and bounded angular velocity (max 0.8π rad/s)
  if (tracked_id == "outpost") {
    target_state(8) = 0.275;
    target_state(7) = std::clamp(target_state(7), -0.8 * M_PI, 0.8 * M_PI);
    ekf->setState(target_state);
  } else if (target_state(8) < 0.05) {
    target_state(8) = 0.05;
    ekf->setState(target_state);
  } else if (target_state(8) > 0.5) {
    target_state(8) = 0.5;
    ekf->setState(target_state);
  }

  update_count_++;
  if (!is_converged_) {
    is_converged_ = convergened();
  }

  // Tracking state machine
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
        FYT_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
      FYT_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
      FYT_DEBUG("armor_solver", "Tracker state: TEMP_LOST {}", tracked_id);
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_thres) {
        lost_count_ = 0;
        tracker_state = LOST;
        FYT_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
      FYT_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
    }
  }
}

void Tracker::initEKF(const Armor &a) noexcept {
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  last_yaw_ = 0;
  double yaw = orientationToYaw(a.pose.orientation);

  // Set initial position at 0.2m behind the target
  target_state = Eigen::VectorXd::Zero(X_N);
  double r = 0.26;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  double zc = za;
  d_za = 0, d_zc = 0, another_r = r;
  target_state << xc, 0, yc, 0, zc, 0, yaw, 0, r, d_zc;

  ekf->setState(target_state);
  isinit = true;
}

void Tracker::handleArmorJump(const Armor &current_armor) noexcept {
  jumped = true;
  double last_yaw = target_state(6);
  double yaw = orientationToYaw(current_armor.pose.orientation);

  if (std::abs(yaw - last_yaw) > 0.4) {
    // Armor angle also jumped, take this case as target spinning
    target_state(6) = yaw;
    // Only 4 armors has 2 radius and height
    if (tracked_armors_num == ArmorsNum::NORMAL_4) {
      d_za = target_state(4) + target_state(9) - current_armor.pose.position.z;
      std::swap(target_state(8), another_r);
      d_zc = d_zc == 0 ? -d_za : 0;
      target_state(9) = d_zc;
    }
    // Outpost spiral: plate changed. Convert observed plate-k to plate-0 reference.
    if (tracked_armors_num == ArmorsNum::OUTPOST_3) {
      auto p = current_armor.pose.position;
      int plate_idx = matchOutpostPlateIndex(target_state, Eigen::Vector3d(p.x, p.y, p.z), outpost_step_height_);
      if (plate_idx != 0) {
        const double k_angle = plate_idx * (2.0 * M_PI / 3.0);
        target_state(6) = angles::normalize_angle(yaw - k_angle);          // plate-0 yaw
        target_state(4) = p.z - plate_idx * outpost_step_height_;          // plate-0 zc
      }
      d_zc = 0;
      target_state(9) = d_zc;
      FYT_DEBUG("armor_solver", "Outpost plate jump to idx {}", plate_idx);
    }
    FYT_DEBUG("armor_solver", "Armor Jump!");
  }

  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);  // plate 0

  if ((current_p - infer_p).norm() > max_match_distance_) {
    // If the distance between the current armor and the inferred armor is too
    // large, the state is wrong, reset center position and velocity in the
    // state
    d_zc = 0;
    double r = target_state(8);
    target_state(0) = p.x + r * cos(yaw);  // xc
    target_state(1) = 0;                   // vxc
    target_state(2) = p.y + r * sin(yaw);  // yc
    target_state(3) = 0;                   // vyc
    target_state(4) = p.z;                 // zc
    target_state(5) = 0;                   // vzc
    target_state(9) = d_zc;                // d_zc
    FYT_WARN("armor_solver", "State wrong!");
  }

  ekf->setState(target_state);
}

bool Tracker::diverged() const noexcept {
  auto r_ok = target_state(8) > 0.05 && target_state(8) < 0.5;
  auto l_ok = another_r > 0.05 && another_r < 0.5;

  if (tracked_armors_num == ArmorsNum::NORMAL_4) {
    return !(r_ok && l_ok);
  }

  return !r_ok;
}

bool Tracker::convergened() {
  if (tracked_id != "outpost" && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  if (tracked_id == "outpost" && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

bool Tracker::checkinit() const noexcept { return isinit; }

double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion &q) noexcept {
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous (-pi~pi to -inf~inf)
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd &x,
                                                    int plate_idx,
                                                    double step_height) noexcept {
  // Calculate predicted position of the specified armor plate.
  // plate_idx=0 (default): the reference plate at angle yaw, height zc+d_zc.
  // For outpost spiral, plate_idx>0 shifts angle by 120° and z by step_height.
  double xc = x(0), yc = x(2);
  double yaw = x(6), r = x(8);
  double za = x(4) + x(9) + plate_idx * step_height;
  if (plate_idx != 0) {
    double angular_offset = plate_idx * (2.0 * M_PI / 3.0);
    double xa = xc - r * cos(yaw + angular_offset);
    double ya = yc - r * sin(yaw + angular_offset);
    return Eigen::Vector3d(xa, ya, za);
  }
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, za);
}

int Tracker::matchOutpostPlateIndex(const Eigen::VectorXd &state,
                                    const Eigen::Vector3d &measured_pos,
                                    double step_height) noexcept {
  int best_idx = 0;
  double best_diff = std::numeric_limits<double>::max();
  for (int i = 0; i < 3; i++) {
    Eigen::Vector3d predicted = getArmorPositionFromState(state, i, step_height);
    double diff = (predicted - measured_pos).norm();
    if (diff < best_diff) {
      best_diff = diff;
      best_idx = i;
    }
  }
  return best_idx;
}

}  // namespace fyt::auto_aim
