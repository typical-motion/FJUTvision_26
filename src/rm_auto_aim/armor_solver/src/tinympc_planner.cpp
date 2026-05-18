#include "armor_solver/tinympc_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <angles/angles.h>

namespace fyt::auto_aim {

TinyMpcPlanner::TinyMpcPlanner(std::weak_ptr<rclcpp::Node> node) : node_(std::move(node)) {
  auto n = node_.lock();
  if (!n) {
    throw std::runtime_error("Node expired while creating TinyMpcPlanner");
  }

  n->declare_parameter("solver.tinympc.dt", dt_);
  n->declare_parameter("solver.tinympc.yaw_offset", 0.0);
  n->declare_parameter("solver.tinympc.pitch_offset", 0.0);
  n->declare_parameter("solver.tinympc.fire_thresh", fire_thresh_);
  n->declare_parameter("solver.tinympc.shoot_offset", shoot_offset_);
  n->declare_parameter("solver.tinympc.max_yaw_acc", max_yaw_acc_);
  n->declare_parameter("solver.tinympc.max_pitch_acc", max_pitch_acc_);
  n->declare_parameter("solver.tinympc.decision_speed", decision_speed_);
  n->declare_parameter("solver.tinympc.high_speed_delay_time", high_speed_delay_time_);
  n->declare_parameter("solver.tinympc.low_speed_delay_time", low_speed_delay_time_);
  n->declare_parameter("solver.tinympc.Q_yaw", std::vector<double>{40.0, 1.0});
  n->declare_parameter("solver.tinympc.R_yaw", std::vector<double>{40.0});
  n->declare_parameter("solver.tinympc.Q_pitch", std::vector<double>{40.0, 1.0});
  n->declare_parameter("solver.tinympc.R_pitch", std::vector<double>{40.0});

  n.reset();

  refreshParams();
  setupSolvers();
}

TinyMpcPlanner::~TinyMpcPlanner() {
  if (yaw_solver_ != nullptr) {
    delete yaw_solver_->solution;
    delete yaw_solver_->cache;
    delete yaw_solver_->settings;
    delete yaw_solver_->work;
    delete yaw_solver_;
    yaw_solver_ = nullptr;
  }

  if (pitch_solver_ != nullptr) {
    delete pitch_solver_->solution;
    delete pitch_solver_->cache;
    delete pitch_solver_->settings;
    delete pitch_solver_->work;
    delete pitch_solver_;
    pitch_solver_ = nullptr;
  }

  delete yaw_x_warm_;
  delete yaw_u_warm_;
  delete pitch_x_warm_;
  delete pitch_u_warm_;
}

void TinyMpcPlanner::refreshParams() {
  auto n = node_.lock();
  if (!n) {
    throw std::runtime_error("Node expired while refreshing TinyMpcPlanner params");
  }

  dt_ = n->get_parameter("solver.tinympc.dt").as_double();
  yaw_offset_ = n->get_parameter("solver.tinympc.yaw_offset").as_double() * M_PI / 180.0;
  pitch_offset_ = n->get_parameter("solver.tinympc.pitch_offset").as_double() * M_PI / 180.0;
  fire_thresh_ = n->get_parameter("solver.tinympc.fire_thresh").as_double();
  shoot_offset_ = n->get_parameter("solver.tinympc.shoot_offset").as_int();
  max_yaw_acc_ = n->get_parameter("solver.tinympc.max_yaw_acc").as_double();
  max_pitch_acc_ = n->get_parameter("solver.tinympc.max_pitch_acc").as_double();
  decision_speed_ = n->get_parameter("solver.tinympc.decision_speed").as_double();
  high_speed_delay_time_ = n->get_parameter("solver.tinympc.high_speed_delay_time").as_double();
  low_speed_delay_time_ = n->get_parameter("solver.tinympc.low_speed_delay_time").as_double();

  const auto q_yaw = n->get_parameter("solver.tinympc.Q_yaw").as_double_array();
  const auto r_yaw = n->get_parameter("solver.tinympc.R_yaw").as_double_array();
  const auto q_pitch = n->get_parameter("solver.tinympc.Q_pitch").as_double_array();
  const auto r_pitch = n->get_parameter("solver.tinympc.R_pitch").as_double_array();

  // Check if Q/R parameters have changed; if so, update solvers
  bool q_r_changed = false;

  if (q_yaw.size() == 2) {
    Eigen::Vector2d new_q_yaw;
    new_q_yaw << q_yaw[0], q_yaw[1];
    if (!q_yaw_.isApprox(new_q_yaw)) {
      q_yaw_ = new_q_yaw;
      q_r_changed = true;
    }
  }
  if (q_pitch.size() == 2) {
    Eigen::Vector2d new_q_pitch;
    new_q_pitch << q_pitch[0], q_pitch[1];
    if (!q_pitch_.isApprox(new_q_pitch)) {
      q_pitch_ = new_q_pitch;
      q_r_changed = true;
    }
  }
  if (!r_yaw.empty()) {
    Eigen::VectorXd new_r_yaw = 
      Eigen::Map<const Eigen::VectorXd>(r_yaw.data(), static_cast<long>(r_yaw.size()));
    if (r_yaw_.rows() != new_r_yaw.rows() || !r_yaw_.isApprox(new_r_yaw)) {
      r_yaw_ = new_r_yaw;
      q_r_changed = true;
    }
  }
  if (!r_pitch.empty()) {
    Eigen::VectorXd new_r_pitch = 
      Eigen::Map<const Eigen::VectorXd>(r_pitch.data(), static_cast<long>(r_pitch.size()));
    if (r_pitch_.rows() != new_r_pitch.rows() || !r_pitch_.isApprox(new_r_pitch)) {
      r_pitch_ = new_r_pitch;
      q_r_changed = true;
    }
  }

  // Rebuild solvers if Q/R changed to apply new parameters
  if (q_r_changed) {
    setupSolvers();
  }
}

void TinyMpcPlanner::setupSolvers() {
  Eigen::MatrixXd A{{1.0, dt_}, {0.0, 1.0}};
  Eigen::MatrixXd B{{0.0}, {dt_}};
  Eigen::VectorXd f{{0.0, 0.0}};

  // Save warm-start trajectories from previous solvers before rebuild
  const bool has_warm = (yaw_solver_ != nullptr) && (pitch_solver_ != nullptr);
  if (has_warm) {
    yaw_x_warm_ = new tinyMatrix(yaw_solver_->work->x);
    yaw_u_warm_ = new tinyMatrix(yaw_solver_->work->u);
    pitch_x_warm_ = new tinyMatrix(pitch_solver_->work->x);
    pitch_u_warm_ = new tinyMatrix(pitch_solver_->work->u);

    delete yaw_solver_->solution;
    delete yaw_solver_->cache;
    delete yaw_solver_->settings;
    delete yaw_solver_->work;
    delete yaw_solver_;
    yaw_solver_ = nullptr;

    delete pitch_solver_->solution;
    delete pitch_solver_->cache;
    delete pitch_solver_->settings;
    delete pitch_solver_->work;
    delete pitch_solver_;
    pitch_solver_ = nullptr;
  }

  tiny_setup(&yaw_solver_,
             A,
             B,
             f,
             q_yaw_.asDiagonal(),
             r_yaw_.asDiagonal(),
             1.0,
             2,
             1,
             kHorizon,
             0);

  tiny_setup(&pitch_solver_,
             A,
             B,
             f,
             q_pitch_.asDiagonal(),
             r_pitch_.asDiagonal(),
             1.0,
             2,
             1,
             kHorizon,
             0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, kHorizon, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, kHorizon, 1e17);
  Eigen::MatrixXd yaw_u_min = Eigen::MatrixXd::Constant(1, kHorizon - 1, -max_yaw_acc_);
  Eigen::MatrixXd yaw_u_max = Eigen::MatrixXd::Constant(1, kHorizon - 1, max_yaw_acc_);
  Eigen::MatrixXd pitch_u_min = Eigen::MatrixXd::Constant(1, kHorizon - 1, -max_pitch_acc_);
  Eigen::MatrixXd pitch_u_max = Eigen::MatrixXd::Constant(1, kHorizon - 1, max_pitch_acc_);

  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, yaw_u_min, yaw_u_max);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, pitch_u_min, pitch_u_max);

  yaw_solver_->settings->max_iter = 25;
  pitch_solver_->settings->max_iter = 25;

  // Restore warm-start trajectories to avoid cold-start jumps after Q/R change
  if (has_warm) {
    yaw_solver_->work->x = *yaw_x_warm_;
    yaw_solver_->work->u = *yaw_u_warm_;
    pitch_solver_->work->x = *pitch_x_warm_;
    pitch_solver_->work->u = *pitch_u_warm_;

    delete yaw_x_warm_;
    delete yaw_u_warm_;
    delete pitch_x_warm_;
    delete pitch_u_warm_;
    yaw_x_warm_ = nullptr;
    yaw_u_warm_ = nullptr;
    pitch_x_warm_ = nullptr;
    pitch_u_warm_ = nullptr;
  }
}

double TinyMpcPlanner::normalizeAngle(double angle) {
  return angles::normalize_angle(angle);
}

std::vector<Eigen::Vector3d> TinyMpcPlanner::getArmorPositions(const Eigen::Vector3d &target_center,
                                                               double target_yaw,
                                                               double r1,
                                                               double r2,
                                                               double d_zc,
                                                               double d_za,
                                                               size_t armors_num) const {
  auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());
  bool is_current_pair = true;
  double r = 0.0;
  double target_dz = 0.0;
  for (size_t i = 0; i < armors_num; i++) {
    const double temp_yaw = target_yaw + static_cast<double>(i) * (2.0 * M_PI / armors_num);
    if (armors_num == 4) {
      r = is_current_pair ? r1 : r2;
      target_dz = d_zc + (is_current_pair ? 0.0 : d_za);
      is_current_pair = !is_current_pair;
    } else {
      r = r1;
      target_dz = d_zc;
    }
    armor_positions[i] =
      target_center + Eigen::Vector3d(-r * std::cos(temp_yaw), -r * std::sin(temp_yaw), target_dz);
  }
  return armor_positions;
}

Eigen::Vector3d TinyMpcPlanner::getClosestArmorPosition(const Eigen::Vector3d &target_center,
                                                        double target_yaw,
                                                        double r1,
                                                        double r2,
                                                        double d_zc,
                                                        double d_za,
                                                        size_t armors_num) const {
  const auto armors = getArmorPositions(target_center, target_yaw, r1, r2, d_zc, d_za, armors_num);
  if (armors.empty()) {
    return Eigen::Vector3d::Zero();
  }

  double min_dist = std::numeric_limits<double>::max();
  Eigen::Vector3d picked = armors.front();
  for (const auto &armor : armors) {
    const double dist = armor.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      picked = armor;
    }
  }
  return picked;
}

TinyMpcPlanner::AimPoint TinyMpcPlanner::computeAimForArmor(
    const Eigen::Vector3d &armor_position,
    TrajectoryCompensator *trajectory_compensator,
    double bullet_speed) const {
  double yaw = std::atan2(armor_position.y(), armor_position.x()) + yaw_offset_;
  double pitch = std::atan2(armor_position.z(), armor_position.head<2>().norm());

  if (trajectory_compensator != nullptr) {
    trajectory_compensator->velocity = bullet_speed;
    double compensated_pitch = pitch;
    if (!trajectory_compensator->compensate(armor_position, compensated_pitch)) {
      throw std::runtime_error("Unsolvable bullet trajectory");
    }
    pitch = compensated_pitch;
  }

  pitch -= pitch_offset_;

  return {normalizeAngle(yaw), pitch, armor_position.x(), armor_position.y(), armor_position.z()};
}

TinyMpcPlanner::AimPoint TinyMpcPlanner::computeAim(const Eigen::Vector3d &target_center,
                                                    double target_yaw,
                                                    double r1,
                                                    double r2,
                                                    double d_zc,
                                                    double d_za,
                                                    size_t armors_num,
                                                    TrajectoryCompensator *trajectory_compensator,
                                                    double bullet_speed) const {
  const Eigen::Vector3d picked =
    getClosestArmorPosition(target_center, target_yaw, r1, r2, d_zc, d_za, armors_num);
  return computeAimForArmor(picked, trajectory_compensator, bullet_speed);
}

TinyMpcPlanner::Trajectory TinyMpcPlanner::buildTrajectory(const rm_interfaces::msg::Target &target,
                                                           const rclcpp::Time &current_time,
                                                           TrajectoryCompensator *trajectory_compensator,
                                                           double bullet_speed,
                                                           double prediction_delay,
                                                           double &yaw0,
                                                           double &target_yaw,
                                                           double &target_pitch,
                                                           double &aim_distance,
                                                           double &aim_height) const {
  std::array<AimPoint, kHorizon> aim_points{};

  const Eigen::Vector3d center0(target.position.x, target.position.y, target.position.z);
  const Eigen::Vector3d velocity(target.velocity.x, target.velocity.y, target.velocity.z);

  const double dt_to_now = (current_time - rclcpp::Time(target.header.stamp)).seconds();

  // Decision delay: extra look-ahead to compensate for comm/mechanical latency.
  // This is separate from flying_time (bullet travel time) which is added below.
  const double decision_delay = std::abs(target.v_yaw) > decision_speed_ ? high_speed_delay_time_ :
                                                                   low_speed_delay_time_;

  // Compute flying_time at the nominal center of the MPC horizon (without yet
  // including flying_time itself — first-order approximation).
  // Flying time is the physical bullet travel time and must be added so that
  // the reference trajectory aims at where the target WILL be when the bullet
  // arrives, not where it IS when the bullet is fired.
  const double nominal_center_t = dt_to_now + prediction_delay + decision_delay;
  const Eigen::Vector3d nominal_center_pos = center0 + velocity * nominal_center_t;
  const double flying_time = trajectory_compensator->getFlyingTime(nominal_center_pos);

  const double base_t = nominal_center_t + flying_time;

  // Lock armor index at horizon center to keep reference trajectory self-consistent.
  // Without this, each step independently calls getClosestArmorPosition(), which can
  // flip between armor plates near the equidistant boundary, injecting 30°-50° spikes
  // into the MPC reference and causing the solver to produce sharp yaw transients.
  const double center_t = base_t;
  const Eigen::Vector3d center_pos = center0 + velocity * center_t;
  const double center_yaw = target.yaw + target.v_yaw * center_t;
  const auto center_armors = getArmorPositions(center_pos,
                                               center_yaw,
                                               target.radius_1,
                                               target.radius_2,
                                               target.d_zc,
                                               target.d_za,
                                               static_cast<size_t>(target.armors_num));
  size_t locked_idx = 0;
  {
    double min_dist = std::numeric_limits<double>::max();
    for (size_t j = 0; j < center_armors.size(); ++j) {
      const double dist = center_armors[j].head<2>().norm();
      if (dist < min_dist) {
        min_dist = dist;
        locked_idx = j;
      }
    }
  }

  for (int i = 0; i < kHorizon; i++) {
    const double local_t = (static_cast<double>(i) - kHalfHorizon) * dt_;
    const double t = base_t + local_t;

    const Eigen::Vector3d center = center0 + velocity * t;
    const double yaw = target.yaw + target.v_yaw * t;

    // Use the same locked armor index for every step to avoid intra-trajectory switching
    const auto armors = getArmorPositions(center,
                                          yaw,
                                          target.radius_1,
                                          target.radius_2,
                                          target.d_zc,
                                          target.d_za,
                                          static_cast<size_t>(target.armors_num));
    aim_points[i] = computeAimForArmor(armors[locked_idx], trajectory_compensator, bullet_speed);
  }

  yaw0 = aim_points[kHalfHorizon].yaw;
  target_yaw = aim_points[kHalfHorizon].yaw;
  target_pitch = aim_points[kHalfHorizon].pitch;
  aim_distance = std::hypot(aim_points[kHalfHorizon].x, aim_points[kHalfHorizon].y);
  aim_height = aim_points[kHalfHorizon].z;

  Trajectory traj = Trajectory::Zero();

  for (int i = 0; i < kHorizon; i++) {
    const int prev = std::max(i - 1, 0);
    const int next = std::min(i + 1, kHorizon - 1);

    const double yaw_vel =
      normalizeAngle(aim_points[next].yaw - aim_points[prev].yaw) / (2.0 * dt_);
    const double pitch_vel = (aim_points[next].pitch - aim_points[prev].pitch) / (2.0 * dt_);

    traj(0, i) = normalizeAngle(aim_points[i].yaw - yaw0);
    traj(1, i) = yaw_vel;
    traj(2, i) = aim_points[i].pitch;
    traj(3, i) = pitch_vel;
  }

  return traj;
}

TinyMpcPlanner::PlanResult TinyMpcPlanner::plan(const rm_interfaces::msg::Target &target,
                                                const rclcpp::Time &current_time,
                                                TrajectoryCompensator *trajectory_compensator,
                                                double bullet_speed,
                                                double prediction_delay) {
  PlanResult result;

  if (target.armors_num <= 0) {
    return result;
  }

  refreshParams();

  if (bullet_speed < 10.0 || bullet_speed > 30.0) {
    bullet_speed = 22.0;
  }

  double yaw0 = 0.0;
  double target_yaw = 0.0;
  double target_pitch = 0.0;
  double aim_distance = 0.0;
  double aim_height = 0.0;

  Trajectory traj;
  try {
    traj = buildTrajectory(target,
                           current_time,
                           trajectory_compensator,
                           bullet_speed,
                           prediction_delay,
                           yaw0,
                           target_yaw,
                           target_pitch,
                           aim_distance,
                           aim_height);
  } catch (const std::exception &) {
    return result;
  }

  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);
  yaw_solver_->work->Xref = traj.block(0, 0, 2, kHorizon);
  tiny_solve(yaw_solver_);

  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);
  pitch_solver_->work->Xref = traj.block(2, 0, 2, kHorizon);
  tiny_solve(pitch_solver_);

  const int idx = kHalfHorizon;
  const int fire_idx = std::clamp(idx + shoot_offset_, 0, kHorizon - 1);

  double raw_yaw = normalizeAngle(yaw_solver_->work->x(0, idx) + yaw0);
  double raw_pitch = pitch_solver_->work->x(0, idx);

  // Exponential moving average smoothing to suppress residual jumps
  if (!smoothing_initialized_) {
    smoothed_yaw_ = raw_yaw;
    smoothed_pitch_ = raw_pitch;
    smoothing_initialized_ = true;
  } else {
    // Handle angle wrap for yaw smoothing
    double diff = normalizeAngle(raw_yaw - smoothed_yaw_);
    smoothed_yaw_ = normalizeAngle(smoothed_yaw_ + kSmoothingAlpha * diff);
    smoothed_pitch_ = smoothed_pitch_ + kSmoothingAlpha * (raw_pitch - smoothed_pitch_);
  }

  result.valid = true;
  result.target_yaw = target_yaw;
  result.target_pitch = target_pitch;
  result.aim_distance = aim_distance;
  result.aim_height = aim_height;
  result.plan_yaw = smoothed_yaw_;
  result.plan_pitch = smoothed_pitch_;

  const double yaw_err = normalizeAngle(traj(0, fire_idx) - yaw_solver_->work->x(0, fire_idx));
  const double pitch_err = traj(2, fire_idx) - pitch_solver_->work->x(0, fire_idx);
  result.fire = std::hypot(yaw_err, pitch_err) < fire_thresh_;

  return result;
}

}  // namespace fyt::auto_aim
