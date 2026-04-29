#ifndef ARMOR_SOLVER_TINYMPC_PLANNER_HPP_
#define ARMOR_SOLVER_TINYMPC_PLANNER_HPP_

#include <array>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"
#include "tinympc/tiny_api.hpp"

namespace fyt::auto_aim {

class TinyMpcPlanner {
public:
  struct PlanResult {
    bool valid{false};
    bool fire{false};
    double plan_yaw{0.0};
    double plan_pitch{0.0};
    double target_yaw{0.0};
    double target_pitch{0.0};
    double aim_distance{0.0};
    double aim_height{0.0};
  };

  explicit TinyMpcPlanner(std::weak_ptr<rclcpp::Node> node);
  ~TinyMpcPlanner();

  PlanResult plan(const rm_interfaces::msg::Target &target,
                  const rclcpp::Time &current_time,
                  TrajectoryCompensator *trajectory_compensator,
                  double bullet_speed,
                  double prediction_delay);

private:
  static constexpr int kHalfHorizon = 50;
  static constexpr int kHorizon = kHalfHorizon * 2;

  using Trajectory = Eigen::Matrix<double, 4, kHorizon>;

  struct AimPoint {
    double yaw{0.0};
    double pitch{0.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  void setupSolvers();
  void refreshParams();

  static double normalizeAngle(double angle);

  std::vector<Eigen::Vector3d> getArmorPositions(const Eigen::Vector3d &target_center,
                                                 double target_yaw,
                                                 double r1,
                                                 double r2,
                                                 double d_zc,
                                                 double d_za,
                                                 size_t armors_num) const;

  Eigen::Vector3d getClosestArmorPosition(const Eigen::Vector3d &target_center,
                                          double target_yaw,
                                          double r1,
                                          double r2,
                                          double d_zc,
                                          double d_za,
                                          size_t armors_num) const;

  AimPoint computeAim(const Eigen::Vector3d &target_center,
                      double target_yaw,
                      double r1,
                      double r2,
                      double d_zc,
                      double d_za,
                      size_t armors_num,
                      TrajectoryCompensator *trajectory_compensator,
                      double bullet_speed) const;

  Trajectory buildTrajectory(const rm_interfaces::msg::Target &target,
                             const rclcpp::Time &current_time,
                             TrajectoryCompensator *trajectory_compensator,
                             double bullet_speed,
                             double prediction_delay,
                             double &yaw0,
                             double &target_yaw,
                             double &target_pitch,
                             double &aim_distance,
                             double &aim_height) const;

  std::weak_ptr<rclcpp::Node> node_;

  TinySolver *yaw_solver_{nullptr};
  TinySolver *pitch_solver_{nullptr};

  double dt_{0.01};
  double yaw_offset_{0.0};
  double pitch_offset_{0.0};
  double fire_thresh_{0.05};
  int shoot_offset_{2};

  double decision_speed_{8.0};
  double high_speed_delay_time_{0.12};
  double low_speed_delay_time_{0.05};

  double max_yaw_acc_{6.0};
  double max_pitch_acc_{6.0};
  Eigen::Vector2d q_yaw_{40.0, 1.0};
  Eigen::VectorXd r_yaw_{Eigen::VectorXd::Constant(1, 40.0)};
  Eigen::Vector2d q_pitch_{40.0, 1.0};
  Eigen::VectorXd r_pitch_{Eigen::VectorXd::Constant(1, 40.0)};

  // Warm-start buffers for solver rebuild (preserved across Q/R changes)
  tinyMatrix *yaw_x_warm_{nullptr};
  tinyMatrix *yaw_u_warm_{nullptr};
  tinyMatrix *pitch_x_warm_{nullptr};
  tinyMatrix *pitch_u_warm_{nullptr};

  // Output exponential smoothing state
  bool smoothing_initialized_{false};
  double smoothed_yaw_{0.0};
  double smoothed_pitch_{0.0};
  static constexpr double kSmoothingAlpha = 0.3;  // Lower = more smoothing

  // Keep previous Q/R values to detect changes
  Eigen::Vector2d q_yaw_prev_{40.0, 1.0};
  Eigen::VectorXd r_yaw_prev_{Eigen::VectorXd::Constant(1, 40.0)};
  Eigen::Vector2d q_pitch_prev_{40.0, 1.0};
  Eigen::VectorXd r_pitch_prev_{Eigen::VectorXd::Constant(1, 40.0)};
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_TINYMPC_PLANNER_HPP_
