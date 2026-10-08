// Copyright 2026 BYU FROST Lab
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

#pragma once

#include <Eigen/Core>
#include <atomic>
#include <memory>
#include <mutex>

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace mppi::critics {

class BeliefStateCritic : public CriticFunction {
 public:
  void initialize() override;

  void score(CriticData& data) override;

 private:
  static constexpr int kStateDim = 15;

  // --- Callbacks ---
  void fgOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  void fgVelCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr& msg);

  void fgBiasCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr& msg);

  // --- ROS Interfaces ---
  std::string fg_odom_topic_;
  std::string fg_velocity_topic_;
  std::string fg_bias_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr bias_sub_;

  // --- Parameters ---
  unsigned int power_{0};
  float weight_{0};
  float dvl_update_rate_hz_{0};
  float ahrs_update_rate_hz_{0};
  Eigen::Matrix3d dvl_noise_cov_;
  Eigen::Matrix<double, 1, 1> ahrs_noise_cov_;
  Eigen::Matrix3d gyro_noise_cov_;
  Eigen::Matrix3d accel_noise_cov_;
  Eigen::Matrix3d accel_bias_rw_cov_;
  Eigen::Matrix3d gyro_bias_rw_cov_;
  Eigen::Vector3d gravity_;
  double integration_covariance_{0};

  // --- State ---
  mutable std::mutex state_cov_mutex_;
  Eigen::Matrix<double, kStateDim, kStateDim> init_state_cov_{
      Eigen::Matrix<double, kStateDim, kStateDim>::Identity()};
  std::atomic<bool> received_odom_{false};
  std::atomic<bool> received_vel_{false};
  std::atomic<bool> received_bias_{false};
};

}  // namespace mppi::critics
