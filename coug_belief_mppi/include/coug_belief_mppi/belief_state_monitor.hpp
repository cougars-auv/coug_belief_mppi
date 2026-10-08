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
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "coug_belief_mppi/belief_state_monitor_parameters.hpp"

namespace coug_belief_mppi {

class BeliefStateMonitorNode : public rclcpp::Node {
 public:
  explicit BeliefStateMonitorNode(const rclcpp::NodeOptions& options);

 private:
  static constexpr int kStateDim = 15;

  // --- Callbacks ---
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  void velCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr& msg);

  void biasCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr& msg);

  // --- Helpers ---
  void publishTrace();

  // --- ROS Interfaces ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr bias_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr trace_pub_;

  // --- Parameters ---
  std::shared_ptr<belief_state_monitor_node::ParamListener> param_listener_;
  belief_state_monitor_node::Params params_;

  // --- State ---
  Eigen::Matrix<double, kStateDim, kStateDim> state_cov_{
      Eigen::Matrix<double, kStateDim, kStateDim>::Identity()};
  Eigen::Matrix<double, 6, 6> init_bias_cov_inv_{Eigen::Matrix<double, 6, 6>::Identity()};
  bool received_odom_{false};
  bool received_vel_{false};
  bool received_bias_{false};
  bool init_cov_set_{false};
  double last_trace_{-1.0};
};

}  // namespace coug_belief_mppi
