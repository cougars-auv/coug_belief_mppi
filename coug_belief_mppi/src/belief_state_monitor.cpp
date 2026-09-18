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

#include "coug_belief_mppi/belief_state_monitor.hpp"

#include <Eigen/Dense>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "coug_belief_mppi/belief_state_monitor_parameters.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

namespace coug_belief_mppi {

BeliefStateMonitorNode::BeliefStateMonitorNode(const rclcpp::NodeOptions& options)
    : Node("belief_state_monitor_node", options) {
  param_listener_ =
      std::make_shared<belief_state_monitor_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.fg_odom_topic, rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { odomCallback(msg); });

  vel_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      params_.fg_vel_topic, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr& msg) {
        velCallback(msg);
      });

  bias_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      params_.fg_bias_topic, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr& msg) {
        biasCallback(msg);
      });

  trace_pub_ = create_publisher<std_msgs::msg::Float64>(params_.norm_trace_topic,
                                                        rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void BeliefStateMonitorNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  const Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > cov_msg(
      msg->pose.covariance.data());
  Eigen::Matrix<double, 6, 6> pose_cov;
  pose_cov.topLeftCorner<3, 3>() = cov_msg.bottomRightCorner<3, 3>();
  pose_cov.bottomRightCorner<3, 3>() = cov_msg.topLeftCorner<3, 3>();
  pose_cov.topRightCorner<3, 3>() = cov_msg.bottomLeftCorner<3, 3>();
  pose_cov.bottomLeftCorner<3, 3>() = cov_msg.topRightCorner<3, 3>();
  if (!pose_cov.allFinite() || (pose_cov.diagonal().array() <= 0.0).any()) {
    RCLCPP_WARN_ONCE(get_logger(),
                     "Odometry message covariance is unusable (non-finite or non-positive "
                     "diagonal); keeping the last usable one.");
    return;
  }
  state_cov_.block<6, 6>(0, 0) = pose_cov;
  received_odom_ = true;
  publishTrace();
}

void BeliefStateMonitorNode::velCallback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr& msg) {
  const Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > cov_msg(
      msg->twist.covariance.data());
  const Eigen::Matrix3d vel_cov = cov_msg.topLeftCorner<3, 3>();
  if (!vel_cov.allFinite() || (vel_cov.diagonal().array() <= 0.0).any()) {
    RCLCPP_WARN_ONCE(get_logger(),
                     "Velocity message covariance is unusable (non-finite or non-positive "
                     "diagonal); keeping the last usable one.");
    return;
  }
  state_cov_.block<3, 3>(6, 6) = vel_cov;
  received_vel_ = true;
  publishTrace();
}

void BeliefStateMonitorNode::biasCallback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr& msg) {
  const Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > cov_msg(
      msg->twist.covariance.data());
  const Eigen::Matrix<double, 6, 6> bias_cov = cov_msg;
  if (!bias_cov.allFinite() || (bias_cov.diagonal().array() <= 0.0).any()) {
    RCLCPP_WARN_ONCE(get_logger(),
                     "IMU bias message covariance is unusable (non-finite or non-positive "
                     "diagonal); keeping the last usable one.");
    return;
  }
  state_cov_.block<6, 6>(9, 9) = bias_cov;
  received_bias_ = true;
  publishTrace();
}

void BeliefStateMonitorNode::publishTrace() {
  if (!received_odom_ || !received_vel_ || !received_bias_) {
    return;
  }

  if (!init_cov_set_) {
    init_bias_cov_inv_ = state_cov_.block<6, 6>(9, 9).inverse();
    init_cov_set_ = true;
  }

  const double trace = (init_bias_cov_inv_ * state_cov_.block<6, 6>(9, 9)).trace();
  if (trace == last_trace_) {
    return;
  }
  last_trace_ = trace;

  auto msg = std_msgs::msg::Float64();
  msg.data = trace;
  trace_pub_->publish(msg);
}

}  // namespace coug_belief_mppi

RCLCPP_COMPONENTS_REGISTER_NODE(coug_belief_mppi::BeliefStateMonitorNode)
