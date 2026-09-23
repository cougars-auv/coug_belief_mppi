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

#include "coug_belief_mppi/waypoint_nav2.hpp"

#include <chrono>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "coug_belief_mppi/waypoint_nav2_parameters.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace coug_belief_mppi {

WaypointNav2Node::WaypointNav2Node(const rclcpp::NodeOptions& options)
    : Node("waypoint_nav2_node", options) {
  param_listener_ =
      std::make_shared<waypoint_nav2_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  waypoint_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      params_.waypoint_topic, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::PoseArray::ConstSharedPtr& msg) { waypointCallback(msg); });

  nav2_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void WaypointNav2Node::waypointCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr& msg) {
  if (msg->poses.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty waypoint list; canceling navigation.");
    nav2_client_->async_cancel_all_goals();
    return;
  }

  static constexpr std::chrono::seconds kActionServerTimeout{5};
  if (!nav2_client_->wait_for_action_server(kActionServerTimeout)) {
    RCLCPP_ERROR(get_logger(), "Nav2 FollowWaypoints action server not available after %ld s.",
                 static_cast<long>(kActionServerTimeout.count()));
    return;
  }

  auto goal_msg = FollowWaypoints::Goal();

  for (const auto& pose : msg->poses) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = msg->header.frame_id;
    pose_stamped.header.stamp = get_clock()->now();
    pose_stamped.pose = pose;

    goal_msg.poses.push_back(pose_stamped);
  }

  auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
  send_goal_options.result_callback =
      [this](const GoalHandleFollowWaypoints::WrappedResult& result) { resultCallback(result); };

  nav2_client_->async_send_goal(goal_msg, send_goal_options);

  RCLCPP_INFO(get_logger(), "Sent %zu waypoint(s) to Nav2 in '%s'.", goal_msg.poses.size(),
              msg->header.frame_id.c_str());
}

void WaypointNav2Node::resultCallback(const GoalHandleFollowWaypoints::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Nav2 completed the waypoint sequence.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Nav2 aborted the waypoint sequence.");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(get_logger(), "Nav2 canceled the waypoint sequence.");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Nav2 returned unknown result code %d.",
                   static_cast<int>(result.code));
      break;
  }
}

}  // namespace coug_belief_mppi

RCLCPP_COMPONENTS_REGISTER_NODE(coug_belief_mppi::WaypointNav2Node)
