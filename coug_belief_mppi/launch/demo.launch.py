# Copyright 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_rviz_config(agent_ns: str) -> str:
    template_path = os.path.join(
        get_package_share_directory("coug_belief_mppi"), "config", "demo.rviz.template"
    )
    with open(template_path) as template:
        content = template.read().replace("AGENT_NS", agent_ns)

    with tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".rviz") as rendered_config:
        rendered_config.write(content)
        return rendered_config.name


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_delay = LaunchConfiguration("start_delay")
    agent_ns = LaunchConfiguration("agent_ns")
    play_bag_path = LaunchConfiguration("play_bag_path")
    playback_rate = LaunchConfiguration("playback_rate")

    play_bag_path_str = play_bag_path.perform(context)
    agent_ns_str = agent_ns.perform(context)

    actions = []

    if play_bag_path_str:
        play_process = ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                play_bag_path_str,
                "--clock",
                "--rate",
                playback_rate,
                "--start-offset",
                start_delay,
                "--topics",
                "/tf",
                "/tf_static",
                f"/{agent_ns_str}/robot_description",
                f"/{agent_ns_str}/odometry/truth",
                f"/{agent_ns_str}/odometry/global",
                f"/{agent_ns_str}/smoothed_path",
                f"/{agent_ns_str}/waypoints_nav2",
                f"/{agent_ns_str}/belief_state_monitor_node/norm_trace",
                f"/{agent_ns_str}/factor_graph_node/velocity",
                f"/{agent_ns_str}/factor_graph_node/imu/bias",
            ],
        )
        actions.append(play_process)

        actions.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=play_process,
                    on_exit=[LogInfo(msg="Bag playback finished.")],
                )
            )
        )

    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", create_rviz_config(agent_ns_str)],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "start_delay",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "agent_ns",
                default_value="auv0",
            ),
            DeclareLaunchArgument(
                "play_bag_path",
                default_value="",
            ),
            DeclareLaunchArgument(
                "playback_rate",
                default_value="1.0",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
