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

import yaml
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


def create_rviz_config(agent_list: list[str]) -> str:
    template_path = os.path.join(
        get_package_share_directory("coug_belief_mppi"), "config", "demo.rviz.template"
    )
    with open(template_path) as template:
        config = yaml.safe_load(template)

    displays = config["Visualization Manager"]["Displays"]
    templates = [yaml.safe_dump(display, sort_keys=False) for display in displays]
    shared = [text for text in templates if "<agent_ns>" not in text]
    per_agent = [text for text in templates if "<agent_ns>" in text]

    displays[:] = [yaml.safe_load(text) for text in shared]
    for agent_ns in agent_list:
        group = [yaml.safe_load(text.replace("<agent_ns>", agent_ns)) for text in per_agent]
        if len(agent_list) > 1:
            group = [
                {
                    "Class": "rviz_common/Group",
                    "Name": agent_ns,
                    "Enabled": True,
                    "Displays": group,
                }
            ]
        displays.extend(group)

    with tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".rviz") as rendered_config:
        content = yaml.safe_dump(config, sort_keys=False)
        rendered_config.write(content.replace("<agent_ns>", agent_list[0]))
        return rendered_config.name


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_ns = LaunchConfiguration("agent_ns")
    play_bag_path = LaunchConfiguration("play_bag_path")
    playback_rate = LaunchConfiguration("playback_rate")
    start_offset = LaunchConfiguration("start_offset")

    agent_ns_str = agent_ns.perform(context)
    play_bag_path_str = play_bag_path.perform(context)

    actions: list[Action] = []

    if play_bag_path_str:
        play_process = ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                play_bag_path_str,
                "--clock",
                "--start-offset",
                start_offset,
                "--rate",
                playback_rate,
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
            arguments=["-d", create_rviz_config([agent_ns_str])],
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
                "agent_ns",
                default_value="auv0",
            ),
            DeclareLaunchArgument(
                "play_bag_path",
                default_value="",
            ),
            DeclareLaunchArgument(
                "start_offset",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "playback_rate",
                default_value="1.0",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
