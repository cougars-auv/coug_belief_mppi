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

from typing import Any

from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitution import Substitution
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def agent_frame(agent_ns: str | Substitution, frame: str) -> PythonExpression:
    return PythonExpression(["'", agent_ns, f"/{frame}' if '", agent_ns, f"' != '' else '{frame}'"])


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_ns = LaunchConfiguration("agent_ns")

    odom_frame = agent_frame(agent_ns, "odom")
    base_link_frame = agent_frame(agent_ns, "base_link")

    fleet_param_file = RewrittenYaml(
        source_file=PathJoinSubstitution(
            [EnvironmentVariable("CONFIG_DIR"), "fleet", "coug_belief_mppi_params.yaml"]
        ),
        param_rewrites={
            "/**.global_costmap.global_costmap.ros__parameters.global_frame": "map",
            "/**.global_costmap.global_costmap.ros__parameters.robot_base_frame": base_link_frame,
            "/**.local_costmap.local_costmap.ros__parameters.global_frame": odom_frame,
            "/**.local_costmap.local_costmap.ros__parameters.robot_base_frame": base_link_frame,
            "/**.behavior_server.ros__parameters.local_frame": odom_frame,
            "/**.behavior_server.ros__parameters.global_frame": odom_frame,
            "/**.behavior_server.ros__parameters.robot_base_frame": base_link_frame,
            "/**.bt_navigator.ros__parameters.global_frame": "map",
            "/**.bt_navigator.ros__parameters.robot_base_frame": base_link_frame,
        },
    )
    agent_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), [agent_ns, "_params.yaml"]]
    )
    scenario_param_file = (
        LaunchConfiguration("scenario_param_file").perform(context) or agent_param_file
    )

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    return [
        Node(
            package="coug_belief_mppi",
            executable="belief_state_monitor",
            name="belief_state_monitor_node",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
        # --- Navigation2 Pipeline ---
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {"use_sim_time": use_sim_time},
            ],
            remappings=[("/trajectories", "trajectories")],
            additional_env={"OMP_NUM_THREADS": "4"},
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {
                    "use_sim_time": use_sim_time,
                    "autostart": True,
                    "node_names": lifecycle_nodes,
                },
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "agent_ns",
                default_value="auv0",
            ),
            DeclareLaunchArgument(
                "scenario_param_file",
                default_value="",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
