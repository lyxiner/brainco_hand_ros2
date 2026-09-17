#!/usr/bin/env python3

# Copyright (c) 2026 BrainCo
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

"""Start the minimal ros2_control system for one Revo1 right hand."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    protocol_config_file = LaunchConfiguration("protocol_config_file")

    default_protocol_config = PathJoinSubstitution(
        [
            FindPackageShare("brainco_hand_driver"),
            "config",
            "protocol_modbus_revo1_right.yaml",
        ]
    )
    robot_description_file = PathJoinSubstitution(
        [FindPackageShare("brainco_hand_driver"), "config", "revo1_right.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("brainco_hand_driver"), "config", "revo1_right_controllers.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_file,
            " protocol_config_file:=",
            protocol_config_file,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="right_revo1_hand",
        parameters=[robot_description, controllers_file],
        output="both",
    )
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "right_revo1_hand_position_controller",
            "-c",
            "/right_revo1_hand/controller_manager",
            "-n",
            "/right_revo1_hand",
            "-p",
            controllers_file,
            "--controller-manager-timeout",
            "20",
            "--service-call-timeout",
            "10",
            "--switch-timeout",
            "10",
        ],
        output="both",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "protocol_config_file",
                default_value=default_protocol_config,
                description="Revo1 Modbus hardware configuration YAML file",
            ),
            control_node,
            controller_spawner,
        ]
    )
