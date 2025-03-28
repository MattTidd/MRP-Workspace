# Copyright (c) 2024 Matthew Allan Tidd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    robot_state_publisher = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('MRP_Description'),'launch','rsp.launch.py'
                )])
                , launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('MRP_Description'),'launch','joystick.launch.py'
                )])
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(get_package_share_directory('MRP_Description'),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=2.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        # joystick, 
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
    ])