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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # prepare local URDF:
    pkg_path = os.path.join(get_package_share_directory('mrp_real_robot'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'MRP_Mirror.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    params = {'robot_description': ParameterValue(robot_description_config, value_type = str)}

    # mirrored robot state publisher:
    mirrored_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name = 'mirrored_robot_state_publisher',
        output='screen',
        parameters=[params],
        remappings = [('/robot_description', '/mirrored_robot_description')]
    )

    # launch gazebo:
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]), 
            launch_arguments = {
                # 'verbose': 'true',
                'extra_gazebo_args' : '--ros-args -p use_sim_time:=false'
                }.items()
    )

    # spawn the robot model:
    spawn_entity = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        arguments = [
            '-topic', '/mirrored_robot_description',
            '-entity', 'mirrored_robot'
        ]
    )

    # joystick node:
    joystick_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('mrp_description'), 'launch', 'joystick.launch.py')])
        , launch_arguments = {'use_ros_control': 'true'}.items()
    )

    return LaunchDescription([
        mirrored_robot_state_publisher, 
        gazebo_node, 
        spawn_entity,
        joystick_node
    ])
    