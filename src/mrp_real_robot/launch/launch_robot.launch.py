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
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    # launch configuration:
    use_sim_time = LaunchConfiguration('use_sim_time')

    # launch argument:
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true')
    
    # prepare the URDF file:
    pkg_path = os.path.join(get_package_share_directory('mrp_real_robot'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'MRP.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    params = {'robot_description': ParameterValue(robot_description_config, value_type = str), 'use_sim_time': use_sim_time}

    # create a robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # launch:
    return LaunchDescription([
        use_sim_time_arg, 
        robot_state_publisher
    ])