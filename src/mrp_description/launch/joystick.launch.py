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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # launch params:
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # launch arguments:
    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control if true')

    joy_params = os.path.join(get_package_share_directory('mrp_description'), 'config', 'joystick.yaml')

    joy_node = Node(
        package = 'joy',
        executable = 'joy_node',
        parameters = [joy_params]
    )

    teleop_node_ros2_control = Node(
    package = 'teleop_twist_joy',
    executable = 'teleop_node',
    name = 'teleop_node',
    parameters = [joy_params],
    remappings = [('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
    condition = IfCondition(use_ros2_control)
    )

    teleop_node_gazebo = Node(
    package = 'teleop_twist_joy',
    executable = 'teleop_node',
    name = 'teleop_node',
    parameters = [joy_params],
    condition = UnlessCondition(use_ros2_control)
)

    return LaunchDescription([
        use_ros2_control_arg, 
        joy_node,
        teleop_node_ros2_control,
        teleop_node_gazebo
    ])