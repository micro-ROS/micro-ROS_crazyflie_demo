# Copyright (c) 2019 - for information on the respective copyright owner
# see the NOTICE file and/or the repository https://github.com/micro-ROS/micro-ROS_kobuki_demo.
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   
    urdfcrazyflie_attitude = os.path.join(get_package_share_directory('micro-ros_crazyflie_demo_robot-description'),
                    'urdf', 'crazyflie2_attitude.xacro')
    

    rviz_config_crazyflie_attitude = os.path.join(get_package_share_directory('micro-ros_crazyflie_demo_remote'),
                               'config', 'crazyflie_attitude.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            output='screen', arguments=[urdfcrazyflie_attitude]),
        Node(
            package='rviz2',
            node_executable='rviz2',
            arguments=['-d', rviz_config_crazyflie_attitude])   
])