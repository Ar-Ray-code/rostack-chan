# Copyright 2023 Ar-Ray-code.
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
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    rostackchan_dir = get_package_share_directory('rostackchan_description')

    rostackchan_ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            rostackchan_dir, 'launch', 'rostackchan.launch.py')),
    )

    random_face = Node(
        package='rostackchan_example',
        executable='random_face',
        name='random_face',
        output='screen'
    )

    random_move = Node(
        package='rostackchan_example',
        executable='random_move',
        name='random_move',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(random_face)
    ld.add_action(random_move)
    ld.add_action(rostackchan_ros2_control)
    return ld
