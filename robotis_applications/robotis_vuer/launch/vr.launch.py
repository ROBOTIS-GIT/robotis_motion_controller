#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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
#
# Authors: Wonho Yun

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='sh5',
        description='VR model to run: sg2 or sh5',
    )

    model = LaunchConfiguration('model')
    sg2_node = Node(
        package='robotis_vuer',
        executable='vr_publisher_sg2',
        name='vr_publisher_sg2',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression(["'true' if '", model, "' == 'sg2' else 'false'"])
        ),
    )
    sh5_node = Node(
        package='robotis_vuer',
        executable='vr_publisher_sh5',
        name='vr_publisher_sh5',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression(["'true' if '", model, "' == 'sh5' else 'false'"])
        ),
    )

    return LaunchDescription([
        model_arg,
        sg2_node,
        sh5_node,
    ])
