#!/usr/bin/env python3
#
# Launch AI Worker controller (follower) with VR trigger gripper fallback.
# Use when goal poses come from VR (e.g. vr_publisher_sg2) and gripper from
# /vr_controller/left_trigger, /vr_controller/right_trigger.
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('motion_controller_ros'),
                'config',
                'controller_config.yaml',
            ]),
            description='Path to controller config file.',
        ),
        DeclareLaunchArgument(
            'follower_urdf_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('motion_controller_ros'),
                'models',
                'ai_worker',
                'ffw_sg2_follower.urdf',
            ]),
            description='Path to follower robot URDF.',
        ),
        DeclareLaunchArgument(
            'follower_srdf_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('motion_controller_ros'),
                'models',
                'ai_worker',
                'ffw_sg2_follower.srdf',
            ]),
            description='Path to follower robot SRDF.',
        ),
        DeclareLaunchArgument(
            'left_trigger_topic',
            default_value='/vr_controller/left_trigger',
            description='Topic for VR left trigger (gripper fallback). Set to empty to disable.',
        ),
        DeclareLaunchArgument(
            'right_trigger_topic',
            default_value='/vr_controller/right_trigger',
            description='Topic for VR right trigger (gripper fallback). Set to empty to disable.',
        ),
    ]

    config_file = LaunchConfiguration('config_file')
    follower_urdf_path = LaunchConfiguration('follower_urdf_path')
    follower_srdf_path = LaunchConfiguration('follower_srdf_path')
    left_trigger_topic = LaunchConfiguration('left_trigger_topic')
    right_trigger_topic = LaunchConfiguration('right_trigger_topic')

    ai_worker_controller_node = Node(
        package='motion_controller_ros',
        executable='ai_worker_controller_node',
        name='ai_worker_controller',
        parameters=[
            config_file,
            {
                'urdf_path': follower_urdf_path,
                'srdf_path': follower_srdf_path,
                'left_trigger_topic': left_trigger_topic,
                'right_trigger_topic': right_trigger_topic,
            },
        ],
        output='screen',
    )

    return LaunchDescription(declared_arguments + [ai_worker_controller_node])
