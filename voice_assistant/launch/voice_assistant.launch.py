import os
import sys
from glob import glob
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    pkg_dir = get_package_share_directory('voice_assistant')
    list = [
        Node(
            package='openai_ros2',
            executable='chatgpt',
            namespace='',
            output="screen",
            respawn=True,
        ),
        Node(
            package='whisper_ros2',
            executable='whisper_ros2',
            namespace='',
            output="screen",
            respawn=True,
        ),
        Node(
            package='voicevox_ros2',
            executable='voicevox_ros2',
            namespace='',
            parameters=[os.path.join(pkg_dir, 'config', 'voicevox_ros2.yaml')],
            output="screen",
            respawn=True,
        ),
        Node(
            package='voice_assistant',
            executable='voice_assistant',
            namespace='',
            output="screen",
            respawn=True,
        ),
    ]

    return LaunchDescription(list)
