import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamic_tf',
            executable='static_tf_base_laser',
            name='static_tf_base_laser',
        ),
        Node(
            package='dynamic_tf',
            executable='test',
            name='test',
        ),
    ])