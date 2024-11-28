import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='dynamic_tf',
        #     executable='static_tf_base_laser',
        #     name='static_tf_base_laser',
        # ),
        # Node(
        #     package='dynamic_tf',
        #     executable='static_tf_back_laser',
        #     name='static_tf_back_laser',
        # ),
        # Node(
        #     package='dynamic_tf',
        #     executable='static_tf_cam_tag',
        #     name='static_tf_cam_tag',
        # ),        
        # Node(
        #     package='dynamic_tf',
        #     executable='static_tf_end_effector_home',
        #     name='static_tf_end_effector_home',
        # ),
        # Node(
        #     package='dynamic_tf',
        #     executable='static_tf_plate_top',
        #     name='static_tf_plate_top',
        # ),
        # Node(
        #     package='dynamic_tf',
        #     executable='static_tf_front_laser',
        #     name='static_tf_front_laser',
        # ),
         Node(
             package='dynamic_tf',
             executable='static_tf_gripper_origin',
             name='static_tf_gripper_origin',
         ),
         Node(
             package='dynamic_tf',
             executable='dyn_tf_gripper_x_ori',
             name='dyn_tf_gripper_x_ori',
         ),
          Node(
              package='dynamic_tf',
              executable='dyn_tf_gripper_y_ori',
              name='dyn_tf_gripper_y_ori',
          ),
         Node(
             package='dynamic_tf',
             executable='dyn_tf_gripper_z_ori',
             name='dyn_tf_gripper_z_ori',
         ),
    ])
