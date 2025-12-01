#!/usr/bin/env python

# Code for launching motion tracking nodes for GRASP and RAFTI tracking
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mocap4r2_optitrack_driver_launch_dir = PathJoinSubstitution([FindPackageShare('mocap4r2_optitrack_driver'), 'launch'])
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([mocap4r2_optitrack_driver_launch_dir, 'optitrack2.launch.py'])
        ),
        Node(
            package='mocap-transforms',
            executable='mocap_tf2_broadcaster',
            name='mocap_tf2_broadcaster'
        ),
        Node(
            package='mocap-transforms',
            executable='mocap_tf2_post_processor',
            name='mocap_tf2_post_processor'
        )
    ])