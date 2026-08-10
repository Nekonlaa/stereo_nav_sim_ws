#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_launch = os.path.join(
        get_package_share_directory("stereo_nav_gazebo"), "launch", "sim.launch.py"
    )
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "gui": LaunchConfiguration("gui"),
                "headless": LaunchConfiguration("headless"),
                "rviz": LaunchConfiguration("rviz"),
            }.items(),
        ),
    ])
