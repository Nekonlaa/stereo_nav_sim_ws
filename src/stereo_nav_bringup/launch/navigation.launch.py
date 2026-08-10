#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _database_preflight(context):
    database_path = os.path.abspath(
        os.path.expanduser(LaunchConfiguration("database_path").perform(context))
    )
    if not os.path.isfile(database_path):
        raise RuntimeError(
            f"Navigation requires an existing RTAB-Map database: {database_path}"
        )
    return []


def generate_launch_description():
    bringup_share = get_package_share_directory("stereo_nav_bringup")
    gazebo_share = get_package_share_directory("stereo_nav_gazebo")
    nav2_share = get_package_share_directory("nav2_bringup")

    sim_launch = os.path.join(gazebo_share, "launch", "sim.launch.py")
    slam_launch = os.path.join(bringup_share, "launch", "stereo_slam.launch.py")
    nav2_launch = os.path.join(nav2_share, "launch", "navigation_launch.py")
    nav2_params = os.path.join(bringup_share, "config", "nav2.yaml")
    rviz_config = os.path.join(gazebo_share, "rviz", "stereo_nav.rviz")

    start_sim = LaunchConfiguration("start_sim")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument("start_sim", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("moving_obstacle", default_value="false"),
        DeclareLaunchArgument("database_path", default_value="~/.ros/stereo_nav/rtabmap.db"),
        OpaqueFunction(function=_database_preflight),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            condition=IfCondition(start_sim),
            launch_arguments={
                "gui": LaunchConfiguration("gui"),
                "headless": LaunchConfiguration("headless"),
                "rviz": rviz,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                "localization": "true",
                "new_map": "false",
                "database_path": LaunchConfiguration("database_path"),
            }.items(),
        ),
        TimerAction(
            period=5.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "use_sim_time": "true",
                    "autostart": "true",
                    "params_file": nav2_params,
                    "use_composition": "False",
                }.items(),
            )],
        ),
        Node(
            package="stereo_nav_bringup",
            executable="moving_obstacle_controller",
            name="moving_obstacle_controller",
            output="screen",
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("moving_obstacle")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(PythonExpression([
                "'", start_sim, "' != 'true' and '", rviz, "' == 'true'"
            ])),
        ),
    ])
