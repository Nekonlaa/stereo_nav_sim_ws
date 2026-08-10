#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _gazebo_launch(context, ros_gz_share, world_path):
    gui = LaunchConfiguration("gui").perform(context).lower() in ("1", "true", "yes")
    headless = LaunchConfiguration("headless").perform(context).lower() in ("1", "true", "yes")

    if headless:
        gz_args = f"-r -s --headless-rendering -v 3 {world_path}"
    elif gui:
        gz_args = f"-r -v 3 {world_path}"
    else:
        gz_args = f"-r -s -v 3 {world_path}"

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
    )]


def generate_launch_description():
    gazebo_share = get_package_share_directory("stereo_nav_gazebo")
    description_share = get_package_share_directory("stereo_nav_description")
    ros_gz_share = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(gazebo_share, "worlds", "indoor_stereo.sdf")
    model_path = os.path.join(gazebo_share, "models", "stereo_rover", "model.sdf")
    bridge_path = os.path.join(gazebo_share, "config", "bridge.yaml")
    rviz_path = os.path.join(gazebo_share, "rviz", "stereo_nav.rviz")
    xacro_path = os.path.join(description_share, "urdf", "stereo_rover.urdf.xacro")

    rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Start the Gazebo GUI."),
        DeclareLaunchArgument("headless", default_value="false", description="Use server-only rendering mode."),
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz."),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", gazebo_share),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gazebo_share),
        OpaqueFunction(function=_gazebo_launch, args=[ros_gz_share, world_path]),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "robot_description": Command(["xacro ", xacro_path]),
            }],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_stereo_rover",
            output="screen",
            arguments=[
                "-world", "indoor_stereo",
                "-file", model_path,
                "-name", "stereo_rover",
                "-allow_renaming", "false",
                "-x", "-4.2", "-y", "-3.4", "-z", "0.02", "-Y", "0.0",
            ],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="stereo_nav_bridge",
            output="screen",
            parameters=[{"config_file": bridge_path, "use_sim_time": True}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_path],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(rviz),
        ),
    ])
