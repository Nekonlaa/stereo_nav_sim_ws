#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _launch_setup(context):
    localization = _as_bool(LaunchConfiguration("localization").perform(context))
    new_map = _as_bool(LaunchConfiguration("new_map").perform(context))
    database_path = os.path.abspath(
        os.path.expanduser(LaunchConfiguration("database_path").perform(context))
    )

    os.makedirs(os.path.dirname(database_path), exist_ok=True)
    if localization and not os.path.isfile(database_path):
        raise RuntimeError(
            f"Localization requires an existing RTAB-Map database: {database_path}"
        )
    if localization and new_map:
        raise RuntimeError("new_map:=true is invalid in localization mode")

    sync_parameters = {
        "use_sim_time": True,
        "approx_sync": True,
        "approx_sync_max_interval": 0.003,
        "topic_queue_size": 30,
        "sync_queue_size": 20,
        "qos": 2,
        "qos_camera_info": 2,
    }

    stereo_remappings = [
        ("left/image_rect", "/stereo/left/image_raw"),
        ("right/image_rect", "/stereo/right/image_raw"),
        ("left/camera_info", "/stereo/left/camera_info"),
        ("right/camera_info", "/stereo/right/camera_info"),
    ]

    odometry_arguments = [
        "--Reg/Force3DoF", "true",
        "--Vis/MinInliers", "20",
        "--Vis/MaxFeatures", "1000",
        "--Rtabmap/ImagesAlreadyRectified", "true",
    ]

    rtabmap_arguments = [
        "--Reg/Force3DoF", "true",
        "--Rtabmap/DetectionRate", "5",
        "--Rtabmap/ImagesAlreadyRectified", "true",
        "--Vis/MinInliers", "20",
        "--Vis/MaxFeatures", "1000",
        "--RGBD/CreateOccupancyGrid", "true",
        "--Grid/Sensor", "1",
        "--Grid/3D", "false",
        "--Grid/CellSize", "0.05",
        "--Grid/RangeMax", "5.0",
        "--Grid/NormalsSegmentation", "true",
        "--Grid/MaxObstacleHeight", "1.5",
        "--Grid/RayTracing", "true",
        "--Grid/FootprintLength", "0.47",
        "--Grid/FootprintWidth", "0.36",
        "--Grid/FootprintHeight", "0.50",
    ]
    if new_map:
        rtabmap_arguments.append("--delete_db_on_start")

    actions = [
        LogInfo(msg=f"RTAB-Map database: {database_path}"),
        LogInfo(msg="RTAB-Map mode: localization" if localization else "RTAB-Map mode: mapping"),
        Node(
            package="rtabmap_odom",
            executable="stereo_odometry",
            name="stereo_odometry",
            output="screen",
            emulate_tty=True,
            parameters=[sync_parameters, {
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "publish_tf": True,
                "wait_for_transform": 0.20,
                "wait_imu_to_init": False,
                "subscribe_rgbd": False,
                "always_process_most_recent_frame": True,
            }],
            remappings=stereo_remappings + [
                ("odom", "/odom"),
                ("odom_info", "/odom_info"),
            ],
            arguments=odometry_arguments,
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "use_sim_time": True,
                "subscribe_depth": False,
                "subscribe_rgbd": False,
                "subscribe_rgb": False,
                "subscribe_stereo": True,
                "subscribe_scan": False,
                "subscribe_scan_cloud": False,
                "subscribe_user_data": False,
                "subscribe_odom_info": True,
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "",
                "publish_tf": True,
                "database_path": database_path,
                "approx_sync": True,
                "approx_sync_max_interval": 0.003,
                "topic_queue_size": 30,
                "sync_queue_size": 20,
                "qos_image": 2,
                "qos_camera_info": 2,
                "qos_odom": 2,
                "wait_for_transform": 0.20,
                "Mem/IncrementalMemory": "false" if localization else "true",
                "Mem/InitWMWithAllNodes": "true" if localization else "false",
            }],
            remappings=stereo_remappings + [
                ("odom", "/odom"),
                ("odom_info", "/odom_info"),
                ("map", "/map"),
                # CoreWrapper creates several optional asynchronous subscribers.
                # Isolate them so no system IMU/GPS/landmark source can enter this graph.
                ("imu", "/_stereo_nav_disabled/imu"),
                ("gps/fix", "/_stereo_nav_disabled/gps"),
                ("env_sensor", "/_stereo_nav_disabled/env_sensor"),
                ("landmark_detection", "/_stereo_nav_disabled/landmark_detection"),
                ("tag_detections", "/_stereo_nav_disabled/tag_detections"),
                ("fiducial_transforms", "/_stereo_nav_disabled/fiducial_transforms"),
            ],
            arguments=rtabmap_arguments,
        ),
        Node(
            package="rtabmap_util",
            executable="point_cloud_xyzrgb",
            name="stereo_point_cloud",
            output="screen",
            emulate_tty=True,
            parameters=[sync_parameters, {
                "decimation": 2,
                "voxel_size": 0.05,
            }],
            remappings=[
                ("left/image", "/stereo/left/image_raw"),
                ("right/image", "/stereo/right/image_raw"),
                ("left/camera_info", "/stereo/left/camera_info"),
                ("right/camera_info", "/stereo/right/camera_info"),
                ("cloud", "/stereo/points2"),
            ],
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "localization", default_value="false", description="Load database in localization mode."
        ),
        DeclareLaunchArgument(
            "new_map",
            default_value="false",
            description="Explicitly delete the selected database before mapping.",
        ),
        DeclareLaunchArgument(
            "database_path",
            default_value="~/.ros/stereo_nav/rtabmap.db",
            description="RTAB-Map database to create or load.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
