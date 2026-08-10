"""Offline contract tests for calibration, bridges, and strict stereo-only inputs."""

from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import yaml


SOURCE_ROOT = Path(__file__).resolve().parents[2]
MODEL = SOURCE_ROOT / "stereo_nav_gazebo" / "models" / "stereo_rover" / "model.sdf"
WORLD = SOURCE_ROOT / "stereo_nav_gazebo" / "worlds" / "indoor_stereo.sdf"
FLOOR_TEXTURE = (
    SOURCE_ROOT
    / "stereo_nav_gazebo"
    / "materials"
    / "textures"
    / "indoor_floor.png"
)
BRIDGE = SOURCE_ROOT / "stereo_nav_gazebo" / "config" / "bridge.yaml"
NAV2 = SOURCE_ROOT / "stereo_nav_bringup" / "config" / "nav2.yaml"
SLAM = SOURCE_ROOT / "stereo_nav_bringup" / "launch" / "stereo_slam.launch.py"
URDF = SOURCE_ROOT / "stereo_nav_description" / "urdf" / "stereo_rover.urdf.xacro"


def _camera_sensors():
    root = ET.parse(MODEL).getroot()
    return root.findall(".//sensor[@type='camera']")


def test_only_two_camera_sensors_are_present():
    root = ET.parse(MODEL).getroot()
    sensors = root.findall(".//sensor")
    assert len(sensors) == 2
    assert [sensor.attrib["type"] for sensor in sensors] == ["camera", "camera"]
    assert not root.findall(".//sensor[@type='imu']")
    assert not root.findall(".//sensor[@type='gpu_lidar']")
    assert not root.findall(".//sensor[@type='ray']")


def test_visual_landmarks_cover_all_rooms_without_adding_collisions():
    root = ET.parse(WORLD).getroot()
    feature_model = root.find(".//model[@name='distributed_floor_features']")
    assert feature_model is not None
    assert not feature_model.findall(".//collision")

    visuals = feature_model.findall(".//visual")
    assert len(visuals) >= 56
    positions = [tuple(map(float, visual.findtext("pose").split()[:2])) for visual in visuals]
    assert any(x < 0.0 and y < 0.0 for x, y in positions)
    assert any(x < 0.0 and y > 0.0 for x, y in positions)
    assert any(x > 0.0 and y < 0.0 for x, y in positions)
    assert any(x > 0.0 and y > 0.0 for x, y in positions)


def test_floor_uses_a_local_high_resolution_nonempty_texture():
    root = ET.parse(WORLD).getroot()
    floor = root.find(".//visual[@name='floor_visual']")
    assert floor is not None
    assert floor.findtext("material/pbr/metal/albedo_map") == (
        "../materials/textures/indoor_floor.png"
    )
    assert FLOOR_TEXTURE.is_file()
    data = FLOOR_TEXTURE.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    width, height = struct.unpack(">II", data[16:24])
    assert width >= 1024 and height >= 1024
    assert len(data) >= 50_000


def test_stereo_camera_contract():
    cameras = {sensor.attrib["name"]: sensor for sensor in _camera_sensors()}
    assert set(cameras) == {"left_camera", "right_camera"}

    for sensor in cameras.values():
        assert float(sensor.findtext("update_rate")) == 30.0
        assert sensor.findtext("camera/image/width") == "640"
        assert sensor.findtext("camera/image/height") == "480"
        assert sensor.findtext("camera/image/format") == "L8"
        assert float(sensor.findtext("camera/intrinsics/fx")) == 320.0
        assert float(sensor.findtext("camera/intrinsics/fy")) == 320.0
        assert float(sensor.findtext("camera/intrinsics/cx")) == 319.5
        assert float(sensor.findtext("camera/intrinsics/cy")) == 239.5
        assert float(sensor.findtext("camera/clip/near")) == 0.15
        assert float(sensor.findtext("camera/clip/far")) == 8.0

    left_y = float(cameras["left_camera"].findtext("pose").split()[1])
    right_y = float(cameras["right_camera"].findtext("pose").split()[1])
    assert abs((left_y - right_y) - 0.12) < 1.0e-9
    assert float(cameras["left_camera"].findtext("camera/projection/tx")) == 0.0
    assert abs(float(cameras["right_camera"].findtext("camera/projection/tx")) + 38.4) < 0.1


def test_optical_frames_follow_rep_103_rotation():
    root = ET.parse(URDF).getroot()
    optical_joint = root.find(".//joint[@name='stereo_${side}_optical_joint']")
    assert optical_joint is not None
    assert optical_joint.find("origin").attrib["rpy"] == "${-pi/2} 0 ${-pi/2}"


def test_bridge_does_not_import_ground_truth_tf_or_forbidden_sensors():
    entries = yaml.safe_load(BRIDGE.read_text(encoding="utf-8"))
    ros_topics = {entry["ros_topic_name"] for entry in entries}
    assert "/tf" not in ros_topics
    assert "/tf_static" not in ros_topics
    assert "/imu" not in ros_topics
    assert "/scan" not in ros_topics
    assert "/ground_truth/odom" in ros_topics
    gt = next(entry for entry in entries if entry["ros_topic_name"] == "/ground_truth/odom")
    assert gt["direction"] == "GZ_TO_ROS"


def test_ground_truth_comes_from_actual_model_pose_not_diff_drive_integration():
    root = ET.parse(MODEL).getroot()
    plugins = {plugin.attrib["name"]: plugin for plugin in root.findall(".//plugin")}
    diff_drive = plugins["ignition::gazebo::systems::DiffDrive"]
    truth = plugins["ignition::gazebo::systems::OdometryPublisher"]
    assert diff_drive.find("odom_topic") is None
    assert truth.findtext("odom_topic") == "/ground_truth/odom"
    assert truth.findtext("odom_frame") == "world"
    assert truth.findtext("robot_base_frame") == "base_link"


def test_nav2_uses_only_stereo_cloud_for_live_obstacles():
    text = NAV2.read_text(encoding="utf-8")
    params = yaml.safe_load(text)
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]
    source = local["voxel_layer"]["stereo_points"]
    assert source["topic"] == "/stereo/points2"
    assert source["data_type"] == "PointCloud2"
    assert source["min_obstacle_height"] == 0.05
    assert source["max_obstacle_height"] == 1.50
    assert "/scan" not in text
    assert "/imu" not in text
    assert "/ground_truth/odom" not in text


def test_slam_inputs_and_database_guard_are_explicit():
    text = SLAM.read_text(encoding="utf-8")
    assert '"subscribe_stereo": True' in text
    assert '"subscribe_scan": False' in text
    assert '"subscribe_scan_cloud": False' in text
    assert '"wait_imu_to_init": False' in text
    assert '"approx_sync_max_interval": 0.003' in text
    assert 'if new_map:' in text
    assert '"--delete_db_on_start"' in text
