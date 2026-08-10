#!/usr/bin/env python3
"""Audit rates, stereo calibration, topic ownership, and forbidden inputs."""

import argparse
from bisect import bisect_left
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


def _stamp_seconds(message):
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _rate(stamps):
    unique = sorted(set(stamps))
    if len(unique) < 2 or unique[-1] <= unique[0]:
        return 0.0
    return (len(unique) - 1) / (unique[-1] - unique[0])


def _nearest_deltas(left, right):
    right = sorted(right)
    deltas = []
    for value in sorted(left):
        index = bisect_left(right, value)
        candidates = []
        if index < len(right):
            candidates.append(abs(value - right[index]))
        if index:
            candidates.append(abs(value - right[index - 1]))
        if candidates:
            deltas.append(min(candidates))
    return deltas


def _percentile(values, fraction):
    if not values:
        return math.inf
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


class RuntimeAudit(Node):
    def __init__(self):
        super().__init__("stereo_nav_runtime_audit")
        self.left_stamps = []
        self.right_stamps = []
        self.cloud_stamps = []
        self.left_shape = None
        self.right_shape = None
        self.left_encoding = None
        self.right_encoding = None
        self.left_info = None
        self.right_info = None
        self.odom_samples = 0
        self.valid_odom_samples = 0
        self.invalid_odom_start = None
        self.last_odom_stamp = None
        self.max_invalid_odom_seconds = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.create_subscription(
            Image, "/stereo/left/image_raw", self._left_image, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/stereo/right/image_raw", self._right_image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, "/stereo/left/camera_info", self._left_info, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, "/stereo/right/camera_info", self._right_info, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "/stereo/points2", self._cloud, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/odom", self._odom, qos_profile_sensor_data
        )

    def _left_image(self, message):
        self.left_stamps.append(_stamp_seconds(message))
        self.left_shape = (message.width, message.height)
        self.left_encoding = message.encoding

    def _right_image(self, message):
        self.right_stamps.append(_stamp_seconds(message))
        self.right_shape = (message.width, message.height)
        self.right_encoding = message.encoding

    def _left_info(self, message):
        self.left_info = message

    def _right_info(self, message):
        self.right_info = message

    def _cloud(self, message):
        self.cloud_stamps.append(_stamp_seconds(message))

    def _odom(self, message):
        self.odom_samples += 1
        stamp = _stamp_seconds(message)
        self.last_odom_stamp = stamp
        orientation = message.pose.pose.orientation
        norm = math.sqrt(
            orientation.x ** 2
            + orientation.y ** 2
            + orientation.z ** 2
            + orientation.w ** 2
        )
        if abs(norm - 1.0) <= 0.05:
            self.valid_odom_samples += 1
            if self.invalid_odom_start is not None:
                self.max_invalid_odom_seconds = max(
                    self.max_invalid_odom_seconds,
                    stamp - self.invalid_odom_start,
                )
                self.invalid_odom_start = None
        elif self.invalid_odom_start is None:
            self.invalid_odom_start = stamp

    def graph_errors(self):
        errors = []
        odom_publishers = {
            info.node_name for info in self.get_publishers_info_by_topic("/odom")
        }
        if odom_publishers != {"stereo_odometry"}:
            errors.append(f"/odom publishers must be only stereo_odometry, got {sorted(odom_publishers)}")

        map_publishers = {
            info.node_name for info in self.get_publishers_info_by_topic("/map")
        }
        if map_publishers != {"rtabmap"}:
            errors.append(f"/map publishers must be only rtabmap, got {sorted(map_publishers)}")

        allowed_tf_publishers = {"robot_state_publisher", "stereo_odometry", "rtabmap"}
        tf_publishers = {
            info.node_name for info in self.get_publishers_info_by_topic("/tf")
        }
        unexpected_tf = tf_publishers - allowed_tf_publishers
        if unexpected_tf:
            errors.append(f"unexpected /tf publishers: {sorted(unexpected_tf)}")

        forbidden = {"/imu", "/scan", "/scan_cloud", "/ground_truth/odom"}
        target_names = {
            "rtabmap",
            "stereo_odometry",
            "controller_server",
            "planner_server",
            "bt_navigator",
            "local_costmap",
            "global_costmap",
        }
        for node_name, namespace in self.get_node_names_and_namespaces():
            if node_name not in target_names:
                continue
            try:
                subscriptions = self.get_subscriber_names_and_types_by_node(node_name, namespace)
            except RuntimeError as error:
                errors.append(f"cannot inspect {namespace}/{node_name}: {error}")
                continue
            topics = {name for name, _types in subscriptions}
            bad = topics & forbidden
            if bad:
                errors.append(f"{namespace}/{node_name} subscribes to forbidden topics {sorted(bad)}")
        return errors

    def result(self, minimum_valid_odom_fraction=0.50, maximum_invalid_odom_seconds=math.inf):
        max_invalid_odom_seconds = self.max_invalid_odom_seconds
        if self.invalid_odom_start is not None and self.last_odom_stamp is not None:
            max_invalid_odom_seconds = max(
                max_invalid_odom_seconds,
                self.last_odom_stamp - self.invalid_odom_start,
            )
        metrics = {
            "left_hz": _rate(self.left_stamps),
            "right_hz": _rate(self.right_stamps),
            "cloud_hz": _rate(self.cloud_stamps),
            "sync_p95_ms": _percentile(
                _nearest_deltas(self.left_stamps, self.right_stamps), 0.95
            ) * 1000.0,
            "valid_odom_fraction": (
                self.valid_odom_samples / self.odom_samples
                if self.odom_samples
                else 0.0
            ),
            "max_invalid_odom_seconds": max_invalid_odom_seconds,
        }
        errors = []
        if not 28.0 <= metrics["left_hz"] <= 32.0:
            errors.append(f"left image rate outside 30+/-2 Hz: {metrics['left_hz']:.2f}")
        if not 28.0 <= metrics["right_hz"] <= 32.0:
            errors.append(f"right image rate outside 30+/-2 Hz: {metrics['right_hz']:.2f}")
        if metrics["cloud_hz"] < 10.0:
            errors.append(f"point cloud rate below 10 Hz: {metrics['cloud_hz']:.2f}")
        if metrics["sync_p95_ms"] > 3.0:
            errors.append(f"stereo timestamp delta P95 exceeds 3 ms: {metrics['sync_p95_ms']:.3f}")
        if self.odom_samples == 0:
            errors.append("no visual odometry samples received")
        elif metrics["valid_odom_fraction"] < minimum_valid_odom_fraction:
            errors.append(
                "visual odometry valid-pose fraction is below "
                f"{minimum_valid_odom_fraction * 100.0:.1f}%: "
                f"{metrics['valid_odom_fraction'] * 100.0:.1f}%"
            )
        if metrics["max_invalid_odom_seconds"] > maximum_invalid_odom_seconds:
            errors.append(
                "continuous invalid visual odometry exceeds "
                f"{maximum_invalid_odom_seconds:.3f}s: "
                f"{metrics['max_invalid_odom_seconds']:.3f}s"
            )
        try:
            self.tf_buffer.lookup_transform("odom", "base_link", Time())
        except TransformException as error:
            errors.append(f"missing odom->base_link visual transform: {error}")
        if self.left_shape != (640, 480) or self.right_shape != (640, 480):
            errors.append(f"unexpected image dimensions: {self.left_shape}, {self.right_shape}")
        if self.left_encoding != "mono8" or self.right_encoding != "mono8":
            errors.append(f"unexpected image encodings: {self.left_encoding}, {self.right_encoding}")
        if self.left_info is None or self.right_info is None:
            errors.append("missing CameraInfo")
        else:
            for label, info in (("left", self.left_info), ("right", self.right_info)):
                if abs(info.k[0] - 320.0) > 0.1 or abs(info.k[4] - 320.0) > 0.1:
                    errors.append(f"{label} CameraInfo focal length is invalid")
            if abs(self.right_info.p[3] + 38.4) > 0.1:
                errors.append(f"right P[3] must be -38.4, got {self.right_info.p[3]:.3f}")
        errors.extend(self.graph_errors())
        return metrics, errors


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--minimum-valid-odom-fraction", type=float, default=0.50)
    parser.add_argument("--maximum-invalid-odom-seconds", type=float, default=math.inf)
    parsed = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init(args=args)
    node = RuntimeAudit()
    deadline = time.monotonic() + parsed.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        metrics, errors = node.result(
            minimum_valid_odom_fraction=parsed.minimum_valid_odom_fraction,
            maximum_invalid_odom_seconds=parsed.maximum_invalid_odom_seconds,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(
        "runtime metrics: "
        f"left={metrics['left_hz']:.2f}Hz right={metrics['right_hz']:.2f}Hz "
        f"cloud={metrics['cloud_hz']:.2f}Hz sync_p95={metrics['sync_p95_ms']:.3f}ms "
        f"valid_odom={metrics['valid_odom_fraction'] * 100.0:.1f}% "
        f"max_invalid_odom={metrics['max_invalid_odom_seconds']:.3f}s"
    )
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
    print("PASS: runtime stereo and graph audit")


if __name__ == "__main__":
    main()
