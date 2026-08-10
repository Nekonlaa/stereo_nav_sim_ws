#!/usr/bin/env python3
"""Evaluate stereo odometry against Gazebo truth after first-pose SE(2) alignment."""

import argparse
from bisect import bisect_left
import math
import sys
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args


def _stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9


def _yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _pose(message):
    position = message.pose.pose.position
    return position.x, position.y, _yaw(message.pose.pose.orientation)


def _relative(poses):
    x0, y0, yaw0 = poses[0]
    cosine = math.cos(yaw0)
    sine = math.sin(yaw0)
    result = []
    for x_value, y_value, yaw_value in poses:
        dx = x_value - x0
        dy = y_value - y0
        result.append((
            cosine * dx + sine * dy,
            -sine * dx + cosine * dy,
            math.atan2(math.sin(yaw_value - yaw0), math.cos(yaw_value - yaw0)),
        ))
    return result


class TrajectoryCollector(Node):
    def __init__(self):
        super().__init__("stereo_nav_trajectory_evaluator")
        self.estimated = []
        self.truth = []
        self.create_subscription(Odometry, "/odom", self._estimated, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "/ground_truth/odom", self._truth, qos_profile_sensor_data
        )

    def _estimated(self, message):
        self.estimated.append((_stamp(message), _pose(message)))

    def _truth(self, message):
        self.truth.append((_stamp(message), _pose(message)))


def _pair(estimated, truth, tolerance=0.05):
    truth = sorted(truth)
    truth_times = [item[0] for item in truth]
    pairs = []
    for stamp, pose in sorted(estimated):
        index = bisect_left(truth_times, stamp)
        candidates = []
        if index < len(truth):
            candidates.append(truth[index])
        if index:
            candidates.append(truth[index - 1])
        if not candidates:
            continue
        nearest = min(candidates, key=lambda item: abs(item[0] - stamp))
        if abs(nearest[0] - stamp) <= tolerance:
            pairs.append((pose, nearest[1]))
    return pairs


def _path_length(poses):
    return sum(
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(poses, poses[1:])
    )


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--minimum-distance", type=float, default=20.0)
    parser.add_argument("--rmse-limit", type=float, default=0.30)
    parser.add_argument("--final-limit", type=float, default=0.25)
    parsed = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init(args=args)
    node = TrajectoryCollector()
    deadline = time.monotonic() + parsed.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        pairs = _pair(node.estimated, node.truth)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if len(pairs) < 20:
        print(f"FAIL: only {len(pairs)} synchronized odometry pairs", file=sys.stderr)
        raise SystemExit(1)

    estimated = _relative([item[0] for item in pairs])
    truth = _relative([item[1] for item in pairs])
    errors = [
        math.hypot(est[0] - gt[0], est[1] - gt[1])
        for est, gt in zip(estimated, truth)
    ]
    rmse = math.sqrt(sum(value * value for value in errors) / len(errors))
    final_error = errors[-1]
    distance = _path_length(truth)
    print(
        f"trajectory metrics: pairs={len(pairs)} distance={distance:.2f}m "
        f"ATE_RMSE={rmse:.3f}m final_error={final_error:.3f}m"
    )

    failures = []
    if distance < parsed.minimum_distance:
        failures.append(f"route is shorter than {parsed.minimum_distance:.1f}m")
    if rmse > parsed.rmse_limit:
        failures.append(f"ATE RMSE exceeds {parsed.rmse_limit:.2f}m")
    if final_error > parsed.final_limit:
        failures.append(f"final error exceeds {parsed.final_limit:.2f}m")
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        raise SystemExit(1)
    print("PASS: trajectory accuracy")


if __name__ == "__main__":
    main()
