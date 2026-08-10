#!/usr/bin/env python3
"""Send five cross-room Nav2 goals and verify truth-relative final error."""

import argparse
import math
import sys
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args


# Map coordinates assume the unchanged spawn pose in stereo_nav_gazebo/sim.launch.py.
DEFAULT_GOALS = [
    (0.8, 2.5, math.pi / 2.0),
    (1.5, 7.2, 0.0),
    (8.7, 0.4, 0.0),
    (7.5, 4.4, math.pi / 2.0),
    (8.0, 7.0, math.pi),
]
SPAWN_X = -4.2
SPAWN_Y = -3.4


class NavigationAcceptance(Node):
    def __init__(self):
        super().__init__("stereo_nav_navigation_acceptance")
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.truth_position = None
        self.create_subscription(
            Odometry, "/ground_truth/odom", self._truth, qos_profile_sensor_data
        )

    def _truth(self, message):
        position = message.pose.pose.position
        self.truth_position = (
            position.x - SPAWN_X,
            position.y - SPAWN_Y,
        )

    def _wait_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def run_goal(self, x_value, y_value, yaw, timeout):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x_value
        goal.pose.pose.position.y = y_value
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send_future = self.client.send_goal_async(goal)
        if not self._wait_future(send_future, 10.0):
            return False, math.inf, "goal request timed out"
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False, math.inf, "goal rejected"

        result_future = handle.get_result_async()
        if not self._wait_future(result_future, timeout):
            handle.cancel_goal_async()
            return False, math.inf, "navigation timed out"
        wrapped = result_future.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            status = "none" if wrapped is None else str(wrapped.status)
            return False, math.inf, f"navigation ended with status {status}"
        if self.truth_position is None:
            return False, math.inf, "ground-truth odometry unavailable"
        error = math.hypot(
            self.truth_position[0] - x_value,
            self.truth_position[1] - y_value,
        )
        return True, error, "succeeded"


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal-timeout", type=float, default=120.0)
    parser.add_argument("--position-limit", type=float, default=0.25)
    parsed = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init(args=args)
    node = NavigationAcceptance()
    failures = []
    try:
        if not node.client.wait_for_server(timeout_sec=30.0):
            print("FAIL: /navigate_to_pose action is unavailable", file=sys.stderr)
            raise SystemExit(1)
        origin_deadline = time.monotonic() + 10.0
        while node.truth_position is None and time.monotonic() < origin_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.truth_position is None:
            print("FAIL: /ground_truth/odom is unavailable", file=sys.stderr)
            raise SystemExit(1)

        for index, (x_value, y_value, yaw) in enumerate(DEFAULT_GOALS, start=1):
            success, error, reason = node.run_goal(
                x_value, y_value, yaw, parsed.goal_timeout
            )
            print(
                f"goal {index}/5: target=({x_value:.2f}, {y_value:.2f}) "
                f"result={reason} truth_error={error:.3f}m"
            )
            if not success or error > parsed.position_limit:
                failures.append((index, reason, error))
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if failures:
        index, reason, error = failures[0]
        print(
            f"FAIL: goal {index} failed ({reason}, error={error:.3f}m)",
            file=sys.stderr,
        )
        raise SystemExit(1)
    print("PASS: navigation success 5/5 with truth error within limit")


if __name__ == "__main__":
    main()
