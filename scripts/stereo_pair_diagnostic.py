#!/usr/bin/env python3
"""Inspect one timestamp-matched stereo pair without using odometry or truth."""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class StereoPairDiagnostic(Node):
    def __init__(self):
        super().__init__("stereo_pair_diagnostic")
        self.left = {}
        self.right = {}
        self.first_stamp = None
        self.first_left = None
        self.stereo_result = None
        self.result = None
        self.create_subscription(
            Image, "/stereo/left/image_raw", self._left, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/stereo/right/image_raw", self._right, qos_profile_sensor_data
        )

    @staticmethod
    def _stamp(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    @staticmethod
    def _array(message):
        data = np.frombuffer(message.data, dtype=np.uint8)
        return data.reshape(message.height, message.step)[:, : message.width].copy()

    def _left(self, message):
        self.left[self._stamp(message)] = message
        self._match()

    def _right(self, message):
        self.right[self._stamp(message)] = message
        self._match()

    def _match(self):
        common = self.left.keys() & self.right.keys()
        if not common or self.result is not None:
            self._trim()
            return
        stamp = min(common)
        left = self._array(self.left.pop(stamp))
        right = self._array(self.right.pop(stamp))
        if self.first_stamp is None:
            self.first_stamp = stamp
            self.first_left = left
            self.stereo_result = self._analyze_stereo(left, right)
        elif stamp - self.first_stamp >= 1_000_000_000:
            self.result = {
                **self.stereo_result,
                **self._analyze_temporal(self.first_left, left),
            }

    def _trim(self):
        for messages in (self.left, self.right):
            while len(messages) > 10:
                del messages[min(messages)]

    @staticmethod
    def _analyze_stereo(left, right):
        orb = cv2.ORB_create(nfeatures=1000)
        left_keypoints, left_descriptors = orb.detectAndCompute(left, None)
        right_keypoints, right_descriptors = orb.detectAndCompute(right, None)
        good = []
        if left_descriptors is not None and right_descriptors is not None:
            matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
            for candidates in matcher.knnMatch(left_descriptors, right_descriptors, k=2):
                if len(candidates) == 2 and candidates[0].distance < 0.75 * candidates[1].distance:
                    good.append(candidates[0])
        disparities = np.array(
            [left_keypoints[m.queryIdx].pt[0] - right_keypoints[m.trainIdx].pt[0] for m in good]
        )
        vertical = np.array(
            [abs(left_keypoints[m.queryIdx].pt[1] - right_keypoints[m.trainIdx].pt[1]) for m in good]
        )
        epipolar = vertical <= 1.5 if len(vertical) else np.array([], dtype=bool)
        return {
            "left_mean": float(left.mean()),
            "left_std": float(left.std()),
            "right_mean": float(right.mean()),
            "right_std": float(right.std()),
            "left_keypoints": len(left_keypoints),
            "right_keypoints": len(right_keypoints),
            "ratio_matches": len(good),
            "epipolar_matches": int(epipolar.sum()),
            "positive_disparity": int(((disparities > 0.0) & epipolar).sum()),
            "median_disparity": float(np.median(disparities[epipolar])) if epipolar.any() else float("nan"),
            "vertical_p95": float(np.percentile(vertical, 95)) if len(vertical) else float("inf"),
        }

    @staticmethod
    def _analyze_temporal(first, second):
        orb = cv2.ORB_create(nfeatures=1000)
        first_keypoints, first_descriptors = orb.detectAndCompute(first, None)
        second_keypoints, second_descriptors = orb.detectAndCompute(second, None)
        good = []
        if first_descriptors is not None and second_descriptors is not None:
            matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
            for candidates in matcher.knnMatch(first_descriptors, second_descriptors, k=2):
                if len(candidates) == 2 and candidates[0].distance < 0.75 * candidates[1].distance:
                    good.append(candidates[0])
        displacement = np.array(
            [
                np.hypot(
                    first_keypoints[m.queryIdx].pt[0] - second_keypoints[m.trainIdx].pt[0],
                    first_keypoints[m.queryIdx].pt[1] - second_keypoints[m.trainIdx].pt[1],
                )
                for m in good
            ]
        )
        return {
            "temporal_matches": len(good),
            "temporal_median_px": float(np.median(displacement)) if len(displacement) else float("inf"),
            "temporal_p95_px": float(np.percentile(displacement, 95)) if len(displacement) else float("inf"),
        }


def main():
    rclpy.init()
    node = StereoPairDiagnostic()
    deadline = time.monotonic() + 10.0
    try:
        while rclpy.ok() and node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.result is None:
            raise SystemExit("No timestamp-matched stereo pair received within 10 seconds")
        print(" ".join(f"{key}={value}" for key, value in node.result.items()))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
