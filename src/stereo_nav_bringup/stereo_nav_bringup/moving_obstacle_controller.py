#!/usr/bin/env python3
"""Drive the Gazebo test obstacle back and forth across a corridor."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MovingObstacleController(Node):
    def __init__(self):
        super().__init__("moving_obstacle_controller")
        self.declare_parameter("speed", 0.32)
        self.declare_parameter("half_period", 4.0)
        self.publisher = self.create_publisher(Twist, "/moving_obstacle/cmd_vel", 10)
        self.direction = 1.0
        self.last_switch = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_switch).nanoseconds / 1.0e9
        half_period = float(self.get_parameter("half_period").value)
        if elapsed >= half_period:
            self.direction *= -1.0
            self.last_switch = now

        command = Twist()
        command.linear.y = self.direction * float(self.get_parameter("speed").value)
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = MovingObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.publisher.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
