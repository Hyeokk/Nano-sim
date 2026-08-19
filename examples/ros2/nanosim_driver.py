#!/usr/bin/env python3
"""Nano-sim ROS 2 vehicle-control example.

Publishes geometry_msgs/Twist on /cmd_vel. rosbridge relays it to Unity, which
maps linear.x -> forward speed (m/s) and angular.z -> yaw rate (rad/s, + = left).

Publish faster than 2 Hz: Unity stops the car after 0.5 s of silence.
Make sure /cmd_vel is ENABLED in the Unity topic config and that Unity is
connected to this machine's rosbridge (ws://<host>:9090).
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class NanoSimDriver(Node):
    def __init__(self):
        super().__init__("nanosim_driver")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.tick)   # 20 Hz
        self.get_logger().info("Driving Nano-sim over /cmd_vel. Ctrl-C to stop.")

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        cmd = Twist()
        cmd.linear.x = 0.6                       # 0.6 m/s forward
        cmd.angular.z = 0.5 * math.sin(t * 0.5)  # gentle slalom, rad/s (+ = left)
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = NanoSimDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
