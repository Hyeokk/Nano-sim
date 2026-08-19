#!/usr/bin/env python3
"""Nano-sim ROS 1 vehicle-control example.

Publishes geometry_msgs/Twist on /cmd_vel. rosbridge relays it to Unity, which
maps linear.x -> forward speed (m/s) and angular.z -> yaw rate (rad/s, + = left).

Publish faster than 2 Hz: Unity stops the car after 0.5 s of silence.
Make sure /cmd_vel is ENABLED in the Unity topic config and that Unity is
connected to this machine's rosbridge (ws://<host>:9090).
"""
import math

import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node("nanosim_driver")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(20)   # 20 Hz, comfortably above the 0.5 s timeout

    t0 = rospy.get_time()
    rospy.loginfo("Driving Nano-sim over /cmd_vel. Ctrl-C to stop.")
    while not rospy.is_shutdown():
        t = rospy.get_time() - t0
        cmd = Twist()
        cmd.linear.x = 0.6                       # 0.6 m/s forward
        cmd.angular.z = 0.5 * math.sin(t * 0.5)  # gentle slalom, rad/s (+ = left)
        pub.publish(cmd)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
