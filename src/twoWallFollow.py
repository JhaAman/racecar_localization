#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import std_msgs
import math
from PID import PIDController

PID_KP = 0.01
SLICE_LEN = 40
class twoWallFollow:
    def __init__(self):
        rospy.Subscriber('racecar/laser/scan', LaserScan, self.laser_callback, queue_size=10)
        self.publisher = rospy.Publisher('/racecar/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()
        self.pid = PIDController(rospy.Time.now(),  PID_KP)
        self.publish()

    def laser_callback(self, msg):
        averaged = [sum(msg.ranges[i:i+40])/40 for i in range(0, 1080, 40)]
        furthest = max(enumerate(averaged),  key=lambda x: x[1])[0]
        targetPoint = furthest * 40 + 20
        
        error = (targetPoint - 540)/4
        steer_angle = self.pid.update(error,  rospy.Time.now())
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.drive.speed = 1.0
        self.drive_msg.drive.steering_angle = steer_angle

    def publish(self):
        while not rospy.is_shutdown():
            self.publisher.publish(self.drive_msg)
            rospy.Rate(8).sleep()

if __name__ == "__main__":
    rospy.init_node("TwoWallFollow")
    e = twoWallFollow()
    rospy.spin()
