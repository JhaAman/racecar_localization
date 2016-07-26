#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import std_msgs
import math

SLICE_LEN = 60

class objectAvoid:
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=10)
        self.publisher = rospy.Publisher('/vesc/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=10)
        self.obstFlag = False
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.drive.speed = 2.0
        self.publish()

    def publish(self):
        while not rospy.is_shutdown():
            while self.obstFlag:
                self.publisher.publish(self.drive_msg)
                rospy.Rate(8).sleep()
        
    def laser_callback(self, msg):
        averaged = [sum(msg.ranges[i:i+60])/60 for i in range(420, 660, 60)]
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.drive.speed = 2.0
        self.drive_msg.drive.steering_angle = 0.0
        
        if averaged[1] < 0.3 and averaged[2] < 0.3:
            self.obstFlag = True
        else:
            self.obstFlag = False
        if self.obstFlag and averaged[0] < 0.3:
            self.drive_msg.drive.steering_angle = 0.5
        elif self.obstFlag and averaged[3] < 0.3:
            self.drive_msg.drive.steering_angle = -0.5
        elif self.obstFlag:
            self.drive_msg.drive.speed = 0.0
        
 


if __name__ == "__main__":
    rospy.init_node("objectAvoid")
    e =objectAvoid()
    rospy.spin()
