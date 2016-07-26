#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import std_msgs
import math
from PID import PIDController
#MEMES BRO
PID_KP = 0.0045
PID_KP_countclock = 0.0035
PID_KI = 0.0
PID_KD = 0.0
SLICE_LEN = 50
class twoWallFollow:
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=10)
        self.publisher = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()
        self.pid = PIDController(rospy.Time.now(),  PID_KP,PID_KI,PID_KD)
        self.publish()

    def laser_callback(self, msg):
	ranges = [1 if msg.ranges[i] > 1 else 0 for i in range(1081)]
	rangeStarted = False
	currStart = None
	currEnd = None
	currLen = None
	maxLen = None
	maxEnd = None
	maxStart = None
	for i in range(len(ranges)):
	    if ranges[i]:
		if rangeStarted:
			pass
		else:
			currStart = i
			rangeStarted = True
	    else:
		if rangeStarted:
			currEnd = i
			currLen = currEnd - currStart
			if currLen > maxLen:
				maxLen = currLen
				maxStart = currStart
				maxEnd = currEnd
			rangeStarted = False
			
		 	
	

        targetPoint = (maxEnd - maxStart)/2
        print targetPoint
        error = (targetPoint - 540)/4
        steer_angle = self.pid.update(error,  rospy.Time.now())
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.drive.speed = 2.0
        self.drive_msg.drive.steering_angle = steer_angle

    def publish(self):
        while not rospy.is_shutdown():
            self.publisher.publish(self.drive_msg)
            rospy.Rate(8).sleep()

if __name__ == "__main__":
    rospy.init_node("TwoWallFollow")
    e = twoWallFollow()
    rospy.spin()
