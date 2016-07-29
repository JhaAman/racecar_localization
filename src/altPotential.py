#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import std_msgs
import math
import numpy as np

class altPotential:
    def __init__(self):
        self.node_name = "altPotential"
        rospy.Subscriber('racecar/laser/scan', LaserScan, self.scan_callback, queue_size=10)
        self.publisher = rospy.Publisher('/racecar/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()

        #Global values
        self.a = 1.0 #these are both silly arbitrary values, we can empirically weight them after testing
        self.k = 1.0
        self.obstaclePointMatchThreshold = 6
        self.obstacleDistanceThreshold = 1

        self.publish()

    def scan_callback(self, msg):
        goal_dist_x = #how far goal is in x
        goal_dist_y = #how far goal is in y

        #here, check around the car for array of close points under a certain value

        #here, check if a set of consecutive points are under a certain disance amount - group them together in a list
        obstPointList =
        #here, if the list.length is > obstaclePointMatchThreshold, then we know its an obstacle or a wall, avoid it
        if(obstPointList.length > obstaclePointMatchThreshold):
            #obstacle general distance is the average distance of the list distance
            obst_general = sum(obstPointList) / float(len(obstPointList))
        #If there are no nearby objects, all obstacle related things will be 0
        else:
            obst_general = 0

        obst_dist_x = #how far the object is in x. Use trig on general distance
        obst_dist_y = #how far the object is in y. Use trig on general distance

        #gradient descent algorithm
        des_x = k*goal_dist_x + (a*goal_dist_x)/((obst_dist_x * obst_dist_x + obst_dist_y * obst_dist_y)*(obst_dist_x * obst_dist_x + obst_dist_y * obst_dist_y))
        des_y = k*goal_dist_y + (a*goal_dist_y)/((obst_dist_x * obst_dist_x + obst_dist_y * obst_dist_y) * (obst_dist_x * obst_dist_x + obst_dist_y * obst_dist_y))

        #publishable math
        vec_ang = math.atan2(des_y, des_x)
        vec_mag = math.sqrt(des_x*des_x + des_y * des_y)

        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.drive.speed = vec_mag
        self.drive_msg.drive.steering_angle = vec_ang

    def publish(self):
        while not rospy.is_shutdown():
            self.publisher.publish(self.drive_msg)
            rospy.Rate(8).sleep()
if __name__ == "__main__":
    rospy.init_node("altPotential")
    e = simplePotential()
    rospy.spin()
