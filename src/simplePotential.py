#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import std_msgs
import math
import numpy as np
class simplePotential:
    def __init__(self):
        rospy.Subscriber('racecar/laser/scan', LaserScan, self.scan_callback, queue_size=10)
        self.publisher = rospy.Publisher('/racecar/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()
        self.charge_laser_particle = 0.00001
        self.charge_forward_boost = 50
        self.boost_distance = 1.0
        self.publish()
    
    def scan_callback(self, msg):
        scanRadAngles = (msg.angle_increment * np.arange(1080,dtype = float) + msg.angle_min)
        scanXUnitVectors = -np.cos(scanRadAngles)
        scanYUnitVectors = -np.sin(scanRadAngles)
    
        scanXComponent = (self.charge_laser_particle * scanXUnitVectors) / np.square(msg.ranges)
        scanYComponent = (self.charge_laser_particle * scanYUnitVectors) / np.square(msg.ranges)
    
        kickXComponent = np.ones(1) * self.charge_forward_boost/self.boost_distance**2.0
        kickYComponent = np.zeros(1)
    
        totalXComponent = np.sum(scanXComponent) * kickXComponent
        totalYComponent = np.sum(scanYComponent) * kickYComponent
    
        """visualizer_msg = PointStamped()
        visualizer_msg.header.frame_id = 'base_link'
        visualizer_msg.point.x = total_x_component
        visualizer_msg.point.y = total_y_component
    
        self.pub_goal.publish(visualizer_msg)"""
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.drive.speed = np.sqrt((totalXComponent)*(totalXComponent)+(totalYComponent)*(totalYComponent)) * np.sign(totalXComponent)
        self.drive_msg.drive.steering_angle = 0.1*math.atan2(totalYComponent, totalXComponent) * np.sign(totalXComponent)
        print self.drive_msg.drive.steering_angle
    
    def publish(self):
        while not rospy.is_shutdown():
            self.publisher.publish(self.drive_msg)
            rospy.Rate(8).sleep()
if __name__ == "__main__":
    rospy.init_node("TwoWallFollow")
    e = simplePotential()
    rospy.spin()
