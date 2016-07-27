#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import std_msgs
import math
from PID import PIDController

def __init__(self):
    rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=10)
    self.publisher = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)
    self.drive_msg = AckermannDriveStamped()
    self.publish()

def scan_callback(self, msg):
    scan_rad_angles = ( (msg.angle_increment * np.arrange(1001,dtype = float) + msg.angle_min)
    scan_x_unit_vectors = np.cos(scan_rad_angles)
    scan_y_unit_vectors = np.sin(scan_rad_angles)

    scan_x_component = (self.charge_laser_particle * scan_x_unit_vectors) / np.square(msg.ranges)
    scan_y_component = (self.charge_laser_particle * scan_y_unit_vectors) / np.square(msg.ranges)

    kick_x_component = np.ones(1) * self.charge_forward_boost/self.boost_distance**2.0
    kick_y_component = np.zeros(1)

    total_x_component = np.sum(scan_x_component) * kick_x_component
    total_y_component = np.sum(scan_y_component) * kick_y_component

    visualizer_msg = PointStamped()
    visualizer_msg.header.frame_id = 'base_link'
    visualizer_msg.point.x = total_x_component
    visualizer_msg.point.y = total_y_component

    self.pub_goal.publish(visualizer_msg)

    command_msg = AckermannDriveStamped()
    command_msg.drive.speed = np.sqrt((total_x_component)*(total_x_component)+(total_y_component)*(total_y_component)) * np.sign(total_x_component)
    command_msg.drive.steering_angle = 1 math.atan2(total_y_component, total_x_component) * np.sign(total_x_component)

    self.publisher.publish(command_msg)

if __name__ == "__main__":
    rospy.init_node("TwoWallFollow")
    e = twoWallFollow()
    rospy.spin()
