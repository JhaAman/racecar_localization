#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import std_msgs
import math

PID_KP_LEFT =0.5
PID_KP_RIGHT = 0.8

PID_KD = .05

class wall_follow:
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=10)
        self.publisher = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)

	self.drive_msg = AckermannDriveStamped()
        self.last_error = None

        self.follow_left = False

        self.desired = 0.4
    	self.publish()
    def calc_actual_dist(self, ranges):
        if self.follow_left:
            end_index = 900
            start_index= 850
        else: # follow right
            end_index = 1081 - 850
            start_index = 1081 - 900

        angle_degrees = (270.0 / 1081.0) * (end_index - start_index)
        r1 = ranges[start_index] # looking forward-left
        r2 = ranges[end_index] # looking left

        dist = (r1 * r2 * math.sin(math.radians(angle_degrees)))
        dist /= math.sqrt(r1**2 + r2**2 - 2*r1*r2*math.cos(math.radians(angle_degrees)))
        return dist
        
        
    def laser_callback(self, msg):
        self.simulate_callback(msg, self.publisher)
        
    def simulate_callback(self, msg, publisher):
        actual_dist = self.calc_actual_dist(msg.ranges)
        # rospy.loginfo("The actual distance: %f", actual_dist)
        # rospy.loginfo("Direction: %d", self.follow_left)


        error = self.desired - actual_dist
        
        if self.last_error != None:
            deriv = (error - self.last_error) / msg.scan_time
        else:
            deriv = 0

        sign = 1
        if self.follow_left:
            sign = -1

        if self.follow_left:
            pid_kp = PID_KP_LEFT
        else:
            pid_kp = PID_KP_RIGHT
        steer_output = (sign * pid_kp * error) + (sign * PID_KD * deriv)

        if steer_output > 0.4:
            steer_output = 0.4
        elif steer_output < -0.4:
            steer_output = -0.4

        # rospy.loginfo("steering is %f", steer_output)
    
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.drive.speed = 1 # max speed
        self.drive_msg.drive.steering_angle = steer_output
        
	
        self.last_error = error
    def publish(self):
	while not rospy.is_shutdown():
		self.publisher.publish(self.drive_msg)
		rospy.Rate(8).sleep()
if __name__ == '__main__':
    rospy.init_node('wall_follower')
    node = wall_follow()
    rospy.spin()            
