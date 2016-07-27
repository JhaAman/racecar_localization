#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import std_msgs

class safetyMech: 
    def __init__(self):	
	rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=10)
        self.publisher = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)
	rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=10)
	rospy.Subscriber('/drive', AckermannDriveStamped, self.drive_callback, queue_size = 10)
	self.obstacle = False
	self.drive_msg = AckermannDriveStamped()
	self.publish()
	
    def drive_callback(self, msg):
	if self.obstacle == False:
		self.drive_msg = AckermannDriveStamped()
		self.drive_msg = msg

    def laser_callback(self, msg):
	ranges = map(lambda x: int(x<0.2), msg.ranges)
	#print ranges
	self.obstacle = False
	for i in range(360, 711):
		#print sum(ranges[i:i+10])
		if sum(ranges[i:i+10])/10 == 1:			
			self.obstacle = True
			self.drive_msg = AckermannDriveStamped()
			print self.obstacle
    def publish(self):
	while not rospy.is_shutdown():
			self.publisher.publish(self.drive_msg)
			rospy.Rate(8).sleep()
		 
if __name__ == "__main__":
    rospy.init_node("SafetyMech")
    e = safetyMech()
    rospy.spin()

