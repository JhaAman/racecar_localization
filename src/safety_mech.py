#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import std_msgs

class safetyMech: 
  def __init__(self):	
    self.node_name = "SafetyMech"
    rospy.Subscriber('racecar/laser/scan', LaserScan, self.laser_callback, queue_size=10)
    self.publisher = rospy.Publisher('/racecar/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=10)
    self.unsafe = False
    self.drive_msg = AckermannDriveStamped()
    self.publish()
	
  def laser_callback(self, msg):
    #print self.unsafe
    ranges = map(lambda x: int(x<0.35), msg.ranges)
    #print ranges
    self.unsafe = False
    self.drive_msg = AckermannDriveStamped()
    #print ranges[360:711]
    for i in range(360, 711):
      #print sum(ranges[i:i+10])
      if ranges[i] == 1:
        self.unsafe = True
    if self.unsafe == True:
      self.drive_msg.drive.steering_angle = 0.0
      self.drive_msg.drive.speed = -1.0
    if self.unsafe == False:
        self.drive_msg.drive.speed = 0.0
    
  def publish(self):
    while not rospy.is_shutdown():
      	while self.unsafe:
            print self.unsafe
            self.publisher.publish(self.drive_msg)
            rospy.Rate(8).sleep()
		 
if __name__ == "__main__":
    rospy.init_node("SafetyMech")
    e = safetyMech()
    rospy.spin()
