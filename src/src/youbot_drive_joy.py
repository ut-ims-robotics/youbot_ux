#!/usr/bin/env python 


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import time


class YoubotDrive:
	def __init__(self):
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.state_sub = rospy.Subscriber(rospy.get_param('~sub_topic', "state"), String, self.getState)
		self.joy_sub = rospy.Subscriber(rospy.get_param('~sub_topic', "joy"), Joy, self.publish_cmd_velocities)
		self.stateMessage = "safemode"
		
		# Give the publishers time to get setup before trying to do any actual work.
        	rospy.sleep(2)
	
	def getState(self, string):
		self.stateMessage = string.data
		print(self.stateMessage)

	def publish_cmd_velocities(self, joy):
		if self.stateMessage == "driving":
			vel_cmd = Twist()
			vel_cmd.linear.x = joy.axes[1]
			vel_cmd.linear.y = joy.axes[0]
			vel_cmd.linear.z = 0
			vel_cmd.angular.x = 0
			vel_cmd.angular.y = 0
			vel_cmd.angular.z = joy.axes[2]
			self.cmd_vel_pub.publish(vel_cmd)
		else:
			self.publish_cmd_velocities_stop()
		
	def publish_cmd_velocities_stop(self):
		vel_cmd = Twist()
		vel_cmd.linear.x = 0
		vel_cmd.linear.y = 0
		vel_cmd.linear.z = 0
		vel_cmd.angular.x = 0
		vel_cmd.angular.y = 0
		vel_cmd.angular.z = 0
		self.cmd_vel_pub.publish(vel_cmd)
		
	

	def main(self):
		
		rospy.spin()
		

rospy.init_node('youbot_drive_joy', anonymous=True)
velocityControl = YoubotDrive()
velocityControl.main()
