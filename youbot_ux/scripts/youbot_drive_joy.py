#!/usr/bin/env python 


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import time


class YoubotDrive:
	def __init__(self):
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.state_sub = rospy.Subscriber(rospy.get_param('~state_sub_topic', "state"), String, self.getState)
		self.joy_sub = rospy.Subscriber(rospy.get_param('~/control_options/controls/youbot_drive/controlsSubTopic', "joy"), Joy, self.publish_cmd_velocities)
		self.stateMessage = "safemode"
		
		self.axisSpeedMultiplier = rospy.get_param('~/control_options/controls/youbot_drive/axisSpeedMultiplier', 0.5)
		# Give the publishers time to get setup before trying to do any actual work.
		rospy.on_shutdown(self.publish_cmd_velocities_stop)
        	rospy.sleep(2)
	
	def getState(self, string):
		self.stateMessage = string.data
		#print(self.stateMessage)

	def publish_cmd_velocities(self, joy):
		if self.stateMessage == "driving":
			vel_cmd = Twist()
			vel_cmd.linear.x = joy.axes[rospy.get_param('~/control_options/controls/youbot_drive/xAxes', 1)]*self.axisSpeedMultiplier
			vel_cmd.linear.y = joy.axes[rospy.get_param('~/control_options/controls/youbot_drive/yAxes', 0)]*self.axisSpeedMultiplier
			vel_cmd.linear.z = 0
			vel_cmd.angular.x = 0
			vel_cmd.angular.y = 0
			vel_cmd.angular.z = joy.axes[rospy.get_param('~/control_options/controls/youbot_drive/zAxes', 2)]*self.axisSpeedMultiplier
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
