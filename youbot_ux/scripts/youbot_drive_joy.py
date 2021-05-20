#!/usr/bin/env python 


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import time

# For controlling the youBot base

class YoubotDrive:
	def __init__(self):
		self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1) # Send base speed values
		self.stateSub = rospy.Subscriber("state", String, self.getState)  # Receive current regime 
		self.joySub = rospy.Subscriber(rospy.get_param('~/control_options/controls/youbot_drive/controlsSubTopic', "joy"), Joy, self.subJoyStates) # Get controller input
		self.stateMessage = "safemode" # default known regime
		
		self.axisSpeedMultiplier = rospy.get_param('~/control_options/controls/youbot_drive/axisSpeedMultiplier', 0.5)
		self.xAxis = 0.0
		self.yAxis = 0.0
		self.zAxis = 0.0

		self.rate = rospy.Rate(20)
		rospy.on_shutdown(self.publishCmdVelocitiesStop) # On node shutdown, send "0" values to stop the robot
	
	def getState(self, string):
		self.stateMessage = string.data

	def subJoyStates(self, joy):
		self.xAxis = joy.axes[rospy.get_param('~/control_options/controls/youbot_drive/xAxes', 1)]
		self.yAxis = joy.axes[rospy.get_param('~/control_options/controls/youbot_drive/yAxes', 0)]
		self.zAxis = joy.axes[rospy.get_param('~/control_options/controls/youbot_drive/zAxes', 2)]

	def publishCmdVelocities(self):
		if self.stateMessage == "driving":
			velCmd = Twist()
			velCmd.linear.x = self.xAxis*self.axisSpeedMultiplier # Axis values 0.0 to 1.0 multiplied by preset value to decrease maximum speed
			velCmd.linear.y = self.yAxis*self.axisSpeedMultiplier
			velCmd.linear.z = 0
			velCmd.angular.x = 0
			velCmd.angular.y = 0
			velCmd.angular.z = self.zAxis*self.axisSpeedMultiplier
			self.cmdVelPub.publish(velCmd)
		
	def publishCmdVelocitiesStop(self):
		velCmd = Twist()
		velCmd.linear.x = 0
		velCmd.linear.y = 0
		velCmd.linear.z = 0
		velCmd.angular.x = 0
		velCmd.angular.y = 0
		velCmd.angular.z = 0
		self.cmdVelPub.publish(velCmd)

	def main(self):
		while not rospy.is_shutdown():
			self.publishCmdVelocities()
			self.rate.sleep()
		

rospy.init_node('youbot_drive_joy', anonymous=True)
velocityControl = YoubotDrive()
velocityControl.main()
