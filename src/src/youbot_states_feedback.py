#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int8
#from sensor_msgs.msg import JoyFeedbackArray, JoyFeedback
from ds4_driver.msg import Feedback, Status


class Handler(object):
	def __init__(self, feedback_topic='set_feedback'):
		
		self.pub_feedback = rospy.Publisher(feedback_topic, Feedback, queue_size=1)
		self.state_sub = rospy.Subscriber(rospy.get_param('~state_sub_topic', "state"), String, self.getState)
		self.arm_joint_select_sub = rospy.Subscriber(rospy.get_param('~join_select_sub_topic', "joint_select"), Int8, self.getJointSelect)
		self.status_sub = rospy.Subscriber('status', Status, self.getStatusMsg, queue_size=1)
		self.stateMessage = None
		self.statusMessage = None
		self.ledFlash = False
		self.selectedJoint = 1
		self.rumbleDuration = 0.0
		self.currentTime = rospy.get_time()
		self.rumbleStartTime = rospy.get_time()

	def getStatusMsg(self, status):
		self.statusMessage = status

	def getJointSelect(self, int8): # for joint selector feedback (as rumble)
		self.selectedJoint = int8.data
		if int8.data == 1:
			self.rumbleDuration = 0.1
		if int8.data == 2:
			self.rumbleDuration = 0.3
		if int8.data == 3:
			self.rumbleDuration = 0.5
		self.rumbleStartTime = rospy.get_time()
		print(self.selectedJoint)

	def getState(self, string):
		self.stateMessage = string.data

	def publishFeedback(self):
		feedback = Feedback()
		if self.stateMessage == "driving":
			feedback.set_led = True
			feedback.led_r = 0.0
			feedback.led_g = 1.0
			feedback.led_b = 0.0			

		elif self.stateMessage == "manipulatorPerJoint":
			feedback.set_led = True
			feedback.led_r = 1.0
			feedback.led_g = 1.0
			feedback.led_b = 0.0

		elif self.stateMessage == "safeMode":
			feedback.set_led = True
			feedback.led_r = 0.0
			feedback.led_g = 0.0
			feedback.led_b = 1.0
		
		#if self.selectedJoint == 1:
		#	feedback.rumble_duration = 0.2
		#if self.selectedJoint == 2:
		#	feedback.rumble_duration = 0.5
		#if self.selectedJoint == 3:
		#	feedback.rumble_duration = 0.8

		self.currentTime = rospy.get_time()
		if float(self.currentTime) - float(self.rumbleStartTime) < self.rumbleDuration:
			feedback.set_rumble = True
			feedback.rumble_small = 1.0
		print(self.currentTime, self.rumbleStartTime)

		# Notification to user about low controller battery state
		if self.statusMessage != None and self.statusMessage.battery_percentage < 0.2 and self.ledFlash == False:
			feedback.set_let_flash = True
			feedback.led_flash_on = 0.2
			self.ledFlash = True
		elif self.statusMessage != None and self.statusMessage.battery_percentage >= 0.2 and self.ledFlash == True:
			feedback.set_let_flash = True
			feedback.led_flash_on = 0.0
			self.ledFlash = False

		self.pub_feedback.publish(feedback)			
		
	def main(self):
		while not rospy.is_shutdown():
			rate = rospy.Rate(10)
			self.publishFeedback()
			rate.sleep()


#if __name__ == '__main__':
rospy.init_node('youbot_states_feedback', anonymous=True)
handler = Handler()
handler.main()
