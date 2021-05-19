#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int8
#from sensor_msgs.msg import JoyFeedbackArray, JoyFeedback
from ds4_driver.msg import Feedback, Status


class Handler(object):
	def __init__(self, feedback_topic='set_feedback'):
		
		# set up publishers/subscribers
		self.pubFeedback = rospy.Publisher(feedback_topic, Feedback, queue_size=1) # Feedback to ds4 driver
		self.stateSub = rospy.Subscriber(rospy.get_param('~state_sub_topic', "state"), String, self.getState)
		self.armJointSelectSub = rospy.Subscriber(rospy.get_param('~joint_select_sub_topic', "joint_select"), Int8, self.getJointSelect)
		self.trajectoryRecordStateSub = rospy.Subscriber(rospy.get_param('~trajectory_record_state_topic', "trajectory_record_state"), String, self.getTrajectoryRecordState)
		self.status_sub = rospy.Subscriber('status', Status, self.getStatusMsg, queue_size=1)

		# set up initial values
		self.stateMessage = None
		self.statusMessage = None
		self.trajectoryRecordState = None
		self.ledFlash = False # to set if led flashing is to be used
		self.selectedJoint = 1 # init of per joint control join selection feedback
		self.rumbleDuration = 0.0 # init of rumble duration feedback
		self.blinkDuration = 0.5 
		self.warningBatteryLevel = 0.1 # battery level when to warn user (10%)
		self.currentTime = rospy.get_time()
		self.rumbleStartTime = rospy.get_time()
		self.blinkStartTime = rospy.get_time()

	def getStatusMsg(self, status):
		self.statusMessage = status

	def getTrajectoryRecordState(self, state):
		self.trajectoryRecordState = state.data

	def getJointSelect(self, int8): # for joint selector feedback (as rumble)
		self.selectedJoint = int8.data
		if int8.data == 1:
			self.rumbleDuration = 0.1
		if int8.data == 2:
			self.rumbleDuration = 0.3
		self.rumbleStartTime = rospy.get_time()
		print(self.selectedJoint)

	def getState(self, string):
		self.stateMessage = string.data

	def publishFeedback(self):
		rospy.loginfo(self.stateMessage)
		rospy.loginfo(self.trajectoryRecordState)
		feedback = Feedback()
		if self.stateMessage == "driving": # setting of new led colors according to regimes
			feedback.set_led = True
			feedback.led_r = 0.0
			feedback.led_g = 1.0
			feedback.led_b = 0.0			

		elif self.stateMessage == "manipulatorPerJoint":
			feedback.set_led = True
			feedback.led_r = 1.0
			feedback.led_g = 1.0
			feedback.led_b = 0.0

		elif self.stateMessage == "trajectoryRecord":
			if self.trajectoryRecordState == "record":
				feedback.set_led = True
				feedback.led_r = 1.0
				feedback.led_g = 0.0
				feedback.led_b = 0.0
			elif self.trajectoryRecordState == "idle":
				feedback.set_led = True
				feedback.led_r = 0.0
				feedback.led_g = 1.0
				feedback.led_b = 1.0
			elif self.trajectoryRecordState == "playback":
				feedback.set_led = True
				feedback.led_r = 0.5
				feedback.led_g = 0.0
				feedback.led_b = 0.6
			else:
				feedback.set_led = True
				feedback.led_r = 0.0
				feedback.led_g = 0.0
				feedback.led_b = 0.0
			
		elif self.stateMessage == "safeMode":
			feedback.set_led = True
			feedback.led_r = 0.0
			feedback.led_g = 0.0
			feedback.led_b = 1.0
		else:
			feedback.set_led = True
			feedback.led_r = 0.0
			feedback.led_g = 0.0
			feedback.led_b = 0.0

		# Block for device vibration 
		self.currentTime = rospy.get_time()
		if float(self.currentTime) - float(self.rumbleStartTime) < self.rumbleDuration:
			feedback.set_rumble = True
			feedback.rumble_small = 1.0
		
		# Notification to user about low controller battery state (led blink according to self.blinkDuration value)
		if self.statusMessage != None and self.statusMessage.battery_percentage <= self.warningBatteryLevel:
			if float(self.currentTime) - float(self.blinkStartTime) > self.blinkDuration:
				self.blinkStartTime = rospy.get_time()
				if self.ledFlash == False:
					self.ledFlash = True
				else:
					self.ledFlash = False
			#print(self.ledFlash)
			if self.ledFlash == True:
				feedback.led_r = 0.0
				feedback.led_g = 0.0
				feedback.led_b = 0.0
			else:
				pass

		self.pubFeedback.publish(feedback)			
		
	def main(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.publishFeedback()
			rate.sleep()


#if __name__ == '__main__':
rospy.init_node('youbot_states_feedback', anonymous=True)
handler = Handler()
handler.main()