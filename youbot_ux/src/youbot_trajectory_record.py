#!/usr/bin/env python 


import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Empty
from std_srvs.srv import Empty
from trajectory_replayer.srv import *
from trajectory_recorder.srv import *

import time

# todo get services functioning (also in trajectory_replayer)
# finish trajectory_record_state condition
# make parameter server for controls (also add reading of such parameters to gamepad inputs in nodes)
# figure out engine on off service
# variable names recheck in all nodes
# ...

class TrajectoryRecordControl:
	def __init__(self):
		self.trajectory_record_state_pub = rospy.Publisher('trajectory_record_state', String, queue_size=1)
		self.state_sub = rospy.Subscriber(rospy.get_param('~sub_topic', "state"), String, self.getState)
		self.joy_sub = rospy.Subscriber(rospy.get_param('~sub_topic', "joy"), Joy, self.sub_joy_states)
		self.trajectory_record_state = "idle"
		self.stateMessage = "safemode"

		self.motorsOff = rospy.ServiceProxy('arm_1/switchOffMotors', Empty)
		self.motorsOn = rospy.ServiceProxy('arm_1/switchOnMotors', Empty) # how to get empty messages in rospy GLHF
		
		self.trajectoryRecordControl = rospy.ServiceProxy('control_trajectory_recorder', TrajectoryRecorderControl)
		#self.trajectoryRecordGetRecorded = rospy.ServiceProxy('arm_1/switchOnMotors', "{}")
		
		self.recordControlButtonState = 0
		self.playbackControlButtonState = 0
		self.motorControlButtonState = 0

		# Give the publishers time to get setup before trying to do any actual work.
        	rospy.sleep(2)
	
	def getState(self, string):
		self.stateMessage = string.data
		#print(self.stateMessage)

        def sub_joy_states(self, joy):
		self.recordControlButtonState = joy.buttons[0]
		self.playbackControlButtonState = joy.buttons[1]
		self.motorControlButtonState = joy.buttons[3]

	def trajectoryRecordControl(self, joy):
		if self.stateMessage == "trajectoryRecord":

			if self.trajectory_record_state == "idle":
				if jself.recordControlButtonState:
					self.trajectory_record_state = "record"
					self.trajectory_record_state_pub("record")
					self.trajectoryRecordControl("recorder_action: 0")
				elif self.playbackControlButtonState:
					self.trajectory_record_state = "playback"
					self.trajectory_record_state_pub("playback")

			elif self.trajectory_record_state == "record":
				if self.recordControlButtonState:
					self.trajectory_record_state = "idle"
					self.trajectory_record_state_pub("idle")
					self.trajectoryRecordControl("recorder_action: 1")
				if self.motorControlButtonState:
					self.motorsOff()
				else:
					self.motorsOn()

			elif self.trajectory_record_state == "playback":
				print("playback")
				self.trajectory_record_state = "idle"

	def main(self):
		rate = rospy.Rate(10)
		rospy.wait_for_service('control_trajectory_recorder')
		rospy.wait_for_service('get_recorded_trajectory')
		rospy.wait_for_service('trajectory_playback')
		while not rospy.is_shutdown():
			self.trajectoryRecordControl()
			rate.sleep()
		

rospy.init_node('trajectory_record_control', anonymous=True)
trajectoryRecord = TrajectoryRecordControl()
trajectoryRecord.main()
