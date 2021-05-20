#!/usr/bin/env python 

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Empty
from trajectory_replayer.srv import *
from trajectory_recorder.srv import *

import time

# user for controlling the trajectory_recorder and trajectory_replayer nodes

class TrajectoryRecordControl:
	def __init__(self):
		self.trajectoryRecordStatePub = rospy.Publisher('trajectory_record_state', String, queue_size=1)
		self.stateSub = rospy.Subscriber(rospy.get_param('~sub_topic', "state"), String, self.getState)
		self.joySub = rospy.Subscriber(rospy.get_param('~/control_options/controls/youbot_trajectory_record/controlsSubTopic', "joy"), Joy, self.subJoyStates)
		self.trajectoryRecordState = "idle" # local regime state (if recording, idle or playback)
		self.stateMessage = "safemode" # general regime state

		self.motorsOff = rospy.ServiceProxy('arm_1/switchOffMotors', Empty)
		self.motorsOn = rospy.ServiceProxy('arm_1/switchOnMotors', Empty) 
		
		self.trajectoryRecordSrv = rospy.ServiceProxy('control_trajectory_recorder', TrajectoryRecorderControl)
		self.trajectoryReplaySrv = rospy.ServiceProxy('trajectory_playback', SendTrajectory)
		
		self.recordControlButtonState = 0
		self.playbackControlButtonState = 0
		self.motorControlButtonState = 0
		self.motorCurrentState = 1

		self.rate = rospy.Rate(20)

		rospy.on_shutdown(self.trajectoryRecordControlStop)
	
	# get current state
	def getState(self, string):
		self.stateMessage = string.data

        def subJoyStates(self, joy):
		self.recordControlButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_trajectory_record/recordControlButton', 0)]
		self.playbackControlButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_trajectory_record/playbackControlButton', 1)]
		self.motorControlButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_trajectory_record/motorControlButton', 3)]

	def trajectoryRecordControl(self):
		self.trajectoryRecordStatePub.publish(self.trajectoryRecordState)
		rospy.loginfo(self.trajectoryRecordState)
		rospy.loginfo(self.stateMessage)
		if self.stateMessage == "trajectoryRecord":

			if self.trajectoryRecordState == "idle": 
				if self.recordControlButtonState: # button press check for going to recording state
					self.trajectoryRecordState = "record"
					self.trajectoryRecordSrv(0)
					while self.recordControlButtonState:
						self.rate.sleep()
				elif self.playbackControlButtonState: # button press check for going to playback state
					self.trajectoryRecordState = "playback"
					while self.playbackControlButtonState:
						self.rate.sleep()

			elif self.trajectoryRecordState == "record": # to wait for button press to end recording of trajectory and record state
				if self.recordControlButtonState:
					self.trajectoryRecordState = "idle"
					self.trajectoryRecordSrv(1)
					while self.recordControlButtonState:
						self.rate.sleep()

			elif self.trajectoryRecordState == "playback": # wait for trajectory playback service and go back to idle regime
				self.trajectoryReplaySrv()
				self.trajectoryRecordState = "idle"
	
			if self.motorControlButtonState and self.motorCurrentState == 1: # to turn on/off motor resistance
				self.motorsOff()
				self.motorCurrentState = 0 
				self.rate.sleep()
			elif self.motorControlButtonState == 0 and self.motorCurrentState == 0:
				self.motorsOn()
				self.motorCurrentState = 1
				self.rate.sleep()
	
	def trajectoryRecordControlStop(self): # on node close, local regime to None and publish it
		#self.motorsOn()
		self.trajectoryRecordState = "None"
		self.trajectoryRecordStatePub.publish(self.trajectoryRecordState)

	def main(self):
		rate = rospy.Rate(10)
		rospy.wait_for_service('control_trajectory_recorder') # wait for services that are used to control trajectory recording and playback
		rospy.loginfo("control_trajectory_recorder found")
		rospy.wait_for_service('get_recorded_trajectory')
		rospy.loginfo("get_recorded_trajectory found")
		rospy.wait_for_service('trajectory_playback')
		rospy.loginfo("trajectory_playback found")
		while not rospy.is_shutdown():
			self.trajectoryRecordControl()
			rate.sleep()
		

rospy.init_node('youbot_trajectory_record', anonymous=True)
trajectoryRecord = TrajectoryRecordControl()
trajectoryRecord.main()
