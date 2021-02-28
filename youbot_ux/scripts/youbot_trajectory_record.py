#!/usr/bin/env python 


import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Empty
from std_srvs.srv import Empty
from trajectory_replayer.srv import *
from trajectory_recorder.srv import *

import time

# todo get services functioning (also in trajectory_replayer) ...still not working well done you...
# finish trajectory_record_state condition ...cant test without previous task...
# figure out engine on off service ...cant test due to gazebo option not having such services...
# variable names recheck in all nodes
# ...

class TrajectoryRecordControl:
	def __init__(self):
		self.trajectory_record_state_pub = rospy.Publisher('trajectory_record_state', String, queue_size=1)
		self.state_sub = rospy.Subscriber(rospy.get_param('~sub_topic', "state"), String, self.getState)
		self.joy_sub = rospy.Subscriber(rospy.get_param('~/control_options/controls/youbot_trajectory_record/controlsSubTopic', "joy"), Joy, self.sub_joy_states)
		self.trajectory_record_state = "idle"
		self.stateMessage = "safemode"

		self.motorsOff = rospy.ServiceProxy('arm_1/switchOffMotors', Empty)
		self.motorsOn = rospy.ServiceProxy('arm_1/switchOnMotors', Empty) 
		
		self.trajectoryRecordSrv = rospy.ServiceProxy('control_trajectory_recorder', TrajectoryRecorderControl)
		self.trajectoryReplaySrv = rospy.ServiceProxy('trajectory_playback', SendHackedTrajectory)
		#self.trajectoryRecordGetRecorded = rospy.ServiceProxy('arm_1/switchOnMotors', "{}")
		
		self.recordControlButtonState = 0
		self.playbackControlButtonState = 0
		self.motorControlButtonState = 0

		# Give the publishers time to get setup before trying to do any actual work.
        	rospy.sleep(2)
	
	def getState(self, string):
		self.stateMessage = string.data

        def sub_joy_states(self, joy):
		self.recordControlButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_trajectory_record/recordControlButton', 0)]
		self.playbackControlButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_trajectory_record/playbackControlButton', 1)]
		self.motorControlButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_trajectory_record/motorControlButton', 3)]

	def trajectoryRecordControl(self):
		self.trajectory_record_state_pub.publish(self.trajectory_record_state)
		rospy.loginfo(self.trajectory_record_state)
		rospy.loginfo(self.stateMessage)
		if self.stateMessage == "trajectoryRecord":

			if self.trajectory_record_state == "idle":
				if self.recordControlButtonState:
					self.trajectory_record_state = "record"
					self.trajectoryRecordSrv(0)
				elif self.playbackControlButtonState:
					self.trajectory_record_state = "playback"

			elif self.trajectory_record_state == "record":
				if self.recordControlButtonState:
					self.trajectory_record_state = "idle"
					self.trajectoryRecordSrv(1)

				if self.motorControlButtonState:
					#self.motorsOff()
					pass
				else:
					#self.motorsOn()
					pass
					

			elif self.trajectory_record_state == "playback":
				print("playback")
				resp = self.trajectoryReplaySrv()
				while resp != "sendHackedTrajectory: Finished":
					sleep(1)
				self.trajectory_record_state = "idle"

	def main(self):
		rate = rospy.Rate(10)
		rospy.wait_for_service('control_trajectory_recorder')
		rospy.loginfo("control_trajectory_recorder found")
		rospy.wait_for_service('get_recorded_trajectory')
		rospy.loginfo("get_recorded_trajectory found")
		rospy.wait_for_service('trajectory_playback')
		rospy.loginfo("trajectory_playback found")
		while not rospy.is_shutdown():
			self.trajectoryRecordControl()
			rate.sleep()
		

rospy.init_node('trajectory_record_control', anonymous=True)
trajectoryRecord = TrajectoryRecordControl()
trajectoryRecord.main()
