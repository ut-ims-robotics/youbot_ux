#!/usr/bin/env python 


import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Empty
from std_srvs.srv import Empty
import time


class TrajectoryRecordControl:
	def __init__(self):
		self.trajectory_record_state_pub = rospy.Publisher('trajectory_record_state', String, queue_size=1)
		self.state_sub = rospy.Subscriber(rospy.get_param('~sub_topic', "state"), String, self.getState)
		self.joy_sub = rospy.Subscriber(rospy.get_param('~sub_topic', "joy"), Joy, self.publish_cmd_velocities)
		self.trajectory_record_state = "idle"
		self.stateMessage = "safemode"

		self.motorsOff = rospy.ServiceProxy('arm_1/switchOffMotors', Empty)
		self.motorsOn = rospy.ServiceProxy('arm_1/switchOnMotors', Empty) # how to get empty messages in rospy GLHF
		
		self.trajectoryRecordControlStart = rospy.ServiceProxy('control_trajectory_recorder', "recorder_action: 0")
		self.trajectoryRecordControlStop = rospy.ServiceProxy('control_trajectory_recorder', "recorder_action: 1")
		#self.trajectoryRecordGetRecorded = rospy.ServiceProxy('arm_1/switchOnMotors', "{}")
		
		# Give the publishers time to get setup before trying to do any actual work.
        	rospy.sleep(2)
	
	def getState(self, string):
		self.stateMessage = string.data
		#print(self.stateMessage)

	def publish_cmd_velocities(self, joy):
		if self.stateMessage == "trajectoryRecord":
			if self.trajectory_record_state == "idle":
				if joy.buttons[0]:
					self.trajectory_record_state = "record"
					self.trajectory_record_state_pub("record")
					self.trajectoryRecordControlStart()
				elif joy.buttons[1]:
					self.trajectory_record_state = "playback"
					self.trajectory_record_state_pub("playback")
			elif self.trajectory_record_state == "record":
				if joy.buttons[0]:
					self.trajectory_record_state = "idle"
					self.trajectory_record_state_pub("idle")
					self.trajectoryRecordControlStop()
				if joy.buttons[3]:
					self.motorsOff()
				else:
					self.motorsOn()
			elif self.trajectory_record_state == "playback":
				print("playback")
				
		
	def publish_cmd_velocities_stop(self):
		self.cmd_vel_pub.publish(vel_cmd)

	def main(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.publish_arm_joint_velocities()
			rate.sleep()
		

rospy.init_node('trajectory_record_control', anonymous=True)
trajectoryRecordControl = TrajectoryRecordControl()
trajectoryRecordControl.main()
