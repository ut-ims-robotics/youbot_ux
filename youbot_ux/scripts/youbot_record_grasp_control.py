#!/usr/bin/env python 


import rospy

from sensor_msgs.msg import JointState
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class YoubotArm:

	def __init__(self):
		self.gripperPub = rospy.Publisher("arm_1/gripper_controller/position_command", JointPositions, queue_size=1)
		self.stateSub = rospy.Subscriber(rospy.get_param('~state_sub_topic', "state"), String, self.getState)
		self.joySub = rospy.Subscriber(rospy.get_param('~/control_options/controls/youbot_record_grasp_control/controlsSubTopic', "joy"), Joy, self.getJoy)
		self.trajectoryRecordStateSub = rospy.Subscriber(rospy.get_param('~trajectory_record_state_topic', "trajectory_record_state"), String, self.getTrajectoryRecordState)		

		self.stateMessage = "safemode"
		self.trajectoryRecordState = None
		self.prevClick = 0
		self.currentSelected = 0
		
		self.gripperWidthOpen = 0.0099 # value set to check if gripper is open
		self.gripperCurrent = 0.0 # init of value for current gripper state (usual after default calibration)
		self.gripperLast = self.gripperCurrent
		#self.amountOfChange = 0.0 # init of value for amount of change according to user input

		# init of gripper change multiplier and controls 
		self.axisSpeedMultiplier = rospy.get_param('~/control_options/controls/youbot_velocity_move_grasp/axisSpeedMultiplier', 0.00025)
		self.axisGripperClose = 0
		self.axisGripperOpen = 0
		
		self.rate = rospy.Rate(10)


	def getTrajectoryRecordState(self, state):
		self.trajectoryRecordState = state.data

	def getState(self, string):
		self.stateMessage = string.data
		#print(self.stateMessage)
	
	# function for getting and saving controller values
	def getJoy(self, joy):
		self.axisGripperClose = joy.axes[rospy.get_param('~/control_options/controls/youbot_record_grasp_control/axisGripperClose', 4)]
		self.axisGripperOpen = joy.axes[rospy.get_param('~/control_options/controls/youbot_record_grasp_control/axisGripperOpen', 5)]

	def publishGripperJointPositions(self):
		rospy.logdebug(self.trajectoryRecordState)
		if self.stateMessage == "trajectoryRecord" and self.trajectoryRecordState != "playback":

			self.gripperCurrent += self.axisSpeedMultiplier*(-1*self.axisGripperClose+self.axisGripperOpen)
			if self.gripperLast != self.gripperCurrent:
				# check for gripper limit values
				if self.gripperCurrent > self.gripperWidthOpen:
					self.gripperCurrent = self.gripperWidthOpen
				if self.gripperCurrent < 0.0:
					self.gripperCurrent = 0.0

				desiredPositions = JointPositions() # start of setting new positions for gripper
				jointCommands = []

				joint = JointValue() # left gripper
				joint.joint_uri = "gripper_finger_joint_l"
				joint.unit = "m"
				joint.value = self.gripperCurrent
				jointCommands.append(joint)

				joint = JointValue() # right gripper
				joint.joint_uri = "gripper_finger_joint_r"
				joint.unit = "m"
				joint.value = self.gripperCurrent
				jointCommands.append(joint)

				desiredPositions.positions = jointCommands

				self.gripperPub.publish(desiredPositions)	

	def main(self):
		while not rospy.is_shutdown():
			rate = rospy.Rate(10)
			self.publishGripperJointPositions()
			self.rate.sleep()


#if __name__ == '__main__':
rospy.init_node('youbot_record_grasp_control', anonymous=True)
velocityControl = YoubotArm()
velocityControl.main()

