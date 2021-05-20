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
		self.stateSub = rospy.Subscriber(rospy.get_param('~state_sub_topic', "state"), String, self.getState)
		self.joySub = rospy.Subscriber(rospy.get_param('~/control_options/controls/youbot_velocity_move_grasp/controlsSubTopic', "joy"), Joy, self.getJoy)		
		self.trajectoryRecordStateSub = rospy.Subscriber(rospy.get_param('~trajectory_record_state_topic', "trajectory_record_state"), String, self.getTrajectoryRecordState)  

		self.stateMessage = "safemode"
		self.trajectoryRecordState = None
		self.prevClick = 0
		self.currentSelected = 0
		
		self.gripperWidthOpen = 0.0099 # value set to check if gripper is open
		self.gripperCurrent = rospy.get_param("arm_1_gripper/gripper_value", 0.007) # memory and init of value for current gripper state (usual after default calibration)
		self.gripperLast = 0.1 # forcing initial alignment

		# init of gripper change multiplier and controls 
		self.axisSpeedMultiplier = rospy.get_param('~/control_options/controls/youbot_velocity_move_grasp/axisSpeedMultiplier', 0.00025)
		self.axisGripperClose = 0
		self.axisGripperOpen = 0
		
		self.rate = rospy.Rate(10)


	def getState(self, string):
		self.stateMessage = string.data
		#print(self.stateMessage)

	def getTrajectoryRecordState(self, state):
		self.trajectoryRecordState = state.data
	
	# function for getting and saving controller values
	def getJoy(self, joy):
		self.axisGripperClose = joy.axes[rospy.get_param('~/control_options/controls/youbot_velocity_move_grasp/axisGripperClose', 5)]
		self.axisGripperOpen = joy.axes[rospy.get_param('~/control_options/controls/youbot_velocity_move_grasp/axisGripperOpen', 4)]

	def publishGripperJointPositions(self):
		if self.stateMessage == "manipulatorPerJoint" or (self.stateMessage == "trajectoryRecord" and self.trajectoryRecordState != "playback"):

			self.gripperCurrent += self.axisSpeedMultiplier*(-1*self.axisGripperClose+self.axisGripperOpen)

			self.gripperLast = self.gripperCurrent

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
			self.publishGripperJointPositions()
			self.rate.sleep()
		rospy.set_param("arm_1_gripper/gripper_value", self.gripperCurrent) # remember last joint value


#if __name__ == '__main__':
rospy.init_node('youbot_velocity_move_grasp', anonymous=True)
velocityControl = YoubotArm()
velocityControl.main()

