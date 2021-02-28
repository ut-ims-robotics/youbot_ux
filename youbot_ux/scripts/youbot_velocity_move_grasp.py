#!/usr/bin/env python 


import rospy

from sensor_msgs.msg import JointState
from brics_actuator.msg import JointVelocities
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
from sensor_msgs.msg import Joy
from std_msgs.msg import String
        
gripperWidthOpen = 0.0099

class YoubotArm:

	def __init__(self):
		self.gripper_pub = rospy.Publisher("arm_1/gripper_controller/position_command", JointPositions, queue_size=0)
		self.state_sub = rospy.Subscriber(rospy.get_param('~state_sub_topic', "state"), String, self.getState)
		self.joy_sub = rospy.Subscriber(rospy.get_param('~/control_options/controls/youbot_velocity_move_grasp/controlsSubTopic', "joy") Joy, self.getJoy)		

		self.stateMessage = "safemode"
		self.prevClick = 0
		self.currentSelected = 0

		self.gripperCurrent = 0.0
		self.amountOfChange = 0.0

		# Give the publishers time to get setup before trying to do any actual work.
		rospy.sleep(2)

	def getState(self, string):
		self.stateMessage = string.data
		print(self.stateMessage)
	
	def getJoy(self, joy):
		axisGripperClose =joy.axes[rospy.get_param('~/control_options/controls/youbot_velocity_move_grasp/axisGripperClose', 4)]
		axisGripperOpen = joy.axes[rospy.get_param('~/control_options/controls/youbot_velocity_move_grasp/axisGripperOpen', 5)]
		axisSpeedMultiplier = rospy.get_param('~/control_options/controls/youbot_velocity_move_grasp/axisSpeedMultiplier', 0.00025)
		self.amountOfChange = axisSpeedMultiplier*(-1*axisGripperClose+axisGripperOpen)

	def publish_gripper_joint_positions(self):
		if self.stateMessage == "manipulatorPerJoint":

			self.gripperCurrent += self.amountOfChange

			if self.gripperCurrent > gripperWidthOpen:
				self.gripperCurrent = gripperWidthOpen
			if self.gripperCurrent < 0.0:
				self.gripperCurrent = 0.0

			desiredPositions = JointPositions()
			jointCommands = []

			joint = JointValue()
			joint.joint_uri = "gripper_finger_joint_l"
			joint.unit = "m"
			joint.value = self.gripperCurrent
			jointCommands.append(joint)

			joint = JointValue()
			joint.joint_uri = "gripper_finger_joint_r"
			joint.unit = "m"
			joint.value = self.gripperCurrent
			jointCommands.append(joint)

			desiredPositions.positions = jointCommands

			self.gripper_pub.publish(desiredPositions)

	def main(self):
		while not rospy.is_shutdown():
			rate = rospy.Rate(10)
			self.publish_gripper_joint_positions()
			rate.sleep()


#if __name__ == '__main__':
rospy.init_node('velocity_move_youbot_gripper', anonymous=True)
velocityControl = YoubotArm()
velocityControl.main()

