#!/usr/bin/env python 


import rospy

from sensor_msgs.msg import JointState
from brics_actuator.msg import JointVelocities
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Int8

class YoubotArm:
	def __init__(self):
		self.armJointPub = rospy.Publisher("arm_1/arm_controller/velocity_command", JointVelocities, queue_size=0)
		self.armJointSelectPub = rospy.Publisher('joint_select', Int8, queue_size=1)
		self.stateSub = rospy.Subscriber(rospy.get_param('~state_sub_topic', "state"), String, self.getState)
		self.joySub = rospy.Subscriber(rospy.get_param('~/control_options/controls/youbot_velocity_move/controlsSubTopic', "joy"), Joy, self.subJoyStates)
		self.stateMessage = "safemode"

		self.axisSpeedMultiplier = rospy.get_param('~/control_options/controls/youbot_velocity_move/axisSpeedMultiplier', 0.5)
		self.prevClick = 0
		self.currentSelected = 1
		self.prevSelected = 0 
		self.jointSelectorUp = 0
		self.jointSelectorDown = 0
		self.jointMoverPosition = 0.0	

		self.rate = rospy.Rate(10)

		rospy.on_shutdown(self.publishArmJointVelocitiesStop)

	def getState(self, string):
		self.stateMessage = string.data
		#print(self.stateMessage)

	def subJoyStates(self, joy):
		self.jointSelectorUp = joy.buttons[rospy.get_param('~/control_options/controls/youbot_velocity_move/buttonSelectorUp', 15)]
		self.jointSelectorDown = joy.buttons[rospy.get_param('~/control_options/controls/youbot_velocity_move/buttonSelectorDown', 17)]
		self.jointAxisButtonX1 = -joy.axes[rospy.get_param('~/control_options/controls/youbot_velocity_move/axesButtonX', 0)]
		self.jointAxisButtonX2 = joy.axes[rospy.get_param('~/control_options/controls/youbot_velocity_move/axesButtonXRotate', 2)]
		self.jointAxisButtonY = joy.axes[rospy.get_param('~/control_options/controls/youbot_velocity_move/axesButtonY', 1)]
		

	def publishArmJointVelocities(self):
		if self.stateMessage == "manipulatorPerJoint":

			# block to look for join selector button presses and switching of controllable joint
			if self.prevClick == 0 and (self.jointSelectorUp == 1 or self.jointSelectorDown == 1):
				if self.jointSelectorUp:
					self.currentSelected += 1
					if self.currentSelected > 3:
						self.currentSelected = 3
				if self.jointSelectorDown:
					self.currentSelected -= 1
					if self.currentSelected < 1:
						self.currentSelected = 1
				self.prevClick = 1

			if self.jointSelectorUp == 0 and self.jointSelectorDown == 0:
				self.prevClick = 0

			joint1 = 0.0
			joint2 = 0.0
			joint3 = 0.0
			joint4 = 0.0
			joint5 = 0.0

			jointVelocities = [joint1, joint2, joint3, joint4, joint5]
			jointVelocities[self.currentSelected] = self.axisSpeedMultiplier*(self.jointAxisButtonY)
			jointVelocities[0] = self.axisSpeedMultiplier*(self.jointAxisButtonX1)
			jointVelocities[4] = self.axisSpeedMultiplier*(self.jointAxisButtonX2)

			jointCommands = []

			for i in range(5): # setting each joint with new velocity
				joint = JointValue()
				joint.joint_uri = "arm_joint_" + str(i+1)
				joint.unit = "s^-1 rad"
				joint.value = jointVelocities[i]

				joint.timeStamp = rospy.Time.now()
				jointCommands.append(joint)
				#print(i)
				#print(jointVelocities[i])

			desiredVelocities = JointVelocities()
			desiredVelocities.velocities = jointCommands
			self.armJointPub.publish(desiredVelocities)

			if self.prevSelected != self.currentSelected:
				self.armJointSelectPub.publish(self.currentSelected)
				self.prevSelected = self.currentSelected

	def publishArmJointVelocitiesStop(self): # function for stopping all joints on node shutdown 
		joint1 = 0.0
		joint2 = 0.0
		joint3 = 0.0
		joint4 = 0.0
		joint5 = 0.0

		jointVelocities = [joint1, joint2, joint3, joint4, joint5]
		jointCommands = []

		for i in range(5):
			joint = JointValue()
			joint.joint_uri = "arm_joint_" + str(i+1)
			joint.unit = "s^-1 rad"
			joint.value = jointVelocities[i]

			joint.timeStamp = rospy.Time.now()
			jointCommands.append(joint)

		desiredVelocities = JointVelocities()
		desiredVelocities.velocities = jointCommands
		self.armJointPub.publish(desiredVelocities)
			
	def main(self):
		while not rospy.is_shutdown():
			self.publishArmJointVelocities()
			self.rate.sleep()


#if __name__ == '__main__':
rospy.init_node('youbot_velocity_move', anonymous=True)
velocityControl = YoubotArm()
velocityControl.main()
