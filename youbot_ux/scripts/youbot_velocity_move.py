#!/usr/bin/env python 


import rospy

from sensor_msgs.msg import JointState
from brics_actuator.msg import JointVelocities
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Int8

# For sending youBot's manipulator speeds at which to move its joints

class YoubotArm:
	def __init__(self):
		self.armJointPub = rospy.Publisher("arm_1/arm_controller/velocity_command", JointVelocities, queue_size=0)
		self.armJointSelectPub = rospy.Publisher('joint_select', Int8, queue_size=1) # Publish currently selected joint for feedback vibration on selection
		self.stateSub = rospy.Subscriber(rospy.get_param('~state_sub_topic', "state"), String, self.getState)
		self.joySub = rospy.Subscriber(rospy.get_param('~/control_options/controls/youbot_velocity_move/controlsSubTopic', "joy"), Joy, self.subJoyStates)
		self.stateMessage = "safemode"

		self.axisSpeedMultiplier = rospy.get_param('~/control_options/controls/youbot_velocity_move/axisSpeedMultiplier', 0.5)
		self.prevClick = 0
		self.currentSelected = 1
		self.prevSelected = 0 
		self.jointSelectorUp = 0
		self.jointSelectorDown = 0

		self.jointAxisButtonX1 = 0.0
		self.jointAxisButtonX2 = 0.0
		self.jointAxisButtonY1 = 0.0
		self.jointAxisButtonY2 = 0.0
		

		self.rate = rospy.Rate(10)

		rospy.on_shutdown(self.publishArmJointVelocitiesStop) # On shutdown, last things to be sent are "0" speed values

	def getState(self, string): # get current regime
		self.stateMessage = string.data

	def subJoyStates(self, joy): # get controller input values
		self.jointSelectorUp = joy.buttons[rospy.get_param('~/control_options/controls/youbot_velocity_move/buttonSelectorUp', 15)]
		self.jointSelectorDown = joy.buttons[rospy.get_param('~/control_options/controls/youbot_velocity_move/buttonSelectorDown', 17)]
		self.jointAxisButtonX1 = -joy.axes[rospy.get_param('~/control_options/controls/youbot_velocity_move/axesButtonX', 0)]
		self.jointAxisButtonX2 = joy.axes[rospy.get_param('~/control_options/controls/youbot_velocity_move/axesButtonXRotate', 2)]
		self.jointAxisButtonY1 = joy.axes[rospy.get_param('~/control_options/controls/youbot_velocity_move/axesButtonY1', 1)]
		self.jointAxisButtonY2 = joy.axes[rospy.get_param('~/control_options/controls/youbot_velocity_move/axesButtonY2', 3)]

	def publishArmJointVelocities(self):
		if self.stateMessage == "manipulatorPerJoint":

			# block to look for joint selector button presses and switching of controllable joint
			if self.prevClick == 0 and (self.jointSelectorUp == 1 or self.jointSelectorDown == 1):
				if self.jointSelectorUp:
					self.currentSelected += 1
					if self.currentSelected > 2:
						self.currentSelected = 2
				if self.jointSelectorDown:
					self.currentSelected -= 1
					if self.currentSelected < 1:
						self.currentSelected = 1
				self.prevClick = 1

			if self.jointSelectorUp == 0 and self.jointSelectorDown == 0:
				self.prevClick = 0

			# Block to set joint velocity values according to controller inputs
			jointVelocities = [0.0, 0.0, 0.0, 0.0, 0.0] 
			jointVelocities[self.currentSelected] = self.axisSpeedMultiplier*(self.jointAxisButtonY1)
			jointVelocities[0] = self.axisSpeedMultiplier*(self.jointAxisButtonX1)
			jointVelocities[3] = self.axisSpeedMultiplier*(self.jointAxisButtonY2)
			jointVelocities[4] = self.axisSpeedMultiplier*(self.jointAxisButtonX2)

			jointCommands = []

			for i in range(5): # setting each joint with new velocity
				joints = JointValue()
				joints.joint_uri = "arm_joint_" + str(i+1)
				joints.unit = "s^-1 rad"
				joints.value = jointVelocities[i]

				joints.timeStamp = rospy.Time.now()
				jointCommands.append(joints)

			desiredVelocities = JointVelocities()
			desiredVelocities.velocities = jointCommands
			self.armJointPub.publish(desiredVelocities)

			if self.prevSelected != self.currentSelected: # If previously selected joint was different from current, publish new for controller feedback
				self.armJointSelectPub.publish(self.currentSelected)
				self.prevSelected = self.currentSelected

	def publishArmJointVelocitiesStop(self): # function for stopping all joints on node shutdown 

		jointVelocities = [0.0, 0.0, 0.0, 0.0, 0.0]
		jointCommands = []

		for i in range(5):
			joints = JointValue()
			joints.joint_uri = "arm_joint_" + str(i+1)
			joints.unit = "s^-1 rad"
			joints.value = jointVelocities[i]

			joints.timeStamp = rospy.Time.now()
			jointCommands.append(joints)

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
