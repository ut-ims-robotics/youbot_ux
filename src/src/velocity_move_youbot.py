#!/usr/bin/env python 


import rospy
import numpy as np
#import scipy.linalg
import math
import tf
import tf.transformations as trans

from tf.transformations import euler_from_quaternion
from tf.transformations import compose_matrix
from tf.transformations import is_same_transform
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from brics_actuator.msg import JointVelocities
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Int8

        

jointMax= [5.840139, 2.617989, -0.0157081, 3.42919, 5.641589]
jointMin= [0.01006921, 0.01006921, -5.0264, 0.0221391, 0.11062]

jointHome= [0.01007,0.01007,-0.15709,0.02214,0.1107]
jointCamera= [3.0,0.5,-0.9,0.1,3.0]
jointObject= [3.04171,0.63597,-1.017845,0.36284,2.876194]
jointGrasp = [3.04171,2.04427,-1.5189129,2.5434289757,2.8761944]
jointInitialize= [0.01007,.635971,-1.91989,1.04424,2.87619]

jointGuessForGrasp=[0.0, 0.0, 1.52, 1.84, -1.26, 2.4, 3.10]

armJointPosCandle = np.array([2.9496, 1.1344, -2.5482, 1.789, 2.9234])

class YoubotArm:
	def __init__(self):
		self.arm_joint_pub = rospy.Publisher("arm_1/arm_controller/velocity_command", JointVelocities, queue_size=0)
		self.arm_joint_select_pub = rospy.Publisher('joint_select', Int8, queue_size=1)
		self.state_sub = rospy.Subscriber(rospy.get_param('~sub_topic', "state"), String, self.getState)
		self.joy_sub = rospy.Subscriber(rospy.get_param('~sub_topic', "joy"), Joy, self.sub_joy_states)
		self.stateMessage = "safemode"

		self.prevClick = 0
		self.currentSelected = 1
		self.prevSelected = 0 
		self.jointSelectorUp = 0
		self.jointSelectorDown = 0
		self.jointMoverPosition = 0.0	

		# Give the publishers time to get setup before trying to do any actual work.
        	rospy.sleep(2)

	def getState(self, string):
		self.stateMessage = string.data
		print(self.stateMessage)

	def sub_joy_states(self, joy):
		self.jointSelectorUp = joy.buttons[15]
		self.jointSelectorDown = joy.buttons[17]
		self.jointAxisButtonX1 = -joy.axes[0]
		self.jointAxisButtonX2 = joy.axes[2]
		self.jointAxisButtonY = joy.axes[1]
		

	def publish_arm_joint_velocities(self):
		if self.stateMessage == "manipulatorPerJoint":

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

			joint_velocities = [joint1, joint2, joint3, joint4, joint5]
			joint_velocities[self.currentSelected] = 0.5*(self.jointAxisButtonY)
			joint_velocities[0] = 0.5*(self.jointAxisButtonX1)
			joint_velocities[4] = 0.5*(self.jointAxisButtonX2)

			jointCommands = []

			for i in range(5):
				joint = JointValue()
				joint.joint_uri = "arm_joint_" + str(i+1)
				joint.unit = "s^-1 rad"
				joint.value = joint_velocities[i]

				joint.timeStamp = rospy.Time.now()
				jointCommands.append(joint)
				#print(i)
				#print(joint_velocities[i])

			desiredVelocities = JointVelocities()
			desiredVelocities.velocities = jointCommands
			self.arm_joint_pub.publish(desiredVelocities)
			if self.prevSelected != self.currentSelected:
				self.arm_joint_select_pub.publish(self.currentSelected)
				self.prevSelected = self.currentSelected
		else:
			joint1 = 0.0
			joint2 = 0.0
			joint3 = 0.0
			joint4 = 0.0
			joint5 = 0.0

			joint_velocities = [joint1, joint2, joint3, joint4, joint5]

			jointCommands = []

			for i in range(5):
				joint = JointValue()
				joint.joint_uri = "arm_joint_" + str(i+1)
				joint.unit = "s^-1 rad"
				joint.value = joint_velocities[i]

				joint.timeStamp = rospy.Time.now()
				jointCommands.append(joint)
				#print(i)
				#print(joint_velocities[i])

			desiredVelocities = JointVelocities()
			desiredVelocities.velocities = jointCommands
			self.arm_joint_pub.publish(desiredVelocities)
			self.prevSelected = 0 # so when state reselected, user gets notification of current selected join immediately
			
			
	def main(self):
		while not rospy.is_shutdown():
			rate = rospy.Rate(10)
			self.publish_arm_joint_velocities()
			rate.sleep()


#if __name__ == '__main__':
rospy.init_node('velocity_move_youbot', anonymous=True)
velocityControl = YoubotArm()
velocityControl.main()
