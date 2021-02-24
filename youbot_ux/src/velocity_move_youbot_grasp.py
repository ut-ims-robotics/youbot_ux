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
from std_msgs.msg import String
        

jointMax= [5.840139, 2.617989, -0.0157081, 3.42919, 5.641589]
jointMin= [0.01006921, 0.01006921, -5.0264, 0.0221391, 0.11062]

jointHome= [0.01007,0.01007,-0.15709,0.02214,0.1107]
jointCamera= [3.0,0.5,-0.9,0.1,3.0]
jointObject= [3.04171,0.63597,-1.017845,0.36284,2.876194]
jointGrasp = [3.04171,2.04427,-1.5189129,2.5434289757,2.8761944]
jointInitialize= [0.01007,.635971,-1.91989,1.04424,2.87619]

jointGuessForGrasp=[0.0, 0.0, 1.52, 1.84, -1.26, 2.4, 3.10]

armJointPosCandle = np.array([2.9496, 1.1344, -2.5482, 1.789, 2.9234])

gripperWidthAtGrasp = 0.00411
gripperWidthOpen = 0.0099

# Position and orientation above the grasping target
quat_above_grasp = np.array([0.601, 0.591, -0.372, 0.388])
pos_above_grasp = np.array([0.181, 0.778, 0.108])


class YoubotArm:

	def __init__(self):
		self.gripper_pub = rospy.Publisher("arm_1/gripper_controller/position_command", JointPositions, queue_size=0)
		self.state_sub = rospy.Subscriber(rospy.get_param('~sub_topic', "state"), String, self.getState)
		self.joy_sub = rospy.Subscriber(rospy.get_param('~sub_topic', "joy"), Joy, self.getJoy)		

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
		self.amountOfChange = 0.00025*(-1*joy.axes[4]+joy.axes[5])

		
	def simple_mapping(self, value, in_min, in_max, out_min, out_max):
	
	#return ((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)
		resultValue = ((value-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

		return resultValue

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

