#!/usr/bin/env python

import rospy
from std_msgs.msg import String
#from sensor_msgs.msg import JoyFeedbackArray, JoyFeedback
from ds4_driver.msg import Feedback


class Handler(object):
	def __init__(self, feedback_topic='set_feedback'):
		
		self.pub_feedback = rospy.Publisher(feedback_topic, Feedback, queue_size=1)
		self.state_sub = rospy.Subscriber(rospy.get_param('~sub_topic', "state"), String, self.getState)
		self.stateMessage = None

	def getState(self, string):
		self.stateMessage = string.data

	def publishFeedback(self):
		feedback = Feedback()
		if self.stateMessage == "driving":
			feedback.set_led = True
			feedback.led_r = 0.0
			feedback.led_g = 1.0
			feedback.led_b = 0.0			

		elif self.stateMessage == "manipulatorPerJoint":
			feedback.set_led = True
			feedback.led_r = 1.0
			feedback.led_g = 1.0
			feedback.led_b = 0.0

		elif self.stateMessage == "safeMode":
			feedback.set_led = True
			feedback.led_r = 0.0
			feedback.led_g = 0.0
			feedback.led_b = 1.0

		self.pub_feedback.publish(feedback)			
		
	def main(self):
		while not rospy.is_shutdown():
			rate = rospy.Rate(10)
			self.publishFeedback()
			rate.sleep()


#if __name__ == '__main__':
rospy.init_node('youbot_states_feedback', anonymous=True)
handler = Handler()
handler.main()
