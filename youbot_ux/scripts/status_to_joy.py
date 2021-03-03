#!/usr/bin/env python

import rospy
import copy
from sensor_msgs.msg import Joy
from ds4_driver.msg import Status

# Built by example from ds4_driver controller_ros node

class StatusConverter():
	def __init__(self):
		self.sub_status = rospy.Subscriber('status', Status, self.status_to_joy, queue_size=1)
		self.pub_joy = rospy.Publisher('joy', Joy, queue_size=1)
		self._prev_joy = None
		self._autorepeat_rate = 10

		self.rate = rospy.Rate(100)

		period = 1.0 / self._autorepeat_rate
		rospy.Timer(rospy.Duration(period), self.prev_joy_republish)

	def status_to_joy(self, status):
		msg = Joy()
		msg.header = copy.deepcopy(status.header)
		msg.axes = [
		    status.axis_left_x,
		    status.axis_left_y,
		    status.axis_right_x,
		    status.axis_right_y,
		    status.axis_l2,
		    status.axis_r2,
		]
		msg.buttons = [
		    status.button_square,
		    status.button_triangle,
		    status.button_circle,
		    status.button_cross,
		    status.button_l1,
		    status.button_l2,
		    status.button_r1,
		    status.button_r2,
		    status.button_share,
		    status.button_options,
		    status.button_ps,
		    status.button_trackpad,
		    status.button_l3,
		    status.button_r3,
		    status.button_dpad_left,
		    status.button_dpad_up,
		    status.button_dpad_right,
		    status.button_dpad_down,
		]

		if self._prev_joy is None or msg.axes != self._prev_joy.axes or msg.buttons != self._prev_joy.buttons:
                	self.pub_joy.publish(msg)
		
		self._prev_joy = msg
		self.rate.sleep()

	def prev_joy_republish(self, time):
		if self._prev_joy is not None: 
			self.pub_joy.publish(self._prev_joy)
        
# main
def main(controller):
	rospy.init_node('status_to_joy', anonymous=True)
	if (controller == "ds4"):
 		StatusConverter()
	rospy.spin()

if __name__ == '__main__':
    controller = rospy.get_param('~/control_options/controller', "ds4")
    main(controller)
