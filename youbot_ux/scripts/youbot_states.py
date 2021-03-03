#!/usr/bin/env python

import rospy
import smach
import os
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Empty
from std_srvs.srv import Empty
from ds4_driver.msg import Status

#state defining

class Driving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['toManipulatorPerJoint', 'toDriving', 'toSafeMode']) # possible outcomes
	self.regimeButtonState = 0
	self.safeButtonState = 0
	self.subscriber = rospy.Subscriber(rospy.get_param('~/control_options/controls/youbot_states/controlsSubTopic', "joy"), Joy, self.getJoyValues) # Listen to Joy topic
	self.pub = rospy.Publisher('state', String, queue_size=1) # Send current state via 'state' topic
	self.subStatus = rospy.Subscriber('status', Status, self.statusTime, queue_size=1)
	self.statusTime = 0
	self.rate = rospy.Rate(50) # 50hz
	
    def statusTime(self, status):
	self.statusTime = status.header.stamp.secs
	
    # block for getting and saving joy topic messages
    def getJoyValues(self, joy):
	self.regimeButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_states/regimeButtonState', 2)]
	self.safeButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_states/safeButtonSate', 10)]

    # block to check for controller button input for regime change, check if there is a controller (timer for safe mode)
    def execute(self, userdata):
	self.pub.publish("driving")
	now = rospy.get_rostime()
	self.rate.sleep()

        if self.regimeButtonState == 1:
	    while self.regimeButtonState == 1: # wait until button release
		self.rate.sleep()
	    rospy.loginfo('Going to ManipulatorPerJoint')
	    self.rate.sleep()
            return 'toManipulatorPerJoint'
	elif now.secs - self.statusTime > 1 or self.safeButtonState == 1:
	    while self.safeButtonState == 1: # wait until button release
		self.rate.sleep()
	    return 'toSafeMode'
	else:
	    #rospy.loginfo('Going to Driving')
	    return 'toDriving' # stay in current regime


class ManipulatorPerJoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['toTrajectoryRecord', 'toManipulatorPerJoint', 'toSafeMode']) 
	self.regimeButtonState = 0
	self.safeButtonState = 0
	self.subscriber = rospy.Subscriber(rospy.get_param('~/control_options/controls/youbot_states/controlsSubTopic', "joy"), Joy, self.getJoyValues)
	self.pub = rospy.Publisher('state', String, queue_size=1)
	self.subStatus = rospy.Subscriber('status', Status, self.statusTime, queue_size=1)
	self.statusTime = 0
	self.rate = rospy.Rate(50) # 50hz
	
    def statusTime(self, status):
	self.statusTime = status.header.stamp.secs

    def getJoyValues(self, joy):
	self.regimeButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_states/regimeButtonState', 2)]
	self.safeButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_states/safeButtonSate', 10)]
    
    def execute(self, userdata):
	self.pub.publish("manipulatorPerJoint")
	now = rospy.get_rostime()
	self.rate.sleep()

        if self.regimeButtonState == 1:
	    while self.regimeButtonState == 1: # wait until button release
		self.rate.sleep() # wait after each button state check to save resources
	    rospy.loginfo('Going to TrajectoryRecord')
	    self.rate.sleep()
            return 'toTrajectoryRecord'
	elif now.secs - self.statusTime > 1 or self.safeButtonState == 1:
	    while self.safeButtonState == 1: # wait until button release
		self.rate.sleep()
	    return 'toSafeMode'
	else:
	    #rospy.loginfo('Going to ManipulatorPerJoint')
	    return 'toManipulatorPerJoint' # stay in current regime

class TrajectoryRecord(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['toDriving', 'toTrajectoryRecord', 'toSafeMode']) 
	self.regimeButtonState = 0
	self.safeButtonState = 0
	self.subscriber = rospy.Subscriber(rospy.get_param('~/control_options/controls/youbot_states/controlsSubTopic', "joy"), Joy, self.getJoyValues)
	self.pub = rospy.Publisher('state', String, queue_size=1)
	self.subStatus = rospy.Subscriber('status', Status, self.statusTime, queue_size=1)
	self.statusTime = 0
	self.rate = rospy.Rate(50) # 50hz
	
    def statusTime(self, status):
	self.statusTime = status.header.stamp.secs

    def getJoyValues(self, joy):
	self.regimeButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_states/regimeButtonState', 2)]
	self.safeButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_states/safeButtonSate', 10)]
    
    def execute(self, userdata):
	self.pub.publish("trajectoryRecord")
	now = rospy.get_rostime()
	self.rate.sleep()

        if self.regimeButtonState == 1:
	    while self.regimeButtonState == 1: # wait until button release
		self.rate.sleep()
	    rospy.loginfo('Going to Driving')
	    self.rate.sleep()
            return 'toDriving'
	elif now.secs - self.statusTime > 1 or self.safeButtonState == 1:
	    while self.safeButtonState == 1: # wait until button release
		self.rate.sleep()
	    return 'toSafeMode'
	else:
	    #rospy.loginfo('Going to ManipulatorPerJoint')
	    return 'toTrajectoryRecord' # stay in current regime

class SafeMode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['toSafeMode', 'toDriving']) 
	self.subscriber = rospy.Subscriber(rospy.get_param('~/control_options/controls/youbot_states/controlsSubTopic', "joy"), Joy, self.getJoyValues)
	self.pub = rospy.Publisher('state', String, queue_size=1)
	self.rate = rospy.Rate(50) # 50hz

	self.regimeButtonState = 0
	self.quit1ButtonState = 0
	self.quit2ButtonState = 0

	self.motorsOn = rospy.ServiceProxy('arm_1/switchOnMotors', Empty) # motors on for no free-falling
	self.motorsOnCheck = 0

    def getJoyValues(self, joy):
	self.regimeButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_states/regimeButtonState', 2)]
	self.quit1ButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_states/quit1ButtonState', 8)]
	self.quit2ButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_states/quit2ButtonState', 9)]
    
    def execute(self, userdata):
	self.rate.sleep()
	self.pub.publish("safeMode")
	now = rospy.get_rostime()

	if self.motorsOnCheck == 0: # One time check to turn on motors in case of crash of youbot_trajectory_record.py 
		#self.motorsOn()
		self.motorsOnCheck = 1

        if self.regimeButtonState == 1:
	    while self.regimeButtonState == 1: # wait until button release
		self.rate.sleep()
	    self.motorsOnCheck = 0
	    #rospy.loginfo('Going to Driving')
            return 'toDriving'
	elif self.quit1ButtonState and self.quit2ButtonState:
	    self.pub.publish("powerOff")
	    rospy.sleep(1.)
	    os.system("rosnode kill -a")
	else:
	    #rospy.loginfo('Going to SafeMode')
	    return 'toSafeMode' # stay in current regime
	
        
# main
def main():
    rospy.init_node('youbot_states')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit']) # set program exit outcome
    rospy.sleep(1.)
    # Open the container
    with sm:
        # Add states to the container
	
        smach.StateMachine.add('SafeMode', SafeMode(), 
                               transitions={'toDriving':'Driving',
					    'toSafeMode':'SafeMode'})
        smach.StateMachine.add('Driving', Driving(), 
                               transitions={'toManipulatorPerJoint':'ManipulatorPerJoint', #1 next regime
					    'toDriving':'Driving',			   #2 current class regime
					    'toSafeMode':'SafeMode'})			   #3 safe mode
        smach.StateMachine.add('ManipulatorPerJoint', ManipulatorPerJoint(), 
                               transitions={'toTrajectoryRecord':'TrajectoryRecord',
					    'toManipulatorPerJoint': 'ManipulatorPerJoint',
					    'toSafeMode':'SafeMode'})
        smach.StateMachine.add('TrajectoryRecord', TrajectoryRecord(), 
                               transitions={'toDriving':'Driving',
					    'toTrajectoryRecord': 'TrajectoryRecord',
					    'toSafeMode':'SafeMode'})

	

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    
    main()
