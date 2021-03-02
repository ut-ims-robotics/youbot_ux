#!/usr/bin/env python

import rospy
import smach
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from ds4_driver.msg import Status

#state defining

class Driving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['toManipulatorPerJoint', 'toDriving', 'toSafeMode']) # possible outcomes
	self.regimeButtonState = 0
	self.safeButtonState = 0
	self.subscriber = rospy.Subscriber(rospy.get_param('~/control_options/controls/youbot_states/controlsSubTopic', "joy"), Joy, self.getJoyValues) # Listen to Joy topic
	self.pub = rospy.Publisher('state', String, queue_size=1) # Send current state via 'state' topic
	self.sub_status = rospy.Subscriber('status', Status, self.statusTime, queue_size=1)
	self.statusTime = 0
	self.rate = rospy.Rate(50) # 10hz
	
    def statusTime(self, status):
	self.statusTime = status.header.stamp.secs
	
    def getJoyValues(self, joy):
	self.regimeButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_states/regimeButtonState', 2)]
	self.safeButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_states/safeButtonSate', 10)]

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
	self.sub_status = rospy.Subscriber('status', Status, self.statusTime, queue_size=1)
	self.statusTime = 0
	self.rate = rospy.Rate(50) # 10hz
	
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
	self.sub_status = rospy.Subscriber('status', Status, self.statusTime, queue_size=1)
	self.statusTime = 0
	self.rate = rospy.Rate(50) # 10hz
	
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
	self.regimeButtonState = 0
	self.subscriber = rospy.Subscriber(rospy.get_param('~/control_options/controls/youbot_states/controlsSubTopic', "joy"), Joy, self.getJoyValues)
	self.pub = rospy.Publisher('state', String, queue_size=1)
	self.rate = rospy.Rate(50) # 10hz

    def getJoyValues(self, joy):
	self.regimeButtonState = joy.buttons[rospy.get_param('~/control_options/controls/youbot_states/regimeButtonState', 2)]
    
    def execute(self, userdata):
	self.rate.sleep()
	self.pub.publish("safeMode")
	now = rospy.get_rostime()

        if self.regimeButtonState == 1:
	    while self.regimeButtonState == 1: # wait until button release
		self.rate.sleep()
	    #rospy.loginfo('Going to Driving')
            return 'toDriving'
	else:
	    #rospy.loginfo('Going to SafeMode')
	    return 'toSafeMode' # stay in current regime
        
# main
def main():
    rospy.init_node('youbot_states')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit']) # set program exit outcome
    
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
