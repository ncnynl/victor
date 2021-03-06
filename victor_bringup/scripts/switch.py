#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from sound_play.msg import SoundRequest

from sound_play.libsoundplay import SoundClient

# This script will listen for joystick button 4(Y) being toggled and
# send zero speed messages to the mux to disable the behavior until
# button 4(Y) is pressed again.

class BehaviorSwitch(object):
    def __init__(self):
        self.running = True # Will Toggle Auto To False with logic
	self.in_toggle = False
	self.line_follow_toggle = False
	self.line_follow_running = True
    def callback(self, joy_msg):
        if joy_msg.buttons[4] == 1:
	  self.line_follow_toggle = True
	elif joy_msg.buttons[1] == 1:
	  self.in_toggle = True 
	
	if joy_msg.buttons[4] == 0 and self.line_follow_toggle == True:
	  self.line_follow_running = not self.line_follow_running
	  self.line_follow_toggle = False
	  bool_msg = Bool()
	  bool_msg.data = self.line_follow_running;
	  line_follower_enable_pub = rospy.Publisher('line_follower/enable', Bool, queue_size=1)
	  line_follower_enable_pub.publish(bool_msg)
	  self.soundhandle.play(3)
	  rospy.loginfo("Got Toggle Button (Y) %d", self.line_follow_running)
	elif joy_msg.buttons[1] == 0 and self.in_toggle == True:
	  self.running = not self.running
	  self.in_toggle = False
	  rospy.loginfo("Got Toggle Button (B) %d", self.running)
          self.soundhandle.play(1)
        #rospy.loginfo(repr(joy_msg))

    def run(self):
        rospy.init_node('behavior_switch', anonymous=True)
        self.soundhandle = SoundClient()

        rospy.sleep(1)

        self.soundhandle.stopAll()

        pub = rospy.Publisher('cmd_vel_mux/input/switch', Twist, queue_size=10)
        
        
        rospy.Subscriber('joy', Joy, self.callback)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.running:
                empty_msg = Twist()
                pub.publish(empty_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        behavior_switch = BehaviorSwitch()
        behavior_switch.run()
    except rospy.ROSInterruptException:
        pass

