#!/usr/bin/env python 
import rospy 
from std_msgs.msg import Int8
from time import sleep

# Initialize the node with rospy 
rospy.init_node('propbot_interface') 

# Create publisher 
pub_l = rospy.Publisher("/left_wheel",Int8,queue_size=1) 
pub_r = rospy.Publisher("/right_wheel",Int8,queue_size=1) 


while not rospy.is_shutdown():
	print("insert left/right wheel speeds: ")
	wheel_input = raw_input() #space seperated left right
	pub_l.publish(wheel_input.split()[0])
	pub_r.publish(wheel_input.split()[1])	
	sleep(1)
	
