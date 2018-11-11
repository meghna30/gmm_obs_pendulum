#!/usr/bin/env python

import rospy
import math
import time


from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Point
from math import sin,cos,atan2,sqrt,fabs,pi
from tf.transformations import euler_from_quaternion


#Define a RRBot joint positions publisher for joint controllers.
def pendulum_joint_torque_publisher():


	#Initiate node for controlling joint1 and joint2 positions.
	rospy.init_node('pendulum_joint_torque_node', anonymous=True)

	#Define publishers for each joint position controller commands.
	pub1 = rospy.Publisher('/pendulum/joint1_torque_controller/command', Float64, queue_size=100)
	pub2 = rospy.Publisher('/pendulum/joint2_torque_controller/command', Float64, queue_size=100)
	#pub3 = rospy.Publisher('/pendulum/joint3_torque_controller/command', Float64, queue_size=100)

	rate = rospy.Rate(100) #100 Hz

	#While loop to have joints follow a certain position, while rospy is not shutdown.
	i = 0
	while not rospy.is_shutdown():

		#Have each joint follow a sine movement of sin(i/100).
		torque = 9.81
		rospy.sleep(20)

		#Publish the same sine movement to each joint.
		#if i < 100:
		#pub1.publish(2.5*torque*0)
		pub1.publish(1.2*torque)
		pub2.publish(2*torque)
		#else:
		#gazebo_link_states()

		#i = i+1

		#rate.sleep() #sleep for rest of rospy.Rate(100)

if __name__ == '__main__':
    try: pendulum_joint_torque_publisher()
    except rospy.ROSInterruptException: pass





