#!/usr/bin/env python

import rospy 
import time
import math 
import numpy as np
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelState
import geometry_msgs.msg
from math import sin,cos,atan2, sqrt,fabs,pi
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import JointState

class  torque_controller():

    def __init__(self):
        rospy.init_node('pendulum_joint_torque_node', anonymous=True)
		
		#Define publishers for each joint position controller commands.
        self.pub1 = rospy.Publisher('/pendulum/joint1_torque_controller/command', Float64, queue_size=1)
        self.pub2 = rospy.Publisher('/pendulum/joint2_torque_controller/command', Float64, queue_size=1)

        self.kp = 100
        self.kd = 10
        self.theta1_des = -np.pi
        self.theta2_des = -np.pi/3.

    def create_controller(self):
        state = rospy.Subscriber('/pendulum/joint_states',JointState,self.callback)
        rospy.loginfo('Subscribing to the joint states')
        rospy.spin()

    def callback(self, state):
        """
        jointN_pos = [theta, theta_dot, torque]
        """
        joint1_state = [state.position[0],state.velocity[0],state.effort[0]]
        joint2_state = [state.position[1],state.velocity[1],state.effort[1]]
        
        torque1 = self.kp*(self.theta1_des-joint1_state[0])-self.kd*joint1_state[1]
        torque2 = self.kp*(self.theta2_des-joint2_state[0])-self.kd*joint2_state[1]
        self.pub1.publish(torque1)
        self.pub2.publish(torque2)

if __name__ == '__main__':
    Controller = torque_controller()
    Controller.create_controller()







