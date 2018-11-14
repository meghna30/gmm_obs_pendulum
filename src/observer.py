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

class  pendulum_momentum_obs():

    def __init__(self):
        rospy.init_node('gmm_obs', anonymous=True)

        #rospy.Subscriber('/gazebo/model_states',ModelState,do_obs)
        self.pub1 = rospy.Publisher('/pendulum/torque_disturbance',Float64,queue_size = 100)
        ## loading the model parameters ##
        self.no_joints = 2
        self.m1 = 1
        self.m2 = 1
        self.I1 = 0.01
        self.I2 = 0.01
        self.l1 = 1.5
        self.l2 = 1.5
        self.M = np.zeros((self.no_joints, self.no_joints))
        self.C = np.zeros((self.no_joints, self.no_joints))
        self.G = np.zeros((self.no_joints, 1))
        self.S = np.zeros((self.no_joints, self.no_joints))
        self.J = np.zeros((self.no_joints, self.no_joints))
        self.t_prev = 0.0
        self.freq = 15
        self.g = 9.81
        self.torque_d_prev =[[0],[0]]
        self.P_k_prev = [[0],[0]]
        self.start = 0

    def create_observer(self):

        state = rospy.Subscriber('/pendulum/joint_states',JointState,self.do_obs)
        rospy.loginfo('Subscribing to the joint states')
        rospy.spin()

    def do_obs(self, state):
        """
        jointN_pos = [theta, theta_dot, torque]
        """

        if self.start == 0:
            joint1_state = [state.position[0],state.velocity[0],state.effort[0]]
            joint2_state = [state.position[1],state.velocity[1],state.effort[1]]
        
            self.M[0,0] = 0.25*self.m1*self.l1**2 + self.I1 + self.I2 + self.m2*self.l1**2 + 0.25*self.m2*self.l2**2 + self.m2*self.l1*self.l2*np.cos(joint2_state[0])
            self.M[0,1] = 0.25*self.m2*self.l2**2 + self.I2 + 0.5*self.m2*self.l1*self.l2*np.cos(joint2_state[0])
            self.M[1,0] = 0.25*self.m2*self.l2**2 + self.I2 + 0.5*self.m2*self.l1*self.l2*np.cos(joint2_state[0])
            self.M[1,1] = 0.25*self.m2*self.l2**2 + self.I2

            self.C[0,0] = -0.5*self.l1*self.l2*self.m2*np.sin(joint2_state[0])*joint2_state[1]
            self.C[0,1] = -0.5*self.m2*self.l1*self.l2*np.sin(joint2_state[0])*joint1_state[1] - 0.5*self.l1*self.l2*self.m2*np.sin(joint2_state[0])*joint2_state[1]
            self.C[1,0] = 0.5*self.l1*self.l2*self.m2*np.sin(joint2_state[0])*joint1_state[1]
            self.C[1,1] = 0.0

            self.G[0,0] = -0.5*self.m1*self.g*self.l1*np.sin(joint1_state[0]) - self.m2*self.g*self.l1*np.sin(joint1_state[0]) - 0.5*self.m2*self.g*self.l2*np.sin(joint1_state[0]+joint2_state[0])
            self.G[1,0] = -0.5*self.m2*self.g*self.l2*np.sin(joint1_state[0]+joint2_state[0])

            self.S[0,0] = 1
            self.S[0,1] = 0
            self.S[1,0] = 0
            self.S[1,1] = 1

            self.J[0,0] = -self.l1*np.cos(joint1_state[0]) - self.l2*np.cos(joint1_state[0]+joint2_state[0])
            self.J[0,1] = -self.l2*np.cos(joint1_state[0]+joint2_state[0])
            self.J[1,0] = -self.l1*np.sin(joint1_state[0]) - self.l2*np.sin(joint1_state[0]+joint2_state[0])
            self.J[1,1] = -self.l2*np.sin(joint1_state[0]+joint2_state[0])

            theta_dot = [[joint1_state[1]],[joint2_state[1]]]
        
            t_now = rospy.Time.now() 
            self.t_prev = t_now

        
            self.P_k_prev = np.matmul(self.M,theta_dot)

            self.torque_d_prev = 0
            self.start = 1


        else:
            joint1_state = [state.position[0],state.velocity[0],state.effort[0]]
            joint2_state = [state.position[1],state.velocity[1],state.effort[1]]
        
            self.M[0,0] = 0.25*self.m1*self.l1**2 + self.I1 + self.I2 + self.m2*self.l1**2 + 0.25*self.m2*self.l2**2 + self.m2*self.l1*self.l2*np.cos(joint2_state[0])
            self.M[0,1] = 0.25*self.m2*self.l2**2 + self.I2 + 0.5*self.m2*self.l1*self.l2*np.cos(joint2_state[0])
            self.M[1,0] = 0.25*self.m2*self.l2**2 + self.I2 + 0.5*self.m2*self.l1*self.l2*np.cos(joint2_state[0])
            self.M[1,1] = 0.25*self.m2*self.l2**2 + self.I2

            self.C[0,0] = -0.5*self.l1*self.l2*self.m2*np.sin(joint2_state[0])*joint2_state[1]
            self.C[0,1] = -0.5*self.m2*self.l1*self.l2*np.sin(joint2_state[0])*joint1_state[1] - 0.5*self.l1*self.l2*self.m2*np.sin(joint2_state[0])*joint2_state[1]
            self.C[1,0] = 0.5*self.l1*self.l2*self.m2*np.sin(joint2_state[0])*joint1_state[1]
            self.C[1,1] = 0.0

            self.G[0,0] = -0.5*self.m1*self.g*self.l1*np.sin(joint1_state[0]) - self.m2*self.g*self.l1*np.sin(joint1_state[0]) - 0.5*self.m2*self.g*self.l2*np.sin(joint1_state[0]+joint2_state[0])
            self.G[1,0] = -0.5*self.m2*self.g*self.l2*np.sin(joint1_state[0]+joint2_state[0])

            self.S[0,0] = 1
            self.S[0,1] = 0
            self.S[1,0] = 0
            self.S[1,1] = 1

            self.J[0,0] = -self.l1*np.cos(joint1_state[0]) - self.l2*np.cos(joint1_state[0]+joint2_state[0])
            self.J[0,1] = -self.l2*np.cos(joint1_state[0]+joint2_state[0])
            self.J[1,0] = -self.l1*np.sin(joint1_state[0]) - self.l2*np.sin(joint1_state[0]+joint2_state[0])
            self.J[1,1] = -self.l2*np.sin(joint1_state[0]+joint2_state[0])
    
            theta =[[joint1_state[0]],[joint2_state[0]]]
            theta_dot = [[joint1_state[1]],[joint2_state[1]]]
            torque_ext = [[joint1_state[2]],[joint2_state[2]]]

            t_now = rospy.Time.now()
            t_delta = 1./1000.
            self.t_prev = t_now

            gamma = np.exp(-self.freq*t_delta)

            beta = (1-gamma)/(gamma*t_delta)

            P_k = np.matmul(self.M,theta_dot)
            alpha_k = beta*P_k + np.matmul(self.S,torque_ext) + np.matmul(np.transpose(self.C),theta_dot) - self.G
            torque_d = (gamma-1)*alpha_k + beta*(P_k - gamma*self.P_k_prev) + gamma*self.torque_d_prev

            disturbance_force = np.matmul(np.matmul(np.linalg.inv(np.matmul(self.S,np.transpose(self.J))),self.S),torque_d)

            self.torque_d_prev = torque_d
            self.P_k_prev = P_k
            print(disturbance_force)

if __name__ == '__main__':
    O = pendulum_momentum_obs()
    O.create_observer()

