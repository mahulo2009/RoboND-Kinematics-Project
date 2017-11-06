#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
import numpy as np
import math
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

# DH symbols	
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5 ,a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# Joint angle symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')


# Create Modified DH parameters
#
#
s = {
alpha0: 0,         a0: 0,                           d1: 0.75, 
alpha1: -pi/2.,     a1: 0.35,        q2: q2-pi/2.,    d2:0.0,  
alpha2: 0,         a2: 1.25,                        d3:0.0,
alpha3: -pi/2.,     a3: -0.054,                      d4:1.5,
alpha4: pi/2.,      a4: 0.0,                         d5:0.0,
alpha5: -pi/2.,     a5: 0.0,                         d6:0.0,
alpha6: 0,         a6: 0.0,         q7:0.0,         d7:0.303

}

# Transformation matrix
#
#
T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
           [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
           [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
           [                   0,                   0,            0,               1]])
T0_1 = T0_1.subs(s)

T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
           [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
           [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
           [                   0,                   0,            0,               1]])
T1_2 = T1_2.subs(s)

T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
           [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
           [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
           [                   0,                   0,            0,               1]])
T2_3 = T2_3.subs(s)

T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
       [                   0,                   0,            0,               1]])
T3_4 = T3_4.subs(s)

T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
       [                   0,                   0,            0,               1]])
T4_5 = T4_5.subs(s)

T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
       [                   0,                   0,            0,               1]])
T5_6 = T5_6.subs(s)

T6_7 = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
       [                   0,                   0,            0,               1]])
T6_7 = T6_7.subs(s)

T0_7 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7



R_z = Matrix([[ cos(np.pi),     -sin(np.pi),    0],
   [sin(np.pi),     cos(np.pi),     0],
   [0,              0,              1],
   ])

R_y = Matrix([[ cos(-np.pi/2.),  0,              sin(-np.pi/2.)],
  [ 0,              1,              0,],
  [ -sin(-np.pi/2.), 0,              cos(-np.pi/2.)],
  ])
R_corr = R_z * R_y


beta0, beta1, beta2 = symbols('beta0:3')    

R_EEx = Matrix([[ 1,           0,   0],
  [ 0,      cos(beta0),    -sin(beta0)],
  [ 0,      sin(beta0),    cos(beta0)]])

R_EEy = Matrix([[ cos(beta1),        0,  sin(beta1)],
  [       0,        1,        0],
  [-sin(beta1),        0,  cos(beta1)]])

R_EEz = Matrix([[ cos(beta2), -sin(beta2),        0],
  [ sin(beta2),  cos(beta2),        0],
  [ 0,              0,        1]])

R_EE_corr = R_EEz * R_EEy * R_EEx * R_corr 

R0_1 = T0_1.extract([0,1,2],[0,1,2])
R1_2 = T1_2.extract([0,1,2],[0,1,2])
R2_3 = T2_3.extract([0,1,2],[0,1,2])

R_0_3 = R0_1  * R1_2 * R2_3


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

        # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here
             
        # Compensate for rotation discrepancy between DH parameters and Gazebo
        #
        #
            R_EE_corr_eval = R_EE_corr.evalf(subs={beta0: roll, beta1:pitch, beta2:yaw})

        # Calculate joint angles using Geometric IK method
        #
        #
            EE_Position = Matrix ( [[px],[py],[pz]] ) 
            WC_location_eval = EE_Position  - (0.303) * R_EE_corr_eval[:,2]
            
            L=WC_location_eval[2]-0.75
            W=math.sqrt(math.pow(WC_location_eval[0],2)+math.pow(WC_location_eval[1],2))-0.35
            
            A = 1.500971685275908
            B = math.sqrt(math.pow(L,2)+math.pow(W,2))
            C = 1.25
            
            angle_a = math.acos((B*B+C*C-A*A)/(2*B*C))
            angle_b = math.acos((A*A+C*C-B*B)/(2*A*C))
    
            theta1 = math.atan2(WC_location_eval[1],WC_location_eval[0])
            theta2 = math.pi/2.0-angle_a-math.atan2(L,W)
            theta3 = math.pi/2.0-(angle_b+0.036)

            R_0_3_eval = R_0_3.evalf(subs={q1: theta1, q2:theta2, q3:theta3})
            R_0_3_Inv = Transpose(R_0_3_eval)
            R_3_6 = R_0_3_Inv * R_EE_corr_eval   

            theta4 = math.atan2(R_3_6[2,2],-R_3_6[0,2])
            theta5 = math.atan2( math.sqrt( math.pow(R_3_6[0,2],2)+math.pow(R_3_6[2,2],2)), R_3_6[1,2] )
            theta6 = math.atan2(-R_3_6[1,1],R_3_6[1,0])

            ###
	    print("theta4= ",theta4," theta5= ",theta5," theta6= ",theta6)
        
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
