#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        #DH Param
	#offset
    	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    	#length
    	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    	#twist
    	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    	#Joints
    	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

	# Create Modified DH parameters
	#KR210 Forward Kinematics
    	DH = { 
		  alpha0:    0,  a0:     0, d1: 0.75, q1:          q1,
		  alpha1:-pi/2., a1:  0.35, d2:    0, q2: -pi/2. + q2,
		  alpha2:    0,  a2:  1.25, d3:    0, q3:          q3,
		  alpha3:-pi/2., a3:-0.054, d4:  1.5, q4:          q4,
		  alpha4: pi/2,  a4:     0, d5:    0, q5:          q5,
		  alpha5:-pi/2., a5:     0, d6:    0, q6:          q6,
		  alpha6:    0,  a6:     0, d7:0.303, q7:           0
             }
        print('Created DH Table')
    	def TF_Matrix(alpha, a, d, q):
		TF = Matrix([
			[           cos(q),           -sin(q),           0,             a],
			[sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
			[sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
			[                0,                 0,           0,             1]
		    	])
		return TF
 
        T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH)
        T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH)
        T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH)
        T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH)
        T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH)
        T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH)
        T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH)
	print('Transforms Calculated')

        #Transformation Matrix from Base Link to End Effector
        T0_EE = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE)
        print('Calculated T0_EE')

	# Compensate for rotation discrepancy between DH parameters and Gazebo
	r, p, y = symbols('r p y')	

	#Roll Rotation
   	rotation_x = Matrix([
			 [1,      0,       0],
			 [0, cos(r), -sin(r)],
			 [0, sin(r),  cos(r)]
		       ])
    	#Pitch Rotation
    	rotation_y = Matrix([
                         [ cos(p), 0, sin(p)],
                         [      0, 1,      0],
                         [-sin(p), 0, cos(p)]
                       ])
    	#Yaw Rotation
    	rotation_z = Matrix([
                         [cos(y), -sin(y), 0],
                         [sin(y),  cos(y), 0],
                         [     0,       0, 1]
                       ])

	#Rotation End Effector
    	rotation_EE = rotation_z * rotation_y * rotation_x
    	rotation_error = rotation_z.subs(y, radians(180)) * rotation_y.subs(p, radians(-90))
    	rotation_EE = rotation_EE * rotation_error
        print('Calculated Rotation EE')

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            print('Start of loop')
	    # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
     	    pose_x = req.poses[x].position.x
    	    pose_y = req.poses[x].position.y
	    pose_z = req.poses[x].position.z

    	    #End Effector Orientation
    	    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([
								req.poses[x].orientation.x,
								req.poses[x].orientation.y,
								req.poses[x].orientation.z,
								req.poses[x].orientation.w
								])
	    
    	    rotation_EE = rotation_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
	    
    	    #End Effector Matrix
    	    EE_matrix = Matrix([[pose_x], [pose_y], [pose_z]])
    	    wrist_center = EE_matrix - (0.303) * rotation_EE[:,2]
	    
	    # Calculate joint angles using Geometric IK method
    	    s_a = 1.501
    	    s_b = sqrt(pow((sqrt(wrist_center[0] * wrist_center[0] + wrist_center[1] * wrist_center[1]) - 0.35),2)
		 + pow((wrist_center[2] - 0.75),2))
    	    s_c = 1.25

    	    #Angles
    	    a_a = acos((s_b * s_b + s_c * s_c - s_a * s_a) / (2 * s_b * s_c))
    	    a_b = acos((s_a * s_a + s_c * s_c - s_b * s_b) / (2 * s_a * s_c))
    	    a_c = acos((s_a * s_a + s_b * s_b - s_c * s_c) / (2 * s_a * s_b))

    	    #Joint Angles
    	    theta1 = atan2(wrist_center[1], wrist_center[0])
    	    theta2 = pi / 2 - a_a - atan2(wrist_center[2] - 0.75, sqrt(wrist_center[0] * wrist_center[0] +
                                                                wrist_center[1] * wrist_center[1]) - 0.35)
    	    theta3 = pi /2 - (a_b + 0.036)

    	    #Calculate rotation from 0 to 3
    	    r0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            r0_3 = r0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

    	    #Calculate rotation from 3 to 6
    	    #r3_6 = r0_3.inv("LU") * rotation_EE
    	    r3_6 = r0_3.transpose() * rotation_EE
    	    #print(simplify(r3_6))

    	    #Remaining Joint Angles
    	    theta4 = atan2(r3_6[2,2], -r3_6[0,2])
    	    theta5 = atan2(sqrt(r3_6[0,2] * r3_6[0,2] + r3_6[2,2] * r3_6[2,2]), r3_6[1,2])
    	    theta6 = atan2(-r3_6[1,1], r3_6[1,0])

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
