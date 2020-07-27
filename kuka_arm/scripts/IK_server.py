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
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def createMatrix(alpha, a, q, d):
    mat = Matrix([[cos(q), -sin(q), 0, a],
                  [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                  [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
                  [0, 0, 0, 1]])

    return mat


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        d0, d1, d2, d3, d4, d5, d6 = symbols('d0:7')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        q0, q1, q2, q3, q4, q5, q6 = symbols('q0:7')

        # Create Modified DH parameters
        dh_params = {alpha0: 0, a0: 0, d0: 0.75, q0: q0,
                     alpha1: -pi / 2, a1: 0.35, d1: 0, q1: -pi / 2 + q1,
                     alpha2: 0, a2: 1.25, d2: 0, q2: q2,
                     alpha3: -pi / 2, a3: -0.054, d3: 1.50, q3: q3,
                     alpha4: pi / 2, a4: 0, d4: 0, q4: q4,
                     alpha5: -pi / 2, a5: 0, d5: 0, q5: q5,
                     alpha6: 0, a6: 0, d6: 0.303, q6: 0}

        # Define Modified DH Transformation matrix
        T0_1 = createMatrix(alpha0, a0, q0, d0).subs(dh_params)
        T1_2 = createMatrix(alpha1, a1, q1, d1).subs(dh_params)
        T2_3 = createMatrix(alpha2, a2, q2, d2).subs(dh_params)
        T3_4 = createMatrix(alpha3, a3, q3, d3).subs(dh_params)
        T4_5 = createMatrix(alpha4, a4, q4, d4).subs(dh_params)
        T5_6 = createMatrix(alpha5, a5, q5, d5).subs(dh_params)
        T6_G = createMatrix(alpha6, a6, q6, d6).subs(dh_params)

        # Create individual transformation matrices
        T0_2 = T0_1 * T1_2
        T0_3 = T0_2 * T2_3
        T0_4 = T0_3 * T3_4
        T0_5 = T0_4 * T4_5
        T0_6 = T0_5 * T5_6
        T0_G = T0_6 * T6_G

        # Extract rotation matrices from the transformation matrices
        R0_3 = T0_3[0:3, 0:3]

        r, p, y = symbols("r p y")

        ROT_x = Matrix([[1,      0,       0],
			[0, cos(r), -sin(r)],
			[0, sin(r), cos(r)]])

        ROT_y = Matrix([[cos(p),  0, sin(p)],
			[0,       1,      0],
			[-sin(p), 0, cos(p)]])

        ROT_z = Matrix([[cos(y), -sin(y), 0],
			[sin(y),  cos(y), 0],
			[0,            0, 1]])

        ROT_EE = ROT_z * ROT_y * ROT_x
        ROT_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

        ROT_EE = ROT_EE * ROT_Error

	# Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()
            print('Calculating %s from %s' % (x + 1, len(req.poses)))

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

            ROT_EE_num = ROT_EE.subs({"r": roll, "p": pitch, "y": yaw})

            EE = Matrix([[px],
                         [py],
                         [pz]])

            WC = EE - 0.303 * ROT_EE_num[:, 2]

            # Calculate joint angles using Geometric IK method
            theta1 = atan2(WC[1], WC[0])

            # SSS triangle for theta2 and theta3
            side_a = 1.501
            side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
            side_c = 1.25

            # cos laws
            angle_a = acos((side_b ** 2 + side_c ** 2 - side_a ** 2) / (2 * side_b * side_c))
            angle_b = acos((side_a ** 2 + side_c ** 2 - side_b ** 2) / (2 * side_a * side_c))
            # angle_c = acos((side_b ** 2 + side_b ** 2 - side_c ** 2) / (2 * side_a * side_b))

            theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
            theta3 = pi / 2 - (angle_b + 0.036)

            R0_3_num = R0_3.evalf(subs={q0: theta1, q1: theta2, q2: theta3})

            R3_6 = R0_3_num.transpose() * ROT_EE_num

            theta5 = atan2(sqrt(R3_6[0, 2] ** 2 + R3_6[2, 2] ** 2), R3_6[1, 2])
            if sin(theta5) < 0:
                theta4 = atan2(- R3_6[2, 2], R3_6[0, 2])
                theta6 = atan2(R3_6[1, 1], - R3_6[1, 0])
            else:
                theta4 = atan2(R3_6[2, 2], - R3_6[0, 2])
                theta6 = atan2(- R3_6[1, 1], R3_6[1, 0])


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
