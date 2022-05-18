#!/usr/bin/env python3

import rospy
import cv2, cv_bridge
import numpy as np
import math
import os

import moveit_commander

class Arm(object):
    def __init__(self):
        """
        Initialize Node
        """
        rospy.init_node('wscr_arm')

        #initialize parameters
        self.l2 = 5 #TODO fill in
        self.l1 = 5 #TODO fill in
        self.curr_arm_goals = [math.radians(0.0), math.radians(20.0), math.radians(0.0), math.radians(-10.0)]

        #arm subscriber
        rospy.Subscriber('/robot_arm_action', RobotArmAction, self.arm_action_received) 

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

    def inverse_kin(self, x, y, depth):
        # inverse kinematics: return 4 angles for the arm joints
        # that would make the laser aim at the desired location
        # Could also use depth if we are using the depth camera
        #use inverse kinematics to get joint angles for joint1 and joint3

        q3 = math.acos((-(self.l1**2) - self.l2**2 + x**2 + y**2) / (2*self.l1*self.l2))
        q1 = math.atan2(y, x) - math.atan2((self.l2*math.sin(q3)) / (self.l1 + self.l2*math.cos(q3)))
        return [q1, q3]

    def arm_action_received(self, data):
        # extract shot location from command
        x, y = data.x, data.y
        depth = data.depth

        joints = self.inverse_kin(x, y, depth)


        '''
        # open the claw (laser off)
        gripper_joint_goal = [0.01, 0.01]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        rospy.sleep(0.5)
        self.move_group_gripper.stop()
        '''
        # move the arm
        self.curr_arm_goals[0] = joints[0]
        self.curr_arm_goals[2] = joints[1]
        self.move_group_arm.go(self.curr_arm_goals, wait=True)
        self.move_group_arm.stop()

        # close the claw (laser on)
        gripper_joint_goal = [-0.01, -0.01]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        rospy.sleep(0.5)
        self.move_group_gripper.stop()
    
    # Sets the arm position of the robot to point upward
    def reset_arm_position(self):

        # Fetch arm joint and gripper positions from self.positions
        arm_joint_goal = [math.radians(0.0), math.radians(20.0), math.radians(0.0), math.radians(-10.0)]
        gripper_joint_goal = [0.019, 0.019]

        # Send arm joint move, and call stop() to prevent residual movement
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop() 

        # Send gripper move, and call stop() to prevent residual movement
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        rospy.sleep(4)


    def run(self):
        rospy.spin()