#!/usr/bin/env python3

import rospy
import cv2, cv_bridge
import numpy as np
import math
import os

class Arm(object):
    def __init__(self):
        """
        Initialize Node
        """
        rospy.init_node('wscr_arm')

        command_sub = rospy.Subscriber() # arm command message topic and type, self.command_callback

    def inverse_kin(self, x, y):
        # inverse kinematics: return 4 angles for the arm joints
        # that would make the laser aim at the desired location
        # Could also use depth if we are using the depth camera

    def command_callback(self, data):
        # extract shot location from command
        x, y = data.x, data.y

        angle1, angle2, angle3, angle4 = self.inverse_kin(x, y)

        # open the claw (laser off)
        gripper_joint_goal = [0.01, 0.01]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        rospy.sleep(0.5)
        self.move_group_gripper.stop()

        # move the arm
        arm_joint_goal = [angle1, angle2, angle3, angle4]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        rospy.sleep(2.0)
        self.move_group_arm.stop()

        # close the claw (laser on)
        gripper_joint_goal = [-0.01, -0.01]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        rospy.sleep(0.5)
        self.move_group_gripper.stop()

    def run(self):
        rospy.spin()