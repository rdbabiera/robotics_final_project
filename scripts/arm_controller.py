#!/usr/bin/env python3

import rospy
import cv2, cv_bridge
import numpy as np
import math
import os
from robotics_final_project.msg import VisionCoords


import moveit_commander

class Arm(object):
    def __init__(self):
        """
        Initialize Node
        """
        rospy.init_node('wscr_arm')

        #initialize parameters
        self.curr_arm_goals = [math.radians(0.0), math.radians(20.0), math.radians(0.0), math.radians(-10.0)]

        #arm subscriber
        rospy.Subscriber('/robot_arm_action', VisionCoords, self.arm_action_received) 

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Allow subscriber time to set up
        rospy.sleep(1)
        print("[ARM] Ready to Go!")

    def spherical_to_cartesian(self, r, theta, phi):
        return np.array([r * np.cos(phi) * np.sin(theta),
                         r * np.sin(phi) * np.sin(theta),
                         r * np.cos(theta)])

    def cartesian_to_spherical(self, x, y, z):
        r = np.sqrt(x**2 + y**2 + z**2)
        theta = math.acos(z / r)
        if x > 0:
            phi = math.atan(y / x)
        elif x < 0:
            phi = math.atan(y / x) + (np.pi if y >= 0 else -np.pi)
        else:
            phi = np.pi/2 if y >= 0 else -np.pi/2
        return np.array([r, theta, phi])


    def inverse_kin(self, x, y, depth):
        # inverse kinematics: return 4 angles for the arm joints
        # that would make the laser aim at the desired location
        # Could also use depth if we are using the depth camera
        
        # x and y are image pixel coordinates, depth is the
        # corresponding reading from the depth camera

        H = 480 # image height in pixels
        W = 640 # image width in pixels
        H_FOV = 42 * np.pi/180 # camera total vertical FOV in radians
        W_FOV = 50 * np.pi/180 # camera total horizontal FOV in radians

        # get spherical coordinates of target point in 3D relative to camera's location
        phi = math.atan((float(x)/W - 0.5) * np.tan(.5 * W_FOV) * 2)
        theta = np.pi/2 - math.atan((float(y)/H - 0.5) * np.tan(.5 * H_FOV) * 2)
        r = float(depth)

        # adjust point based on difference between camera location and laser location
        # convert to cartesian, adjust, then convert back to spherical
        # x axis is forward and back
        # y axis is left and right
        # z axis is up and down
        pos = self.spherical_to_cartesian(r, theta, phi)

        # Location of the laser relative to the camera
        # this should be in the same units as the depth reading (mm)
        # Assuming laser goes out parallel to the gripper direction
        # we can say that the laser "starts" at the elbow joint, which is
        # always in the same location
        camera_laser_relative_pos = np.array([-200, 0, 175])

        pos_relative_to_laser = pos - camera_laser_relative_pos
        r, theta, phi = self.cartesian_to_spherical(*pos_relative_to_laser)

        # the first joint swivles to match phi, the second joint points straight up
        # the third joint rotates to match theta, the wrist joint is straight
        return [-phi, 0.0, (np.pi/2 - theta - 0.1), 0.0]

    def arm_action_received(self, data):
        # extract shot location from command
        print("received x:", data.x)
        print("received y:", data.y)
        print("received depth:", data.depth)

        x, y = data.x, data.y
        depth = data.depth

        joints = self.inverse_kin(x, y, depth)
        print("joints:", joints)

        # move the arm
        self.curr_arm_goals = joints
        self.move_group_arm.go(self.curr_arm_goals, wait=True)
        self.move_group_arm.stop()
    
    # Sets the arm position of the robot to neutral position when starting
    def reset_arm_position(self):

        # Fetch arm joint and gripper positions from self.positions
        arm_joint_goal = [math.radians(0.0), math.radians(0.0), math.radians(0.0), math.radians(0.0)]
        gripper_joint_goal = [-0.01, -0.01]
        #gripper_joint_goal = [0.0, 0.0]

        # Send arm joint move, and call stop() to prevent residual movement
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop() 

        # Send gripper move, and call stop() to prevent residual movement
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        rospy.sleep(1)


    def run(self):
        self.reset_arm_position()
        rospy.spin()
    
if __name__ == '__main__':
    # declare the ROS node and run it
    ArmClass = Arm()
    ArmClass.run()