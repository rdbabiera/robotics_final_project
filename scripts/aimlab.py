#!/usr/bin/env python3

import rospy
import cv2, cv_bridge
import numpy as np
import math
import os


from arm_controller import Arm
from vision_controller import Eyes


from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Path of directory to locate where to find q_matrices
learning_path = os.path.dirname(__file__) + "/learning/"


class Aimlab(object):
    def __init__(self):
        """
        Initialize Node
        """
        rospy.init_node('wscr_aimlab')

        #--- Initialize Vision and Control Components
        self.arm = Arm()
        self.eyes = Eyes()

        #--- Initialize Publishers and Subscribers
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.last_image = None

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.last_scan = None

        #--- Warmup Sleep
        rospy.sleep(3)
        

    """
    Callback Methods
    """
    def image_callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.last_image = image

    def scan_callback(self, data):
        self.last_scan = data


    """
    Run Method
    """
    def run(self):
        rospy.spin()


if __name__=="__main__":
    node = Aimlab()
    node.run()