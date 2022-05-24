#!/usr/bin/env python3

import rospy
import cv2, cv_bridge
import numpy as np
import math
import os

import torch

from arm_controller import Arm
from vision_controller import ObjectDetector

from dqn_model import AimlabNetwork

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from robotics_final_project.msg import ListVisionCoords, VisionCoords

# Path of directory to locate where to find dqn models
model_path = os.path.dirname(__file__) + "/models/"


class Aimlab(object):
    def __init__(self):
        """
        Initialize Node
        """
        rospy.init_node('wscr_aimlab')

        #--- Initialize Model
        self.model = AimlabNetwork()
        self.model.load_state_dict(torch.load(model_path + 'aimlab_model.pt'))
        self.model.eval()

        #--- Initialize Vision and Control Components
        self.arm = Arm()
        self.object_detector = ObjectDetector()

        #--- Initialize Publishers and Subscribers
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.last_image = None

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.last_scan = None

        rospy.Subscriber('/robot_vision', ListVisionCoords, self.vision_received)
        self.arm_pub = rospy.Publisher('/robot_arm_action', VisionCoords, queue_size=10)

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
    
    def vision_received(self, data):
        #model stuff
        #self.arm_pub.publish(whatever visioncoord model decides)
        pass


    """
    Run Method
    """
    def run(self):
        rospy.spin()


if __name__=="__main__":
    node = Aimlab()
    node.run()