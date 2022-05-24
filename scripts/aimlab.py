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
        rate = rospy.Rate(5)

        # If No Data, then stare down middle of frame
        if len(data.positions) == 0:
            blank_target = VisionCoords(239, 319, 1)
            for i in range(0, 10):
                self.arm_pub.publish(blank_target)
                rate.sleep()
            return

        # If Data Exists, then Normalize
        state = []
        max_depth = 0
        for row in data.positions:
            if row[2] > max_depth:
                max_depth = row[2]
        for row in data.positions:
            state.append([row[0]/480, row[1]/640, row[2]/max_depth])

        # Convert Tensor to Torch, Get Action from Pretrained Model
        state = torch.tensor(state, dtype=torch.double)
        state = state.reshape(1, -1, 3)

        action = self.model(state.float()).max(1)[1].view(1, 1).item()

        # Get Target Info, Send to Arm Controller
        target = data.positions[action]
        target_location = VisionCoords(target[0], target[1], target[2])

        for i in range(0, 10):
            self.arm_pub.publish(target_location)
            rate.sleep()

        return


    """
    Run Method
    """
    def run(self):
        rospy.spin()


if __name__=="__main__":
    node = Aimlab()
    node.run()