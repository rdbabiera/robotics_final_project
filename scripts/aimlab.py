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

        #--- Initialize Publishers and Subscribers
        rospy.Subscriber('/robot_vision', ListVisionCoords, self.vision_received)
        self.arm_pub = rospy.Publisher('/robot_arm_action', VisionCoords, queue_size=10)

        #--- Warmup Sleep
        rospy.sleep(2)
        print("[AIMLAB] Ready to Go!")
        

    """
    Callback Methods
    """
    
    def vision_received(self, data):
        rate = rospy.Rate(5)

        # If No Data, then stare down middle of frame
        if len(data.positions) == 0:
            blank_target = VisionCoords(239, 319, 1)
            self.arm_pub.publish(blank_target)
            return

        # If Data Exists, then Normalize
        state = []
        max_depth = 0
        for row in data.positions:
            print('row:', row)
            if row.depth > max_depth:
                max_depth = row.depth
        
        for row in data.positions:
            state.append([row.y/480, row.x/640, row.depth/max_depth])

        # Convert Tensor to Torch, Get Action from Pretrained Model
        state = torch.tensor(state, dtype=torch.double)
        state = state.reshape(1, -1, 3)

        action = self.model(state.float()).max(1)[1].view(1, 1).item()

        # Get Target Info, Send to Arm Controller
        target = data.positions[action]
        target_location = VisionCoords(target.x, target.y, target.depth)

        self.arm_pub.publish(target_location)

        return


    """
    Run Method
    """
    def run(self):
        rospy.spin()


if __name__=="__main__":
    node = Aimlab()
    node.run()