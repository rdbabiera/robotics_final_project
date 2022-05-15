#!/usr/bin/env python3

import rospy
import cv2, cv_bridge
import numpy as np
import math
import os

class QRewarder(object):
    def __init__(self):
        """
        Initialize Node
        """
        rospy.init_node('wscr_q_rewards')