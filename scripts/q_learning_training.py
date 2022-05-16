#!/usr/bin/env python3

from collections import namedtuple
import rospy
import cv2, cv_bridge
import numpy as np
import math
import os

# Import ML Libraries
from collections import namedtuple, deque
import random

from dqn_model import ReplayMemory, AimlabNetwork, Transition

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torchvision.transforms as T
import torch.distributions as distributions

model_path = os.path.dirname(__file__) + "/models/"


class QTrainer(object):
    def __init__(self, h, w, outputs):
        """
        Initialize Node
        """
        rospy.init_node('wscr_q_trainer')

        """
        Model Parameters
        """
        self.batch_size = 16
        self.gamma = 0.999
        self.eps_start = 0.9
        self.eps_end = 0.05
        self.eps_decay = 200
        self.target_update = 10

        self.memory = ReplayMemory(5000)
        self.optimizer = optim.RMSprop(self.model.parameters())

        self.model = AimlabNetwork(1, 1, 1)


    def run(self):
        torch.save(self.model.state_dict(), model_path + 'dqn_model.pt')
        pass


if __name__=="__main__":
    node = QTrainer()
    node.run()