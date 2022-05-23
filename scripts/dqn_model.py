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

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torchvision.transforms as T
import torch.distributions as distributions

class AimlabNetwork(nn.Module):
    def __init__(self):
        super(AimlabNetwork, self).__init__()
        self.input_channels = 1

        self.lstm = nn.LSTM(3, 3, num_layers = 1, batch_first=True, bidirectional=True)
        self.encoder = nn.ModuleList([
            nn.Linear(6, 1),
            nn.ReLU(),
        ])

    def forward(self, x):
        out, hidden = self.lstm(x)
        for i, l in enumerate(self.encoder):
            out = l(out)
        out = out.flatten(1)

        return out