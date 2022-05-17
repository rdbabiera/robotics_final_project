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

model_path = os.path.dirname(__file__) + "/models/"

Transition = namedtuple('Transition', ('state', 'action', 'reward', 'next_state'))

class ReplayMemory(object):
    def __init__(self, capacity, batch_size):
        self.memory = deque([],maxlen=capacity)
        self.capacity = capacity
        self.batch_size = batch_size

    def push(self, *args):
        """Save a transition"""
        self.memory.append(Transition(*args))

    def sample(self):
        return random.sample(self.memory, self.batch_size)

    def __len__(self):
        return len(self.memory)


class AimlabNetwork(nn.Module):
    def __init__(self):
        super(AimlabNetwork, self).__init__()
        self.input_channels = 1
        self.output_channels = 1
        self.hidden_size_1 = 32
        self.hidden_size_2 = 64
        self.hidden_size_3 = 128

        self.encoder = nn.ModuleList([
            #--- Stage 1
            nn.BatchNorm2d(self.hidden_size_1), 
            nn.ReLU(),
            nn.Conv2d(self.hidden_size_1, self.hidden_size_1, kernel_size=3, padding=1), 
            nn.BatchNorm2d(self.hidden_size_1), 
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2, return_indices = True), 
            #--- Stage 2
            nn.Conv2d(self.hidden_size_1, self.hidden_size_2, kernel_size=3, padding=1), 
            nn.BatchNorm2d(self.hidden_size_2), 
            nn.ReLU(),
            nn.Conv2d(self.hidden_size_2, self.hidden_size_2, kernel_size=3, padding=1), 
            nn.BatchNorm2d(self.hidden_size_2), 
            nn.ReLU(),
            nn.Conv2d(self.hidden_size_2, self.hidden_size_2, kernel_size=3, padding=1), 
            nn.BatchNorm2d(self.hidden_size_2), 
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2, return_indices = True), 
            #--- Stage 3
            nn.Conv2d(self.hidden_size_2, self.hidden_size_3, kernel_size=3, padding=1), 
            nn.BatchNorm2d(self.hidden_size_3), 
            nn.ReLU(),
            nn.Conv2d(self.hidden_size_3, self.hidden_size_3, kernel_size=3, padding=1), 
            nn.BatchNorm2d(self.hidden_size_3), 
            nn.ReLU(),
            nn.Conv2d(self.hidden_size_3, self.hidden_size_3, kernel_size=3, padding=1), 
            nn.BatchNorm2d(self.hidden_size_3), 
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2, return_indices = True)
        ])

        self.decoder = nn.ModuleList([                                              
            #--- Stage 3-2
            nn.MaxUnpool2d(kernel_size=2, stride=2), 
            nn.ConvTranspose2d(self.hidden_size_3, self.hidden_size_3, kernel_size=3, padding=1), 
            nn.BatchNorm2d(self.hidden_size_3),
            nn.ReLU(),  
            nn.ConvTranspose2d(self.hidden_size_3, self.hidden_size_3, kernel_size=3, padding=1), 
            nn.BatchNorm2d(self.hidden_size_3), 
            nn.ReLU(),               
            nn.ConvTranspose2d(self.hidden_size_3, self.hidden_size_2, kernel_size=3, padding=1), 
            nn.BatchNorm2d(self.hidden_size_2), 
            nn.ReLU(),
            #--- Stage 2-2
            nn.MaxUnpool2d(kernel_size=2, stride=2),
            nn.ConvTranspose2d(self.hidden_size_2, self.hidden_size_2, kernel_size=3, padding=1),
            nn.BatchNorm2d(self.hidden_size_2), 
            nn.ReLU(),
            nn.ConvTranspose2d(self.hidden_size_2, self.hidden_size_2, kernel_size=3, padding=1),
            nn.BatchNorm2d(self.hidden_size_2), 
            nn.ReLU(),
            nn.ConvTranspose2d(self.hidden_size_2, self.hidden_size_1, kernel_size=3, padding=1), 
            nn.BatchNorm2d(self.hidden_size_1), 
            nn.ReLU(),
            #--- Stage 1-2
            nn.MaxUnpool2d(kernel_size=2, stride=2), 
            nn.ConvTranspose2d(self.hidden_size_1, self.hidden_size_1, kernel_size=3, padding=1),
            nn.BatchNorm2d(self.hidden_size_1), 
            nn.ReLU(),
            nn.ConvTranspose2d(self.hidden_size_1, self.output_channels, kernel_size=3, padding=1), 
            nn.BatchNorm2d(self.output_channels), 
            nn.Softmax(dim=1)
        ])

    def forward(self, x):
        stack = []
        # Forward
        out = x
        for i, l in enumerate(self.encoder):
            if isinstance(l, nn.MaxPool2d): 
                out, indices = l(out)
                stack.append(indices)
            else:
                out = l(out)
        
        # Backward
        for i, l in enumerate(self.decoder):
            if isinstance(l, nn.MaxUnpool2d): 
                indices = stack.pop()
                out = l(out, indices)
            else:
                out = l(out)

        return out
