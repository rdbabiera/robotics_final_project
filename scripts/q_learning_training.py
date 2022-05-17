#!/usr/bin/env python3

from collections import namedtuple
import rospy
import cv2, cv_bridge
import numpy as np
import math
import os

from robotics_final_project.srv import DeepQLearning, DeepQLearningResponse

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
    def __init__(self, h, w):
        """
        Initialize Node
        """
        rospy.init_node('wscr_q_trainer')

        """
        Image Parameters
        """

        # Camera is 640 (w) 480 (h)
        self.h = h
        self.w = w

        """
        Model Parameters
        """
        self.batch_size = 1
        self.gamma = 0.999
        self.eps_start = 0.9
        self.eps_end = 0.05
        self.eps_decay = 200
        self.target_update = 10

        self.steps_done = 0

        self.memory = ReplayMemory(5000)
        self.optimizer = optim.RMSprop(self.model.parameters())

        self.model = AimlabNetwork()

    def select_action(self, state):
        sample = random.random()
        eps_threshold = self.eps_end + (self.eps_start - self.eps_end) * \
            math.exp(-1. * self.steps_done / self.eps_decay)
        self.steps_done += 1
        if sample > eps_threshold:
            with torch.no_grad:
                return self.model(state)
        else:
            random_action = torch.rand((1, 1, self.h, self.w))
            random_action = F.softmax(random_action, dim=1)
            return random_action

    def optimize(self):
        if len(self.memory) < self.memory.capacity:
            return
        transitions = self.memory.sample(self.batch_size)
        batch = Transition(*zip(*transitions))
        non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                        batch.next_state,)), dtype=torch.bool)
        non_final_next_states = torch.cat([s for s in batch.next_state
                                                if s is not None])
        state_batch = torch.cat(batch.state)
        action_batch = torch.cat(batch.action)
        reward_batch = torch.cat(batch.reward)

        # Compute Q(s_t, a)
        state_action_values = self.model(state_batch).gather(1, action_batch)

        # Compute V(s_{t+1}) for all next states.
        next_state_values = torch.zeros(self.batch_size, device=device)
        next_state_values[non_final_mask] = self.target_model(non_final_next_states).max(1)[0].detach()
        # Compute expected Q
        expected_state_action_values = (next_state_values * self.gamma) + reward_batch
        
        # Compute Huber loss
        criterion = nn.SmoothL1Loss()
        loss = criterion(state_action_values, expected_state_action_values.unsqueeze(1))
        
        # Optimize Model
        self.optimizer.zero_grad()
        loss.backward()
        for param in self.model.parameters():
            param.grad.data.clamp_(-1, 1)
        self.optimizer.step()

    

    def train(self):
        num_episodes = 10000
        for i_episode in range(num_episodes):
            if i_episode % 1000 == 0:
                print(f"{i_episode}")

            # Receive Fresh State
            state = None

            # While not Done
            done = False
            while not done:
                # Select and perform an action
                action = self.select_action(state)
    
    def perform_action(self):
        pass
                
            

    def run(self):
        torch.save(self.model.state_dict(), model_path + 'dqn_model.pt')
        pass


if __name__=="__main__":
    node = QTrainer()
    node.run()