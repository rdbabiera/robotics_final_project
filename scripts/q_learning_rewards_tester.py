#!/usr/bin/env python3

import rospy
import cv2, cv_bridge
import numpy as np

from robotics_final_project.srv import DeepQLearning, DeepQLearningReward
from q_learning_rewards import Balloon, QRewarder

class QRewarderTest(object):
    def __init__(self, h, w):
        
        rospy.init_node('wscr_q_rewarder_test')

        # set up reward service proxy
        rospy.wait_for_service('action_request_service')
        self.q_rewarder = rospy.ServiceProxy('action_request_service', DeepQLearning)

        # keep track of the state of the current game
        self.current_state = np.zeros((1, h, w), dytpe=np.int8)
        self.game_done = True
        self.h = h
        self.w = w


    def run(self):
        # make 1 shot per second
        r = rospy.Rate(1)
        while True:
            # send a request to the Q rewarder node
            request = DeepQLearning()
            if self.game_done:
                # game is done, send request to start new game
                request.start = True
                request.action = np.zeros((1, self.h, self.w), dtype=np.int8) # TODO change to message format
                print('Sending request to start new game')
            else:
                request.start = False
                # put my next action in the request (random shot)
                action = np.zeros((1, self.h, self.w), dtype=np.int8)
                shot_h = np.random.randint(self.h)
                shot_w = np.random.randint(self.w)
                action[shot_h, shot_w] = 1
                request.action = action # TODO convert into message format
                print(f'Shooting at position h = {shot_h}, w = {shot_w}')

            # get response and update knowledge of game state
            response = self.q_rewarder(request)
            self.current_state = response.next_state # TODO convert from message format
            if self.game_done:
                print('New state:')
                print(self.current_state)
            if response.reward > 0:
                print(f'Hit balloon, received reward {response.reward}')
                print('New state:')
                print(self.current_state)
            self.game_done = response.done

            r.sleep()


if __name__ == '__main__':
    node = QRewarderTest(480, 640) # might want to reduce for testing
    node.run()
