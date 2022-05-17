#!/usr/bin/env python3

import rospy
import cv2, cv_bridge
import numpy as np
import math
import os

from robotics_final_project.srv import DeepQLearning, DeepQLearningResponse

class Balloon(object):
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

    def determine_hit(self, coords):
        dist = np.linalg.norm(self.center - coords)
        if dist <= self.radius:
            return True
        return False
    
    def determine_prize(self, coords, prev_coords, shape):
        dist = np.linalg.norm(self.center - coords)
        prize = 176 - (2 * self.radius)
        scaling = 1 - (dist / self.radius)
        h_flick = abs(prev_coords[0] - coords[0]) / (0.5 * shape[0])
        w_flick = abs(prev_coords[1] - coords[1]) / (0.5 * shape[1])
        prize += min(h_flick * 50, 50) + min(w_flick * 50, 50)

        return prize * scaling


class QRewarder(object):
    def __init__(self):
        """
        Initialize Node
        """
        rospy.init_node('wscr_q_rewards')

        self.current_view = None
        self.current_balloons = {}
        self.last_shot = None

    def handle_request(self, request):
        response = DeepQLearningResponse()
        response.done = False
        response.reward = 0

        action = np.array(response.action)
        b, d, h, w = action.shape
        # if first frame, make a new game
        if request.start:
            self.current_view = np.zeros((d, h, w), dtype=np.int8)
            self.current_balloons = {}
            self.last_shot = np.array([h//2, w//2])

            # Generate Balloons
            #--- 1. Choose Radius
            #--- 2. Choose Position
            #--- 3. Draw on View
            #--- 4. Add to Dictionary
            for i in range(0, 3):
                rad = np.random.randint(38, 80)

                y_coord = np.random.randint(20, h-20)
                x_coord = np.random.randint(20, w-20)
                pos = np.array([x_coord, y_coord])

                bloon = Balloon(pos, rad)

                for y in range(y_coord - rad, y_coord + rad):
                    if y < 0 or y >= h:
                        continue
                    for x in range(x_coord - rad, x_coord + rad):
                        if x < 0 or x >= h:
                            continue
                        curr = np.array([y, x])
                        if np.linalg.norm(pos - curr) <= rad:
                            self.current_view[0, y, x] = 1

                self.current_balloons[i] = bloon

            response.reward = 0
            response.done = False
            response.next_state = self.current_view

        else:
            # Get Coordinate of Attack
            max_action = np.where(action == np.amax(action))
            action = np.array([max_action[0], max_action[1]])

            to_remove = []

            for i in self.current_balloons:
                balloon = self.current_balloons[i]
                if balloon.determine_hit(action):
                    # Determine Reward
                    response.reward += balloon.determine_prize(action, 
                        self.last_shot, np.array([h, w]))

                    # Remove From Grid/Dictionary
                    for y in range(balloon.center[0] - balloon.radius, balloon.center[0] + balloon.radius):
                        if y < 0 or y >= h:
                            continue
                        for x in range(balloon.center[1] - balloon.radius, balloon.center[1] + balloon.radius):
                            if x < 0 or x >= w:
                                continue
                            self.current_view[0, y, x] = 0
                    
                    to_remove.append(i)
                
            for idx in to_remove:
                self.current_balloons.pop(idx)

            self.last_shot = action
            
            response.next_state = self.current_view
            
            if len(self.current_balloons) == 0:
                response.done = True
        
        return response


    def run(self):
        s = rospy.Service('action_request_service', DeepQLearning, self.handle_request)
        print("[SERVER] Ready to Handle Requests")
        rospy.spin()


if __name__=="__main__":
    node = QRewarder()
    node.run()