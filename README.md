# Will Smith Robot Slaps Chris Rock Balloon
A CMSC 20600 Final Project, spring 2022

## Team Members
RD Babiera, Jason Chee, David Pan, Kendrick Xie

## Project Description
### Goals

write here

### Why this is interesting

write here

### Accomplishments

write here


### Project Diagram

write here

<hr>

## System Architecture
### Computer Vision

### Q Learning
For the Q-Learning componenent of the project, we decided to explore a Machine 
Learning approach in order to account for the sheer amount of states traditional 
Q-Learning may have to account for. In Deep Q-Learning, we train a model to 
approximate the Q-Value of a state. 

The code that trains the model is found in q_learning_notebooks_points.ipynb. 
The original code utilized a service between two ROS nodes in which one trained 
the model and sent commands to a game server, while the game server returned 
rewards, however that proved to be inefficient due to ROS quickly reaching 
allocated memory caps. The Python notecode contains balloon and game classes used 
to simulate popping 3 randomly generated balloons on a screen as well as the 
DQN model and the required infrastructure to train in.

Once the model is completely trained, the aimlab.py script acts as the 'brain' 
of the TurtleBot. Upon receiving a VisionCoordsList message from the Vision 
Controller, the Aimlab model normalizes the data for the model, passes these 
states to the model, and given an index, creates a VisionCoords message with 
the coordinates of the corresponding balloon. The system then proceeds with the 
Kinematics Controller.

### Kinematics

<hr>

## Code Execution

write here

<hr>

## Challenges

write here

<hr>

## Future Work

write here

<hr>

## Takeaways

write here