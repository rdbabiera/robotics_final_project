# Will Smith Robot Slaps Chris Rock Balloon
A CMSC 20600 Final Project, spring 2022

## Team Members
RD Babiera, Jason Chee, David Pan, Kendrick Xie

## Project Description
### Goals

As an ode to our namesake, the first original goal of this project was to identify 
Chris Rock's face on a balloon and shoot it with a laser pointer. Because that 
project was mostly a computer vision problem, we have since changed our goal to 
have Turtlebot identify balloons and shoot them in order according to the "game" 
rules, which are as followed:

1. Each balloon gives a base score of 125 * (depth/max depth of all balloons)
2. Each shot gives a bonus of (x displacement / half the screen width) * 100, 
with a max bonus of 100. This bonus is identical in the y direction as well. 

Because of these rules, the game is not as simple as shooting the furthest 
balloon or shooting the furthest from the center balloons. Thus, Q-Learning must 
be used to order shots properly. Once balloons are identified, the arm must then 
properly aim at the correct coordinates.

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
DQN model and the required infrastructure to train.

The model architecture is as followed:

1. BiLSTM that takes in list of n points with 3 features (y, x, d), returns an 
(n, 6) tensor.
2. Linear Layer that takes in (n, 6) tensor and outputs (n, 1) scores.
3. ReLU activation that sets each score to max(0, x)

The model plays 10,000 randomly generated games 20 times so that it has the 
opportunity to explore each path in each game. State-action pairs are stored 
in memory, and at each optimization step one of the last 50 actions is randomly 
sampled and the Bellman Equation is used in order to calculate losses. The 
expected values are pulled from a model that lags 500 games behind the policy 
network in order to prevent immediate overfitting.

Once the model is completely trained, the aimlab.py script acts as the 'brain' 
of the TurtleBot. Upon receiving a VisionCoordsList message from the Vision 
Controller, the Aimlab model normalizes the data for the model, passes these 
states to the model, and given an index, creates a VisionCoords message with 
the coordinates of the corresponding balloon. The system then proceeds with the 
Kinematics Controller.

### Kinematics

The arm controller takes in x and y image coordinates and a corresponding depth
reading, and moves the arm so that the laser aims at that point in 3D space. The
position of the target point relative to the camera in spherical coordinates is 
caluculated using the coordinates and depth, as well as known values for camera's 
vertical and horizontal FOV. This point is then moved to account for the difference 
in position between the camera and the laser, which is done by converting it to
cartesian coordinates, then converting it back to spherical coordinates after the
adjustment. Once we know the angles we want to fire at, we aim the laser by 
manipulating the first and third arm joints: the first joint controls the 
left-and-right angle, and the third joint controls the up-and-down angle.

<hr>

## Code Execution

write here

<hr>

## Challenges
### Vision Challenges

### Q Learning Challenges
Desiging a Deep Q Learning Network is a problem much more complex than it may 
seem. The first design involved taking in the entire screen as input, identifying 
balloons as part of a computer vision problem, and then determining a score for 
each pixel in the grid to aim at. This quickly became problematic because at the 
evaluation step, given a state, the model will always give the same output. Thus, 
the game never completes if the model misses a shot. The expectation of the model 
slowly shifted as the vision component of the project improved; the model was then 
designed to determine the order of balloons to shoot such that it will never miss, 
but its goal became to optimize the score of the game. Because the number of 
features total became quickly limited to 3 per point (coordinates), the next 
challenge was designing the model to not overfit one result for every possible 
state. This was overcome by decreasing the number of total parameters present as 
well as normalizing the feature space to not only be agnostic to image resolutions 
of all kinds but also to understand the score space better in terms of 
depth-based scoring.

### Kinematics Challenges
While the theoretical foundations of our kinematics calculations are sound, the fact
that the laser travels across a long distance means that very small errors in the 
setup or measurements have a large impact on where the laser lands. Because of this,
we had to experiment with many different parameters, such as the FOV we used and the
precise camera position, to get the aim to be reasonably accurate, and the system is 
still fairly sensitive to small misalignments.

<hr>

## Future Work

Given 2-3 more weeks to work on this project, the most ambitious thing we could 
achieve is to live up to our group name and not only train a "Chris Rock classifier 
model", but also to have turtlebot drive up to said balloon and use the arm to 
slap it.

One method we considered for making the kinematics system more robust to error is 
using input from the camera to make small corrections to the aim. This would involve 
using our current kinematics calculations to aim close to the target point, then 
identifying where the laser is hitting on the camera feed and adjusting the aim
accordingly with a form of proportional control. This adjustment step could be done 
several times in a row until the desired accuracy is reached.

<hr>

## Takeaways

write here
