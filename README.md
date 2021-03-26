# Autnomous_Robot-Pick_and_Place_System

### Introduction

This is the final project of the RBE3001 course. The main goal of the project was to build an autonomous sorting system. The robot had to utilize computer vision to detech and locate the different color balls within the robot's workspace, grab the balls by controlling the robot's end-effector tool and place them at a pre-determined position. The robotic arm had a 3 degree-of-freedom (DOF) motion. The locations of the balls in the robot's workspace was captured by the camera in pixel coordinates. To utilize these locations, the team had to calculate the transformations to turn the pixel values into the coordinates in reference to the robot's baseframe. The team then had to sort the balls using a motion trajectory planner and the gripper arm to manipulate them to their designated locations.
### Workflow:

1) Forward Kinematics and Joint-Space Motion Planning
2) Inverse Kinematics and Task-Space Motion Planning
3) Trajectory Planning
4) Computer Vision

### Forward Kinematics and Joint-Space Motion Planning

To calcauate the forward kinematics, the team used the Denavit-Hartenverg convention to create D-H parameter table. Using this table, the team calcuated the transforamtion matrix from the base frame of the robot to the end-effector.

### Inverse Kinematics and Task-Space Motion Planning

Given the three end-effector positions (x,y and z coordinates), the team calcuated the joint angles (in degrees) by formulating sets of equations for each joint angle. The team utilized the geometric approach and the robot's elbow-down configuration to formulate the sets of equations.

### Trajectory Planning:

The team approached the final project with a guard-free-guard motion. For the free motion, the team used the quintic trajectory to hover over the ball as it gave an addtional control over the acceleration of the robot's motion when compared to the cubic trajectory.

### Computer Vision:

The team used MATLAB's image processing and computer vision toolkit to calibrate the camera, mask the different colored balls on the robot's workspace, process the image to reduce the noise, and finally calcuate the coordinates of the balls.

## Link to the Demo:

https://youtu.be/7rwIkverpHw
