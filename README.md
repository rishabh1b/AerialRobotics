# AerialRobotics
This repository has the code base for the projects accomplished as part of the course _CMSC828T:Planning, Perception and control for Aerial Robots_ at the University of Maryland. _MATLAB_ is the language used for all the implementations. 
### Trajectory Planning for Helical and Diamond Trajectories
---------------------------------------
The folder P1Ph2 in P1 involves code to generate a diamond and helical trajectory and tuned controllers to achieve it.

| Diamond Trajectory  | Helical Trajectory|
| ------------------- | ----------------- |
| <img src="https://github.com/rishabh1b/AerialRobotics/blob/master/P1/P1Ph2/gifs/diamond.gif" alt="step" width="400">| <img src="https://github.com/rishabh1b/AerialRobotics/blob/master/P1/P1Ph2/gifs/helical.gif" alt="step" width="400"/>|

### Trajectory Planning and control through known environment
The code for simulating a drone to go from one point to other in a known environment while avoiding obstacles is located in folder P1Ph3 of folder P1. The folder essentially has the implementation for Dijkstra algorithm in 3D, a path refinement algorithm to filter points from the output of Dijkstra to fit smooth polynomials and a generic algorithm to fit fifth order and seventh order polynomials through the refined waypoints. 

 <img src="https://github.com/rishabh1b/AerialRobotics/blob/master/P2/gifs/map2Output.gif" alt="step" width="400"> <img src="https://github.com/rishabh1b/AerialRobotics/blob/master/P2/gifs/map3Output.gif" alt="step" width="400">

### SLAM 
---------------------------------------
An implementation of SLAM using April tags for good features and standard trajectories like straight line and square followed on top of these April tags. The code is located in the folder P2. Bundle Adjustment and smoothing is performed using a factor graph. The factor graph is created using the GTSAM toolbox. This toolbox is essentially a dependency for the code to work correctly. The toolbox can be downloaded from [here](https://research.cc.gatech.edu/borg/story/gtsam-320-release).

 <img src="https://github.com/rishabh1b/AerialRobotics/blob/master/P2/plots/MountainXZ.jpg" alt="step" width="400"> <img src="https://github.com/rishabh1b/AerialRobotics/blob/master/P2/plots/StraightLineXY.jpg" alt="step" width="400"> 
 <img src="https://github.com/rishabh1b/AerialRobotics/blob/master/P2/plots/mappingXY.jpg" alt="step" width="400"> <img src="https://github.com/rishabh1b/AerialRobotics/blob/master/P2/plots/slowcircleXY.jpg" alt="step" width="400"> 
 
 ### EKF
 Folder ekf is an attempt to implement an Extended Kalman Filter to fuse the IMU and camera readings to obtain better state estimate of the drone. We use essentially using a 15x1 state vector. The parameters in the implemention needs further tuning. 

