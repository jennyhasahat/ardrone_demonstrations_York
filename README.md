ARDrone Demonstrations York
====================

A collection of swarm robot demonstrations using Parrot AR Drones.

This code assumes the following packages are installed:
1. ROS: www.ros.org
2. ardrone_autonomy: [ardrone_autonomy rosbuild](https://github.com/AutonomyLab/ardrone_autonomy) [ardrone_autonomy catkin](https://github.com/jennyhasahat/ardrone_autonomy)
3. tum_ardrone: [tum_ardrone rosbuild](https://github.com/tum-vision/tum_ardrone) [tum_ardrone catkin](https://github.com/jennyhasahat/tum_ardrone)

There are 3 demonstrations included in this ROS package:
* A simple joystick controller of one drone
* A dance to the Blue Danube tune
* Joystick controlled swarm aggregation

Installation
------------

To install this package, navigate to the `src` folder in your ROS workspace:
```shell
git clone https://github.com/jennyhasahat/ardrone_demonstrations_York.git
```
Then navigate to the top level folder of the ROS workspace and run 
```shell
catkin_make
```
If you find the installation process doesn't work, make sure that the ardrone_autonomy and tum_ardrone drivers are using the catkin build system, otherwise catkin will not build this package.

Using Multiple Drones
---------------------

To control multiple drones with ROS, you can set each drone to join the same ad-hoc wifi network, instead of each drone creating its own ad-hoc wifi network. Instructions on doing this can be found at:
* https://github.com/AutonomyLab/ardrone_autonomy/wiki/Multiple-AR-Drones

To then control the drones, you can connect your computer to the drones' ad-hoc network and run the ROS launch scripts.
These launch scripts are optimised for the swarm set up at the University of York, but can be easily modified to any swarm of ARDrones by altering the IP addresses used to launch ardrone_autonomy drivers.




