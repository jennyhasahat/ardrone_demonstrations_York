ARDrone Demonstrations York
====================

A collection of swarm robot demonstrations using Parrot AR Drones.

This code assumes the following packages are installed:
* ROS: www.ros.org
* ardrone_autonomy: [ardrone_autonomy rosbuild](https://github.com/AutonomyLab/ardrone_autonomy) [ardrone_autonomy catkin](https://github.com/jennyhasahat/ardrone_autonomy)
* tum_ardrone: [tum_ardrone rosbuild](https://github.com/tum-vision/tum_ardrone) [tum_ardrone catkin](https://github.com/jennyhasahat/tum_ardrone)

There are 3 demonstrations included in this ROS package:
* A simple joystick controller of one drone
* A dance to the Blue Danube tune
* Joystick controlled swarm aggregation

Installation
------------

To install this package, navigate to the `src` folder in your ROS workspace:
```sh
git clone https://github.com/jennyhasahat/ardrone_demonstrations_York.git
```
Then navigate to the top level folder of the ROS workspace and run 
```sh
catkin_make
```
If you find the installation process doesn't work, make sure that the ardrone_autonomy and tum_ardrone drivers are using the catkin build system, otherwise catkin will not build this package.

Joystick Control of One Drone
--------------------------------

This demonstration requires one drone.
1. Connect to the drone over wifi
2. Start the ROS core by entering `roscore` into a terminal
3. In a new terminal tab or window launch `joystick-controlled-drone.launch`:
```sh
roslaunch ardrone_demonstrations_york joystick-controlled-drone.launch
```
You can edit the mapping between joystick buttons and axes by editing the launch file. Current settings are optimised for the Logitech Extreme 3D pro joystick. A guide to setting the joystick mappings can be found at <http://robohub.org/up-and-flying-with-the-ar-drone-and-ros-joystick-control/>. This demonstration is essentially the same as the tutorial on that robohub website.

Blue Danube Dance
-------------------

### Set up

This demonstration requires you to have [set up multiple drones](#using-multiple-drones).

The driver launch files needs to be edited to your particular wifi set up. The example files `bd_64.xml`, `bd_66.xml` and `bd_68.xml` can be copied and edited so that the IP address arguement the node is given on initialisation, is the correct IP address of the drone you want to connect to. 
So:
```xml
<node name="ardrone_driver" pkg="ardrone_autonomy" type="ardrone_driver" output="screen" clear_params="true"  args="-ip 192.168.1.64">
	...
	...
</node>
```
The `args="-ip 192.168.1.64"` bit is the part that should be changed, the rest should be left the same.


The `bluedanube_driver.launch` file should then be edited so that the `dronesL` group `<include>s` the xml files for the drones that will do the left hand side dance, and the `dronesR` group `<include>s` the xml files for the drones that will do the right hand side dance.

### Execution
In a terminal window, `cd` into the ARDrone Demonstrations York directory.
Run each of the following commands in a different terminal tab:
* `roscore`
* `roslaunch bluedanube_driver.launch`
* `sh bluedanube.sh`
The first line launches ROS, the second command starts the ardrone_autonomy driver. The last line runs a script that will start playing the blue danube tune, and starts the dance.

Using Multiple Drones
---------------------

To control multiple drones with ROS, you can set each drone to join the same ad-hoc wifi network, instead of each drone creating its own ad-hoc wifi network. Instructions on doing this can be found at:
* https://github.com/AutonomyLab/ardrone_autonomy/wiki/Multiple-AR-Drones

To then control the drones, you can connect your computer to the drones' ad-hoc network and run the ROS launch scripts.
These launch scripts are optimised for the swarm set up at the University of York, but can be easily modified to any swarm of ARDrones by altering the IP addresses used to launch ardrone_autonomy drivers.




