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

The driver launch files needs to be edited to your particular wifi set up. The example files [bd_64.xml](launch/bd_64.launch), [bd_66.xml](launch/bd_66.launch) and [bd_68.xml](launch/bd_68.launch) can be copied and edited so that the IP address arguement the node is given on initialisation, is the correct IP address of the drone you want to connect to. 
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
* `roslaunch ardrone_demonstrations_york bluedanube_driver.launch`
* `sh bluedanube.sh`

The first line launches ROS, the second command starts the ardrone_autonomy driver. The last line runs a script that will start playing the blue danube tune, and starts the dance.


Joystick Controlled Aggregation
-------------------------------

### Set up
This demonstration requires you to have [set up multiple drones](#using-multiple-drones).

As with the [joystick control of one drone demo](#joystick-control-of-one-drone), this code is optimised for the Logitech Extreme 3D pro joystick. The TUM autopilot specific buttons can be re-mapped by changing lines 146 to 149 of <src/joystick2TUMARDroneInterpreter.py>. The current mapping is:
* Send Coords to followers: 7
* Reset PTAM (the feature tracking): 8
* Land the follower drones: 9
* Start the follower drones: 10

The driver launch files needs to be edited to your particular wifi set up. The example files [ta_64.xml](launch/ta_64.launch), [ta_66.xml](launch/ta_66.launch) and [ta_68.xml](launch/ta_68.launch) can be copied and edited so that the IP address arguement the node is given on initialisation, is the correct IP address of the drone you want to connect to. 
So:
```xml
<node name="ardrone_driver" pkg="ardrone_autonomy" type="ardrone_driver" output="screen" clear_params="true"  args="-ip 192.168.1.64">
	...
	...
</node>
```
The `args="-ip 192.168.1.64"` bit is the part that should be changed, the rest should be left the same.


Finally, the <launch/autopilot_joystick_multi.launch> script needs to be updated so that the joystick-controlled leader drone uses the IP of the drone you want to be the leader. The follower drones should use the IP address of the drone you want to be a follower. An arbitary number of followers can be used by adding more groups like the `follower1` group and specifying different drones to be followers. Remember that the more drones there are in the swarm, the more likely they are to crash into each other.

### Execution
In a terminal window, `cd` into the ARDrone Demonstrations York directory.
Run each of the following commands in a different terminal tab:
* `roscore`
* `roslaunch ardrone_demonstrations_york autopilot_joystick_multi.launch`
* `rosrun ardrone_demonstrations_york TargetAggregation_joystick`

### Running the Demonstration
When you launch the drivers, several windows will open up. There should be a Map estimation, and (once the ardrone_autonomy driver has connected to the drone) a camera feed.

Running the TargetAggregation_joystick node will cause one of the drones to take off and initialise the TUM state estimation node. The initialisation will have finished when there are coloured blobs on the camera feed window relating to visual features of the image (for example <http://ros.org/wiki/tum_ardrone/drone_stateestimation#Video_Window>).

* Once the leader's state estimation has initialised, use the joystick to navigate the drone to some desired position, then press the button to start the follower drones (by default this should be the button labelled 10 on your joystick). 
* Once the _follower drone(s)_ has initialised their state estimation, press the joystick button to send the leader's coordinates to the followers (by default this should be the button labelled 7 on your joystick).
* Ideally, the followers will move to approximately where the leader drone is.
* You can move the leader drone and send more coordinates to the followers, and (ideally) they will move to that new position.
* To finish the demo press the button to land the followers (by default this should be the button labelled 9 on your joystick), and land the leader drone with the joystick.

Using Multiple Drones
---------------------

To control multiple drones with ROS, you can set each drone to join the same ad-hoc wifi network, instead of each drone creating its own ad-hoc wifi network. Instructions on doing this can be found at:
* <https://github.com/AutonomyLab/ardrone_autonomy/wiki/Multiple-AR-Drones>

To then control the drones, you can connect your computer to the drones' ad-hoc network and run the ROS launch scripts.
These launch scripts are optimised for the swarm set up at the University of York, but can be easily modified to any swarm of ARDrones by altering the IP addresses used to launch ardrone_autonomy drivers.




