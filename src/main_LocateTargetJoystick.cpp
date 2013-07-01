#include <ros/ros.h>
#include "LocateTarget.h"


int main(int argc, char** argv)
{
	ROS_INFO("Locate Target Joystick Control");
	ros::init(argc, argv, "LocateTargetJoystickControl");

	LocateTargetJoystickControl lt;
	lt.run();

	return 0;
}

