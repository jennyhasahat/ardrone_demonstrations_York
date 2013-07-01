#include <ros/ros.h>
#include "TargetAggregation.h"


int main(int argc, char** argv)
{
	ROS_INFO("Locate Target Joystick Control");
	ros::init(argc, argv, "LocateTargetJoystickControl");

	TargetAggregationJoystickControl lt;
	lt.run();

	return 0;
}

