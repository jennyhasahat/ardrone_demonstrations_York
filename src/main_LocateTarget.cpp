#include <ros/ros.h>
#include "LocateTarget.h"

int main(int argc, char** argv)
{
	ROS_INFO("Locate Target");
	ros::init(argc, argv, "LocateTarget");

	LocateTarget lt;
	lt.run();

	return 0;
}



