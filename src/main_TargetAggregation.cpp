#include <ros/ros.h>
#include "TargetAggregation.h"

int main(int argc, char** argv)
{
	ROS_INFO("Locate Target");
	ros::init(argc, argv, "LocateTarget");

	TargetAggregation lt;
	lt.run();

	return 0;
}



