#include <ros/ros.h>
#include "Drone.h"
#include <ardrone_autonomy/navdata_demo.h>
#include "ardrone_autonomy/navdata_vision_detect.h"

void navdata_demo_callback(const ardrone_autonomy::navdata_demo::ConstPtr& raw_msg)
{
	ROS_INFO("test_drone successfully received navdata demo message");
}

void navdata_vd_callback(const ardrone_autonomy::navdata_vision_detect::ConstPtr& msg)
{
	ROS_INFO("test_drone successfully received navdata vision detect message");
}

int main(int argc, char** argv)
{
	ROS_INFO("Using subscriptions demo");
	ros::init(argc, argv, "UsingSubscriptionsDemo");
	Drone d;
	d.addNavdataDemoCallback(navdata_demo_callback);
	d.addNavdataVisionDetectCallback(navdata_vd_callback);

	ros::spin(); //checks subscriptions forever

	return 0;
}
