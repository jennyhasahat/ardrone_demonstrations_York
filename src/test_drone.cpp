#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include "Drone.h"
#include <ardrone_autonomy/navdata_demo.h>
#include "ardrone_autonomy/navdata_vision_detect.h"

void navdata_demo_callback(const ardrone_autonomy::navdata_demo::ConstPtr& raw_msg)
{
	ROS_INFO("test_drone successfully received navdata demo message");
}

void navdata_vd_callback(const ardrone_autonomy::navdata_vision_detect::ConstPtr& msg)
{
	//ROS_INFO("test_drone successfully received navdata vision detect message");
}

//front camera size is 320 x 240??

int main(int argc, char** argv)
{
	ROS_INFO("test flight");
	ros::init(argc, argv, "TestFlight");
	Drone d;
	//d.addNavdataDemoCallback(navdata_demo_callback);
	d.addNavdataVisionDetectCallback(navdata_vd_callback);

	/*d.takeOff(5);
	d.doLEDAnimation(Drone::LED_RED, 0, 0);

	ROS_INFO("yaw right");
	d.setSpeeds(0,0,-0.5,0);
	d.waitSeconds(1);

	ROS_INFO("yaw left");
	d.setSpeeds(0,0,0.5,0);
	d.waitSeconds(1);


	ROS_INFO("pitch forward");
	d.setSpeeds(0,0.25,0,0);
	d.waitSeconds(0.5);

	ROS_INFO("strafe left");
	d.setSpeeds(0.25,0,0,0);
	d.waitSeconds(0.5);

	ROS_INFO("land");
	d.land(5);
	d.doLEDAnimation(Drone::LED_GREEN, 0, 0);*/




	return 0;
}
