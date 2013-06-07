#include <ros/ros.h>


int main(int argc, char** argv)
{
	ROS_INFO("DetectTarget");
	ros::init(argc, argv, "DetectTarget");

	ros::NodeHandle node;
    ros::Rate loop_rate(50);	//update at 50Hz

    while (ros::ok())
    {
    	//Do some stuff

    	ros::spinOnce();	//refreshes subscriptions
    	loop_rate.sleep();
    }

    return 0;
}
