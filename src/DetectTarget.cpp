#include <ros/ros.h>
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_autonomy/navdata_vision_detect.h"
#define IS_ARDRONE1


void navdata_callback(const ardrone_autonomy::Navdata::ConstPtr& msg_in)
{
	//Take in state of ardrone
	//ROS_INFO("detected %d tags", msg_in.tags_count);
	/*for(int i=0; i<msg_in.tags_count; i++)
	{
		ROS_INFO("tag type\t:\t %d", msg_in.tags_type[i]);
		ROS_INFO("tag location\t:\t (%d, %d)", msg_in.tags_xc[i], msg_in.tags_yc[i]);
		ROS_INFO("tag distance\t:\t %f", msg_in.tags_distance[i]);
	}*/
}

void navdata_vision_detect_callback(const ardrone_autonomy::navdata_vision_detect::ConstPtr& msg_in)
{
	//Take in state of ardrone
	//ROS_INFO("detected %d tags", msg_in.nb_detected);
	for(int i=0; i<msg_in->nb_detected; i++)
	{
		ROS_INFO("tag type\t:\t %d", msg_in->type[i]);
		ROS_INFO("tag location\t:\t (%d, %d)", msg_in->xc[i], msg_in->yc[i]);
		ROS_INFO("tag distance\t:\t %d", msg_in->dist[i]);
	}
}

/**This program will print detected tag data to stdout.
 * Make sure that the enable_navdata_vision_detect parameter is set to
 * true in your launch file
 * */
int main(int argc, char** argv)
{
	ROS_INFO("DetectTarget");
	ros::init(argc, argv, "DetectTarget");

	ros::NodeHandle node;
	ros::Rate loop_rate(50);	//update at 50Hz
	//ros::Subscriber sub1 = node.subscribe("/ardrone/navdata", 1, navdata_callback);
	ros::Subscriber sub2 = node.subscribe("/ardrone/navdata_vision_detect", 1, navdata_vision_detect_callback);
	ROS_INFO("Initialised DetectTarget");
	
	while (ros::ok())
	{
		//Do some stuff
		ros::spinOnce();	//refreshes subscriptions
		loop_rate.sleep();
	}
}
