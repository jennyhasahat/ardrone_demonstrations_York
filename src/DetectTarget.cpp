#include <ros/ros.h>
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_autonomy/navdata_vision_detect.h"

/*
00  CAD_TYPE_HORIZONTAL = 0,           Deprecated
01  CAD_TYPE_VERTICAL,                 Deprecated
02  CAD_TYPE_VISION,                   Detection of 2D horizontal tags on drone shells
03  CAD_TYPE_NONE,                     Detection disabled
04  CAD_TYPE_COCARDE,                  Detects a roundel under the drone
05  CAD_TYPE_ORIENTED_COCARDE,         Detects an oriented roundel under the drone
06  CAD_TYPE_STRIPE,                   Detects a uniform stripe on the ground
07  CAD_TYPE_H_COCARDE,                Detects a roundel in front of the drone
08  CAD_TYPE_H_ORIENTED_COCARDE,       Detects an oriented roundel in front of the drone
09  CAD_TYPE_STRIPE_V,
10  CAD_TYPE_MULTIPLE_DETECTION_MODE,  The drone uses several detections at the same time
11  CAD_TYPE_CAP,                      Detects a Cap orange and green in front of the drone
12  CAD_TYPE_ORIENTED_COCARDE_BW,      Detects the black and white roundel
13  CAD_TYPE_VISION_V2,                Detects 2nd version of shell/tag in front of the drone

0  TAG_TYPE_NONE
1  TAG_TYPE_SHELL_TAG
2  TAG_TYPE_ROUNDEL
3  TAG_TYPE_ORIENTED_ROUNDEL
4  TAG_TYPE_STRIPE
5  TAG_TYPE_CAP
6  TAG_TYPE_SHELL_TAG_V2
7  TAG_TYPE_TOWER_SIDE
8  TAG_TYPE_BLACK_ROUNDEL
*/

void navdata_callback(const ardrone_autonomy::Navdata& msg_in)
{
	//Take in state of ardrone
	ROS_INFO("detected %d tags", msg_in.tags_count);
	for(int i=0; i<msg_in.tags_count; i++)
	{
		ROS_INFO("tag type\t:\t %d", msg_in.tags_type[i]);
		ROS_INFO("tag location\t:\t (%d, %d)", msg_in.tags_xc[i], msg_in.tags_yc[i]);
		ROS_INFO("tag distance\t:\t %f", msg_in.tags_distance[i]);
	}
}

void navdata_vision_detect_callback(const ardrone_autonomy::navdata_vision_detect& msg_in)
{
	//Take in state of ardrone
	ROS_INFO("detected %d tags", msg_in.nb_detected);
	for(int i=0; i<msg_in.nb_detected; i++)
	{
		ROS_INFO("tag type\t:\t %d", msg_in.type[i]);
		ROS_INFO("tag location\t:\t (%d, %d)", msg_in.xc[i], msg_in.yc[i]);
		ROS_INFO("tag distance\t:\t %f", msg_in.dist[i]);
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

    while (ros::ok())
    {
    	//Do some stuff

    	ros::spinOnce();	//refreshes subscriptions
    	loop_rate.sleep();
    }
}
