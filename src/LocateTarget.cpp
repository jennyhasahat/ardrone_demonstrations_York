#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

#include "LocateTarget.h"


/* The drone will publish information about any targets it detects on the
 * /ardrone/navdata_vision_detect topic.
 * The autopilot takes goto movement commands on the /tum_ardrone/cmd topic
 * state estimation publishes on /ardrone/predictedPose which the autopilot reads?
 *
 * */


LocateTarget::LocateTarget(std::string rosnamespace)
{
	ros::Rate loop_rate(50);	//update at 50Hz

	//create publishers and subscribers etc.
	/*std::string navdata_channel_name = node.resolveName("ardrone/navdata");
	std::string detect_channel_name = node.resolveName("ardrone/navdata_vision_detect");
	std::string autopilot_channel_name = node.resolveName("tum_ardrone/com");
	ROS_INFO("navdata channel is %s", navdata_channel_name.c_str());
	ROS_INFO("vision data channel is %s", detect_channel_name.c_str());
	ROS_INFO("autopilot channel is %s", autopilot_channel_name.c_str());
	*/

	cmd_pub = node.advertise<std_msgs::String>(node.resolveName("tum_ardrone/com"),50);
	ROS_INFO("initialised LocateTarget");
	return;
}
LocateTarget::~LocateTarget(void)
{

}

void LocateTarget::run(void)
{
	searchWall();
	ros::spin();
}


/**Gives autopilot instructions to search a wall for a target.
 * Drone must be facing the wall you want to search!
 * Drone will start on lower right edge of wall and search up->left->down->left etc.
 *
 * Publishes commands to autopilot.
 * A list of commands is available from http://ros.org/wiki/tum_ardrone/drone_autopilot
 * */
void LocateTarget::searchWall(void)
{
	std_msgs::String msg;

	msg.data = "c start";
	cmd_pub.publish(msg);

	//initialise autopilot
	sendAutopilotInitialisationCommands();


	//get in start position
	cmd_pub.publish(
			createCoordianteCommand("goto",
				(distanceToWall-distanceToViewWallFrom),
				(-1* distanceToSearchSpaceRightEdge),
				searchSpaceDistanceFromGround,
				0)
			);

	//return home
	cmd_pub.publish(createCoordianteCommand("goto",0,0,1,0));
	//land
	msg.data = "c land";
	cmd_pub.publish(msg);




	return;
}


std_msgs::String LocateTarget::createCoordianteCommand(const char* cmd, double x, double y, double z, double yaw)
{
	std_msgs::String out;
	char str[64];
	sprintf(str, "c %s %2.2f %2.2f %2.2f %2.2f\n",
			cmd, x, y, z, yaw);
	out.data = str;
	ROS_INFO("created command %s", out.data.c_str());
	return out;
}

void LocateTarget::sendAutopilotInitialisationCommands(void)
{
	std_msgs::String msg;
	char str[32];

	//autoInit [int moveTimeMS] [int waitTimeMS] [int riseTimeMs] [float initSpeed]
	msg.data = "c takeoff";
	ROS_INFO("created command %s", msg.data.c_str());
	cmd_pub.publish(msg);
	msg.data = "c autoTakeover 500 800 5000 0.5";
	ROS_INFO("created command %s", msg.data.c_str());
	cmd_pub.publish(msg);


	//msg.data = "c autoInit 500 800 5000 0.5";
	//ROS_INFO("created command %s", msg.data.c_str());
	//cmd_pub.publish(msg);

	//set initial coordinates to 0
	//setReference [doube x] [double y] [double z] [double yaw]
	cmd_pub.publish(createCoordianteCommand("setReference",0,0,0,0));

	//set the time spend hovering at search spot
	//setStayTime [double seconds = 2.0]
	sprintf(str, "c setStayTime %2.2f", hoverTime);
	msg.data = str;
	ROS_INFO("created command %s", msg.data.c_str());
	cmd_pub.publish(msg);


	return;
}



int main(int argc, char** argv)
{
	ROS_INFO("Find Target");
	ros::init(argc, argv, "FindTarget");

	LocateTarget lt("");
	lt.run();

	return 0;
}
