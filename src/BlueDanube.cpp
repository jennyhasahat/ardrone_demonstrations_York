#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "ardrone_autonomy/FlightAnim.h"
#include "ardrone_autonomy/LedAnim.h"

/* List of the flight animations and their enum number.
# 0 : ARDRONE_ANIM_PHI_M30_DEG
# 1 : ARDRONE_ANIM_PHI_30_DEG
# 2 : ARDRONE_ANIM_THETA_M30_DEG
# 3 : ARDRONE_ANIM_THETA_30_DEG
# 4 : ARDRONE_ANIM_THETA_20DEG_YAW_200DEG
# 5 : ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG
# 6 : ARDRONE_ANIM_TURNAROUND
# 7 : ARDRONE_ANIM_TURNAROUND_GODOWN
# 8 : ARDRONE_ANIM_YAW_SHAKE
# 9 : ARDRONE_ANIM_YAW_DANCE
# 10: ARDRONE_ANIM_PHI_DANCE
# 11: ARDRONE_ANIM_THETA_DANCE
# 12: ARDRONE_ANIM_VZ_DANCE
# 13: ARDRONE_ANIM_WAVE
# 14: ARDRONE_ANIM_PHI_THETA_MIXED
# 15: ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED
# 16: ARDRONE_ANIM_FLIP_AHEAD
# 17: ARDRONE_ANIM_FLIP_BEHIND
# 18: ARDRONE_ANIM_FLIP_LEFT
# 19: ARDRONE_ANIM_FLIP_RIGHT

See http://www.ros.org/wiki/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
for an example of how services and clients work in ROS.
*/

void waitSomeSeconds(double wait)
{
	double endtime = ros::Time::now().toSec() + wait;
	while (ros::Time::now().toSec()< endtime)
	{

	}
	return;
}

int main(int argc, char** argv)
{
	ROS_INFO("Blue Danube Dance");
	ros::init(argc, argv, "BlueDanubeDance");

	ros::NodeHandle node;

	ros::Publisher takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
	ros::Publisher land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
	ros::Publisher reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1);

	//sets up the LED and flight animation services
	ros::ServiceClient ledClient = node.serviceClient<ardrone_autonomy::LedAnim>("/ardrone/setledanimation");
	ros::ServiceClient flightClient = node.serviceClient<ardrone_autonomy::FlightAnim>("/ardrone/setflightanimation");

	ardrone_autonomy::LedAnim ledMsg;
	ardrone_autonomy::FlightAnim flightMsg;
	std_msgs::Empty emptyMsg;
	ledMsg.request.type = 7; //RED
	ledMsg.request.freq = 1.0; //1Hz
	ledMsg.request.duration = 1; //1 second

	double startTime = ros::Time::now().toSec();
	ROS_INFO("start time is %f", startTime);

	//while (ros::ok())
	//{
		ROS_INFO("sending led request");

		if( ledClient.call(ledMsg) ) //if service call successful
		{
			ROS_INFO("successfully set leds?. Response was %d", ledMsg.response.result? 1:0);
		}
		else
		{
			ROS_INFO("unsuccessful set leds request");
		}
		ROS_INFO("waiting 4.5 seconds");
		waitSomeSeconds(4.5);
		ROS_INFO("finished waiting");

		ROS_INFO("sending led request");
		ledMsg.request.type = 3; //BLINK ORANGE
		if( ledClient.call(ledMsg) ) //if service call successful
		{
			ROS_INFO("successfully set leds?. Response was %d", ledMsg.response.result? 1:0);
		}
		else
		{
			ROS_INFO("unsuccessful set leds request");
		}

		ros::spinOnce();	//refreshes subscriptions
	//}
	return 0;
}
