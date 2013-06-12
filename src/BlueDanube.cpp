#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "ardrone_autonomy/FlightAnim.h"
#include "ardrone_autonomy/LedAnim.h"

#include "Drone.h"

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
	Drone d;

	double startTime = ros::Time::now().toSec();
	ROS_INFO("start time is %f", startTime);

	//while (ros::ok())
	//{
		ROS_INFO("sending led request");

		d.doLEDAnimation(7, 1, 2);
		ROS_INFO("Sending take off");
		d.takeOff(5);
		ROS_INFO("waiting");
		d.waitSeconds(5);
		ROS_INFO("landing");
		d.land(5);


		ros::spinOnce();	//refreshes subscriptions
	//}
	return 0;
}
