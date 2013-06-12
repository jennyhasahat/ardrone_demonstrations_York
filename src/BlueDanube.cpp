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

	double initial_interval = 3.9;
	double dedadadedaaa = 3.5;
	double nana = 1.45;

	double startTime = ros::Time::now().toSec();
	ROS_INFO("start time is %f", startTime);

		ROS_INFO("Sending take off");
		d.takeOff(5);
		ROS_INFO("sending led request");
		//d.doLEDAnimation(0, 3.0, 25);

		ROS_INFO("do do be do dooo 1");
		d.waitSeconds(initial_interval);
		ROS_INFO("na na 1");
		d.doFlightAnimation(3, 0.5);
		d.waitSeconds(nana);
		d.doFlightAnimation(2, 0.5);

		ROS_INFO("do do be do dooo 2");
		d.waitSeconds(dedadadedaaa+0.3);
		ROS_INFO("na na 2");
		d.doFlightAnimation(1, 0.5);
		d.waitSeconds(nana);
		d.doFlightAnimation(0, 0.5);

		ROS_INFO("do do be do dooo 3");
		d.waitSeconds(dedadadedaaa);
		ROS_INFO("na na 3");
		d.doFlightAnimation(3, 0.5);
		d.waitSeconds(nana);
		d.doFlightAnimation(2, 0.5);

		ROS_INFO("do do be do dooo 4");
		d.waitSeconds(dedadadedaaa);
		ROS_INFO("na na 4");
		d.doFlightAnimation(1, 0.5);
		d.waitSeconds(nana-0.2);
		d.doFlightAnimation(0, 0.5);

		ROS_INFO("do do be do dooo 5");
		d.waitSeconds(dedadadedaaa-0.5);
		ROS_INFO("na na 5");
		d.doFlightAnimation(11, 1);
		d.waitSeconds(nana-0.2);
		d.doFlightAnimation(8, 1);

		ROS_INFO("do do be do dooo 6");
		d.waitSeconds(dedadadedaaa-0.5);
		ROS_INFO("na na 6");
		d.doFlightAnimation(12, 1);
		d.waitSeconds(nana-0.25);
		d.doFlightAnimation(14, 1);

		//grand finalé
		ROS_INFO("grand finalé!");
		d.waitSeconds(0.35);
		//yaw left
		d.waitSeconds(2.5);
		d.doFlightAnimation(7, 4.5);
		d.waitSeconds(5);		

		ROS_INFO("landing");
		d.land(5);


		ros::spinOnce();	//refreshes subscriptions
	return 0;
}
