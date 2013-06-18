#include <ros/ros.h>
#include "Drone.h"

	/*	FLIGHT_PHI_M30_DEG=0,		// 0
		FLIGHT_PHI_30_DEG,
		FLIGHT_THETA_M30_DEG,
		FLIGHT_THETA_30_DEG,
		FLIGHT_THETA_20DEG_YAW_200DEG,
		FLIGHT_THETA_20DEG_YAW_M200DEG, // 5
		FLIGHT_TURNAROUND,
		FLIGHT_TURNAROUND_GODOWN,
		FLIGHT_YAW_SHAKE,
		FLIGHT_YAW_DANCE,
		FLIGHT_PHI_DANCE,	//10
		FLIGHT_THETA_DANCE,
		FLIGHT_VZ_DANCE,
		FLIGHT_WAVE,
		FLIGHT_PHI_THETA_MIXED,
		FLIGHT_DOUBLE_PHI_THETA_MIXED, // 15
		FLIGHT_FLIP_AHEAD,
		FLIGHT_FLIP_BEHIND,
		FLIGHT_FLIP_LEFT,
		FLIGHT_FLIP_RIGHT
	*/


int main(int argc, char** argv)
{
	ROS_INFO("Blue Danube Dance");
	ros::init(argc, argv, "BlueDanubeL");
	Drone d("/dronesL");

	double initial_interval = 3.4;
	double dedadadedaaa = 3.5;
	double nana = 1.3;//1.45;

	double startTime = ros::Time::now().toSec();
	ROS_INFO("start time is %f", startTime);

		ROS_INFO("Sending take off");
		d.takeOff(5);
		d.setSpeeds(0,0,0,0);
		ROS_INFO("sending led request");
		d.doLEDAnimation(Drone::LED_SNAKE_GREEN_RED, 0.5, 35);

		ROS_INFO("do do be do dooo 1");
		d.waitSeconds(initial_interval);
		ROS_INFO("na na 1");
		d.doFlightAnimation(Drone::FLIGHT_THETA_30_DEG, 0.3);
		d.waitSeconds(nana);
		d.doFlightAnimation(Drone::FLIGHT_THETA_M30_DEG, 0.3);

		ROS_INFO("do do be do dooo 2");
		d.waitSeconds(dedadadedaaa+0.3);
		ROS_INFO("na na 2");
		d.doFlightAnimation(Drone::FLIGHT_PHI_M30_DEG, 0.3);
		d.waitSeconds(nana);
		d.doFlightAnimation(Drone::FLIGHT_PHI_30_DEG, 0.3);

		ROS_INFO("do do be do dooo 3");
		d.waitSeconds(dedadadedaaa);
		ROS_INFO("na na 3");
		d.doFlightAnimation(Drone::FLIGHT_THETA_M30_DEG, 0.5);
		d.waitSeconds(nana);
		d.doFlightAnimation(Drone::FLIGHT_THETA_30_DEG, 0.5);

		ROS_INFO("do do be do dooo 4");
		d.waitSeconds(dedadadedaaa);
		ROS_INFO("na na 4");
		d.doFlightAnimation(Drone::FLIGHT_PHI_30_DEG, 0.5);
		d.waitSeconds(nana-0.2);
		d.doFlightAnimation(Drone::FLIGHT_PHI_M30_DEG, 0.5);

		ROS_INFO("do do be do dooo 5 (it's getting real!)");
		d.waitSeconds(dedadadedaaa-0.5);
		ROS_INFO("na na 5");
		ROS_INFO("yaw right?");
		d.setSpeeds(0,0,1,0); //yaw left
		d.waitSeconds(nana-0.2);
		d.setSpeeds(0,0,0,0); //stop
		d.setSpeeds(0,0,-1,0); //yaw right
		d.waitSeconds(nana-0.2);
		d.setSpeeds(0,0,0,0); //stop

		ROS_INFO("do do be do dooo 6");
		d.waitSeconds(dedadadedaaa-0.5-nana+0.2);
		ROS_INFO("na na 6");
		ROS_INFO("up?");
		d.setSpeeds(0,0,0,1);
		d.waitSeconds(nana-0.25);
		ROS_INFO("down?");
		d.setSpeeds(0,0,0,-0.5);

		d.waitSeconds(nana-0.25);
		d.setSpeeds(0,0,0,0);

		//grand finalé
		ROS_INFO("grand finale!");
		d.waitSeconds(0.35);
		//d.setSpeeds(0,0,-1,0);
		d.doFlightAnimation(Drone::FLIGHT_YAW_SHAKE, 0);
		d.waitSeconds(2.5);
		d.doFlightAnimation(Drone::FLIGHT_PHI_THETA_MIXED, 0);
		d.waitSeconds(5);


		ROS_INFO("landing");
		d.land(5);


		ros::spinOnce();	//refreshes subscriptions
	return 0;
}
