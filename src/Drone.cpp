#include "Drone.h"
#include <std_msgs/Empty.h>
#include "ardrone_autonomy/FlightAnim.h"
#include "ardrone_autonomy/LedAnim.h"

Drone::Drone()
{
	initialiseROS("");
}

Drone::Drone(std::string rosNameSpace)
{
	initialiseROS(rosNameSpace);
}

Drone::~Drone() {
	// TODO Auto-generated destructor stub
}

void Drone::initialiseROS(std::string rosNameSpace)
{
	takeoff_pub = node.advertise<std_msgs::Empty>(rosNameSpace+"/ardrone/takeoff", 1); /* Message queue length is just 1 */
	land_pub = node.advertise<std_msgs::Empty>(rosNameSpace+"/ardrone/land", 1); /* Message queue length is just 1 */
	reset_pub = node.advertise<std_msgs::Empty>(rosNameSpace+"/ardrone/reset", 1);

	ROS_INFO("takeoff publishes to: %s", takeoff_pub.getTopic().c_str());
	ROS_INFO("land publishes to: %s", land_pub.getTopic().c_str());
	ROS_INFO("reset publishes to: %s", reset_pub.getTopic().c_str());

	//sets up the LED and flight animation services
	led_client = node.serviceClient<ardrone_autonomy::LedAnim>(rosNameSpace+"/ardrone/setledanimation");
	flight_client = node.serviceClient<ardrone_autonomy::FlightAnim>(rosNameSpace+"/ardrone/setflightanimation");
}

void Drone::takeOff(double takeoffTime)
{
	double endtime = ros::Time::now().toSec() + takeoffTime;
	ros::Rate repeatfreq(50);
	while (ros::Time::now().toSec()< endtime)
	{
		takeoff_pub.publish(emptyMsg);
		repeatfreq.sleep();
	}
	return;
}

void Drone::land(double landTime)
{
	double endtime = ros::Time::now().toSec() + landTime;
	ros::Rate repeatfreq(50);
	while (ros::Time::now().toSec()< endtime)
	{
		land_pub.publish(emptyMsg);
		repeatfreq.sleep();
	}
	return;
}


bool Drone::doLEDAnimation(int type, float freq, int duration)
{
	ardrone_autonomy::LedAnim ledMsg;
	ledMsg.request.type = type;
	ledMsg.request.freq = freq;
	ledMsg.request.duration = duration;
	bool successfulCall = led_client.call(ledMsg);
	return (successfulCall && ledMsg.response.result);
}

bool Drone::doFlightAnimation(int type, int duration)
{
	ardrone_autonomy::FlightAnim fmsg;
	fmsg.request.type = type;
	fmsg.request.duration = duration;
	bool successfulCall = flight_client.call(fmsg);
	return (successfulCall && fmsg.response.result);
}

bool Drone::doFlightAnimation(int type)
{
	return doFlightAnimation(type, 0);
}


void Drone::waitSeconds(double wait)
{
	double endtime = ros::Time::now().toSec() + wait;
	while (ros::Time::now().toSec()< endtime)
	{

	}
	return;
}

