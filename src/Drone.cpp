#include <geometry_msgs/Twist.h>
#include "Drone.h"
#include "ardrone_autonomy/FlightAnim.h"
#include "ardrone_autonomy/LedAnim.h"

Drone::Drone() : rosNameSpace("")
{
	initialiseROS();
}

Drone::Drone(std::string _rosNameSpace) : rosNameSpace(_rosNameSpace)
{
	initialiseROS();
}

Drone::~Drone()
{
	navdataCallbacks.clear();
	visionDetectCallbacks.clear();
}

void Drone::initialiseROS(void)
{
	takeoff_pub = node.advertise<std_msgs::Empty>(rosNameSpace+"/ardrone/takeoff", 1); /* Message queue length is just 1 */
	land_pub = node.advertise<std_msgs::Empty>(rosNameSpace+"/ardrone/land", 1); /* Message queue length is just 1 */
	reset_pub = node.advertise<std_msgs::Empty>(rosNameSpace+"/ardrone/reset", 1);
	speeds_pub = node.advertise<geometry_msgs::Twist>(rosNameSpace+"/cmd_vel", 1);

	ROS_INFO("takeoff publishes to: %s", takeoff_pub.getTopic().c_str());
	ROS_INFO("land publishes to: %s", land_pub.getTopic().c_str());
	ROS_INFO("reset publishes to: %s", reset_pub.getTopic().c_str());
	ROS_INFO("speeds publishes to: %s", speeds_pub.getTopic().c_str());

	//sets up the LED and flight animation services
	led_client = node.serviceClient<ardrone_autonomy::LedAnim>(rosNameSpace+"/ardrone/setledanimation");
	flight_client = node.serviceClient<ardrone_autonomy::FlightAnim>(rosNameSpace+"/ardrone/setflightanimation");

	setAutoHoverEnable(true);
	isSubscribedToNavdataDemo = false;
	navdataCallbacks.clear();
	isSubscribedToNavdataVisionDetect = false;
	visionDetectCallbacks.clear();
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

void Drone::setSpeeds(double roll, double pitch, double yaw, double power)
{
	geometry_msgs::Twist msg;
	msg.linear.x = pitch;
	msg.linear.y = roll;
	msg.linear.z = power;
	//for some reason in the driver, to yaw left requires neg value
	//but to roll left requires positive value.
	//IMHO I think both yaw and roll left should need positive.
	msg.angular.z = yaw;
	//these enable auto-hover at 0, and disable otherwise
	msg.angular.x = isAutoHoverDisabled;
	msg.angular.y = isAutoHoverDisabled;
	speeds_pub.publish(msg);
	return;
}

void Drone::addNavdataDemoCallback(navdata_demo_function callback)
{
	if(!isSubscribedToNavdataDemo)
	{
		navdata_demo_sub = node.subscribe(rosNameSpace+"/ardrone/navdata_demo", 1, &Drone::navdataDemoCallback, this );
		ROS_INFO("Opened subscription to %s", navdata_demo_sub.getTopic().c_str());
	}
	navdataCallbacks.push_back(callback);
	return;
}

void Drone::addNavdataVisionDetectCallback(navdata_vision_detect_function callback)
{
	if(!isSubscribedToNavdataVisionDetect)
	{
		vision_detect_sub = node.subscribe(rosNameSpace+"/ardrone/navdata_vision_detect", 1, &Drone::navdataVisionDetectCallback, this);
		ROS_INFO("Opened subscription to %s", vision_detect_sub.getTopic().c_str());
	}
	visionDetectCallbacks.push_back(callback);
	return;
}


bool Drone::doLEDAnimation(int type, float freq, int duration)
{
	ardrone_autonomy::LedAnim msg;
	msg.request.type = type;
	msg.request.freq = freq;
	msg.request.duration = duration;
	bool successfulCall = led_client.call(msg);
	return (successfulCall && msg.response.result);
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

void Drone::setAutoHoverEnable(bool newstate)
{
	//needs to be 0 to enable auto hover, so if(newstate=true) 0.
	isAutoHoverDisabled = (newstate? 0:1);
	ROS_INFO("AutoHoverDisabled set to %f", isAutoHoverDisabled);
	return;
}

void Drone::navdataDemoCallback(const ardrone_autonomy::navdata_demo::ConstPtr& msg)
{
	ROS_DEBUG("got to Drone::navdataDemoCallback");
	for(unsigned int i=0; i<navdataCallbacks.size(); i++)
		navdataCallbacks[i](msg);
}

void Drone::navdataVisionDetectCallback(const ardrone_autonomy::navdata_vision_detect::ConstPtr& msg)
{
	ROS_DEBUG("got to Drone::navdataVisionDetectCallback");
	for(unsigned int i=0; i<visionDetectCallbacks.size(); i++)
		visionDetectCallbacks[i](msg);
}
