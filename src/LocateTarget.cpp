#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

#include "ardrone_autonomy/LedAnim.h"
#include "LocateTarget.h"


/* The drone will publish information about any targets it detects on the
 * /ardrone/navdata_vision_detect topic.
 * The autopilot takes goto movement commands on the /tum_ardrone/cmd topic
 * state estimation publishes on /ardrone/predictedPose which the autopilot reads?
 *
 * */


LocateTarget::LocateTarget(void)
{
	//create publishers and subscribers etc.
	/*std::string navdata_channel_name = node.resolveName("ardrone/navdata");
	std::string detect_channel_name = node.resolveName("ardrone/navdata_vision_detect");
	std::string autopilot_channel_name = node.resolveName("tum_ardrone/com");
	ROS_INFO("navdata channel is %s", navdata_channel_name.c_str());
	ROS_INFO("vision data channel is %s", detect_channel_name.c_str());
	ROS_INFO("autopilot channel is %s", autopilot_channel_name.c_str());
	*/

	state = NOT_STARTED;
	rosnamespace = node.getNamespace();
	tumcom_pub = node.advertise<std_msgs::String>(rosnamespace+"/tum_ardrone/com", 50);
	otherDrones_pub = node.advertise<std_msgs::String>("/followerdrones", 50);
	tumcom_sub = node.subscribe(rosnamespace+"/tum_ardrone/com", 50, &LocateTarget::tumcomCallback, this);
	land_sub = node.subscribe(rosnamespace+"/ardrone/land", 1, &LocateTarget::landSubscriptionCallback, this);
	leds_client = node.serviceClient<ardrone_autonomy::LedAnim>(rosnamespace+"/ardrone/setledanimation");
	sendCoords_srv = node.advertiseService(rosnamespace+"/sendCoords", &LocateTarget::serviceSendCoordsCallback, this);
	wakeUpFollowers_srv = node.advertiseService(rosnamespace+"/wakeupFollowers", &LocateTarget::serviceWakeUpFollowersCallback, this);

	setLEDSignal(8); //green
	ROS_INFO("initialised LocateTarget");
	ROS_INFO("tumcom_pub writes to %s", tumcom_pub.getTopic().c_str());
	ROS_INFO("tumcom_sub reads from %s", tumcom_sub.getTopic().c_str());
	ROS_INFO("otherDrones_pub writes to %s", otherDrones_pub.getTopic().c_str());
	ROS_INFO("resend coordinates service is on topic %s", resendCoords_srv.getService().c_str());
	ROS_INFO("wake up followers service is on topic %s", wakeUpFollowers_srv.getService().c_str());

	return;
}
LocateTarget::~LocateTarget(void)
{
	//land the drone
	setLEDSignal(8); //green
	sendTUMComClearCommand(tumcom_pub);
	sendTUMComStringCommand("c land", tumcom_pub);
}


void LocateTarget::run(void)
{
	ros::Rate loop(50);	//update at 50Hz

	sendSearchWallCommandsToAutopilot();
	ROS_INFO("sent autopilot commands");
	//wait until our tumcom_sub callback tells us that the drone is airborne
	while(state == NOT_STARTED)
	{
		ros::spinOnce();
		loop.sleep();
	}
	ROS_INFO("drone is now airborne");

	//start monitoring detection tags
	vision_sub = node.subscribe(rosnamespace+"/ardrone/navdata_vision_detect", 1, &LocateTarget::targetDetectedCallback, this);
	ROS_INFO("started subscription to target detection on topic %s", vision_sub.getTopic().c_str());

	while(state != LANDING)
	{
		ros::spinOnce();
		loop.sleep();
	}
	return;
}

void LocateTarget::setLEDSignal(int type)
{
	ardrone_autonomy::LedAnim ledMsg;
	ledMsg.request.type = type;
	ledMsg.request.freq = 4;
	ledMsg.request.duration = 0; //forever
	leds_client.call(ledMsg);
	return;
}

void LocateTarget::landSubscriptionCallback(const std_msgs::Empty::ConstPtr& msg)
{
	state = LANDING;
	return;
}

void LocateTarget::targetDetectedCallback(const ardrone_autonomy::navdata_vision_detect::ConstPtr& msg)
{
	static int targetSeenCount = 0;
	//this function will be called, even if there are no targets detected
	if(msg->nb_detected > 0)
	{
		ROS_INFO("can see the target! seen it %d times in a row", targetSeenCount);
		//canSeeTarget = true;
		//only start doing something if a target has been seen more than some number
		if(targetSeenCount > 40)
		{
			if(state == SEARCHING)
				approachTarget();
			if(state == APPROACHING)
			{
				recruitOtherDrones();
			}
			vision_sub.shutdown();
		}
		else targetSeenCount++;
	}
	else
	{
		//canSeeTarget = false;
		//ROS_INFO("SADNESS. Lost the target :(");
		targetSeenCount = 0;
	}

	return;
}

void LocateTarget::tumcomCallback(const std_msgs::String::ConstPtr& msg)
{
	/*Need to check for Command type messages on the channel. These
	 * give information on what the current and next state will be.
	 * sample controlling message:
	 * ---
		data: u c Controlling (Queue: 8)
		Current: autoInit 500 800
		Next: goto 0.5 1 0.5 0
		Target: (0.00,  0.00,  0.00), 0.0
		Error: (0.00,  0.00,  0.00), 0.0 (|.| 0.00)
		Cont.: r 0.00, p 0.00, g 0.00, y 0.00
	 *
	 * */
	static bool autopilotIsInitialising = false;

	std::string data = msg->data;
	if(!isTUMComControllingMessage(data))
		return;

	std::size_t currentCmdStartPos = data.find("Current: ");
	std::string currentCmd = data.substr(currentCmdStartPos, data.find("\nNext:")-currentCmdStartPos );
	ROS_DEBUG("current command is %s", currentCmd.c_str());

	switch(state)
	{
	case NOT_STARTED:
		if(isTUMComCurrentCommandAutoInit(currentCmd))
		{
			autopilotIsInitialising = true;
		}
		else if(autopilotIsInitialising) //if not running autoinit cmd but one has been run...
		{
			ROS_INFO("drone_autopilot has finished initialising.");
			state = SEARCHING;
		}
		break;
	case APPROACHING:
		//check if robot is now hovering and if so start recruiting
		static int approachCount = 0;
		if(isTUMComCurrentCommandNULL(currentCmd))
		{
			ROS_INFO("Moved into approach position. Starting recruitment.");
			//start recruiting
			vision_sub.shutdown();
			recruitOtherDrones();
		}/*
		if(approachCount > 20)
		{
			//clear all autopilot commands
			sendTUMComClearCommand();
			ROS_INFO("approach count is %d, cleared the commands", approachCount);
			//start recruiting
			vision_sub.shutdown();
			recruitOtherDrones();
		}*/
		//if hte robot isn't hovering we'll just wait until we're bored
		approachCount++;
		break;
	}

	return;
}


void LocateTarget::approachTarget(void)
{
	ROS_INFO("Target detected!\nAborting search and approaching target.");
	state = APPROACHING;

	//stop searching the wall
	sendTUMComClearCommand(tumcom_pub);

	//wake up others so by the time they're initialised the target is approached
	wakeUpFollowers();

	//edge forward a tiny bit
	ROS_INFO("moving forward to approach the target");
	sendTUMComCoordianteCommand("moveBy", 0, 0.2, 0, 0, tumcom_pub);
	return;
}

void LocateTarget::wakeUpFollowers(void)
{
	ROS_INFO("waking up other drones.");

	sendTUMComClearCommand(otherDrones_pub);
	sendTUMComStringCommand("c start", otherDrones_pub); //starts the autopilot (not documented but important)
	sendTUMComCoordianteCommand("setReference",0,0,0,0, otherDrones_pub);
	//sendTUMComStringCommand("c setReference $POSE$", otherDrones_pub);
	sendTUMComStringCommand("c setStayTime 0.5", otherDrones_pub);
	sendTUMComStringCommand("c autoInit 500 800 5000 0.5", otherDrones_pub);
}

void LocateTarget::sendTargetCoordinatesToFollowers(double x, double y, double z)
{
	ROS_INFO("Sending coordinates [%2.2f, %2.2f, %2.2f] to drones", x, y, z);
	//ROS_WARN("This function is not yet written. No recruitment is actually happening");
	//send the coordinates
	sendTUMComCoordianteCommand("goto", x,y,z,0, otherDrones_pub);
	//sendTUMComCoordianteCommand("goto", 0,1,1,0, otherDrones_pub);
}

void LocateTarget::recruitOtherDrones(void)
{
	ROS_INFO("Recruiting other drones to this location.");
	state = RECRUITING;
	//flash the LEDs
	setLEDSignal(2); //blink red

	//clear any queued autopilot commands
	sendTUMComClearCommand(tumcom_pub);

	//find current location and	send it to other drones
	position_sub = node.subscribe(rosnamespace+"/ardrone/predictedPose", 10, &LocateTarget::predictedPoseCallback, this);
	ROS_INFO("started subscription to predictedPose on topic %s", position_sub.getTopic().c_str());
}

bool LocateTarget::serviceSendCoordsCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& rep)
{
	ROS_INFO("send coordinates service requested");
	if(position_sub == NULL)
	{
		recruitOtherDrones();
		return true;
	}
	return false;
}

bool LocateTarget::serviceWakeUpFollowersCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& rep)
{
	ROS_INFO("wake up other drones service requested");
	wakeUpFollowers();
	return true;
}

void LocateTarget::predictedPoseCallback(const tum_ardrone::filter_state::ConstPtr& msg)
{
	static double x=0;
	static double y=0;
	static double z=0;
	static int callCount = 0;
	static int sendCount = 0;
	const int maxNumCalls = 10;

	ROS_INFO("measuring position. Currently at [%2.2f %2.2f %2.2f]",
			msg->x, msg->y, msg->z);

	if(callCount < 0)
	{
		callCount++;
	}
	else if( (callCount < maxNumCalls) )
	{
		x += msg->x;
		y += msg->y;
		z += msg->z;
		callCount++;
		//ROS_INFO("Callcount = %d", callCount);
	}
	else
	{
		x /= maxNumCalls;
		y /= maxNumCalls;
		z /= maxNumCalls;

		sendTargetCoordinatesToFollowers(x, y, z);
		sendCount++;
		//change LED colour to indicate the recruiting phase finished
		setLEDSignal(3); //blink orange
		//update new coordinates again soon
		callCount = 0;
		x=0;
		y=0;
		z=0;
		position_sub.shutdown();
	}
	return;
}

void LocateTarget::sendInitialisationCommandsToAutopilot(void)
{
	ros::Rate(0.5).sleep(); //sleep for half second
	ROS_INFO("Initialising TUM state estimation and autopilot");

	sendTUMComClearCommand(tumcom_pub);
	sendTUMComStringCommand("c start", tumcom_pub); //starts the autopilot (not documented but important)
	sendTUMComCoordianteCommand("setReference",0,0,0,0, tumcom_pub);
	//sendTUMComStringCommand("c setReference $POSE$", tumcom_pub);
	sendTUMComStringCommand("c setStayTime 0.5", tumcom_pub);
	sendTUMComStringCommand("c autoInit 500 800 5000 0.5", tumcom_pub);
}



/**Gives autopilot instructions to search a wall for a target.
 * Drone must be facing the wall you want to search!
 * Drone will start on lower right edge of wall and search up->left->down->left etc.
 *
 * Publishes commands to autopilot.
 * A list of commands is available from http://ros.org/wiki/tum_ardrone/drone_autopilot
 * */
void LocateTarget::sendSearchWallCommandsToAutopilot(void)
{
	const double searchSpaceHeight = 0.75;
	const double xDistanceToMoveWhenSearching = 0.25;

	sendInitialisationCommandsToAutopilot();
	ROS_INFO("searching the wall");

	double x=0.5;
	double y=2;
	double z=0.5;
	//x = roll, y = pitch, z=height. yaw is always 0.

	sendTUMComCoordianteCommand("goto", x, y, z, 0, tumcom_pub);
	//up
	z+=searchSpaceHeight;
	sendTUMComCoordianteCommand("goto", x, y, z, 0, tumcom_pub);
	//left
	x -= xDistanceToMoveWhenSearching;
	sendTUMComCoordianteCommand("goto", x, y, z, 0, tumcom_pub);
	//down
	z-=searchSpaceHeight;
	sendTUMComCoordianteCommand("goto", x, y, z, 0, tumcom_pub);
	//left
	x -= xDistanceToMoveWhenSearching;
	sendTUMComCoordianteCommand("goto", x, y, z, 0, tumcom_pub);
	//up
	z+=searchSpaceHeight;
	sendTUMComCoordianteCommand("goto", x, y, z, 0, tumcom_pub);

	//return home
	y = y-1;
	z-=searchSpaceHeight;
	sendTUMComCoordianteCommand("goto", x, y, z, 0, tumcom_pub);
	sendTUMComStringCommand("c land", tumcom_pub);
	ROS_INFO("autopilot data sent");

	return;
}

void LocateTarget::sendTUMComClearCommand(ros::Publisher& tumcompub)
{
	sendTUMComStringCommand("c clearCommands", tumcompub);
}

void LocateTarget::sendTUMComStringCommand(const char* cmd, ros::Publisher& tumcompub)
{
	std_msgs::String msg;
	msg.data = cmd;
	tumcompub.publish(msg);
	ROS_INFO("send TUMCom command \"%s\" on topic %s", cmd, tumcompub.getTopic().c_str());
}

void LocateTarget::sendTUMComCoordianteCommand(const char* cmd, double x, double y, double z, double yaw, ros::Publisher& tumcompub)
{
	std_msgs::String out;
	char str[64];
	sprintf(str, "c %s %2.2f %2.2f %2.2f %2.2f",
			cmd, x, y, z, yaw);
	out.data = str;
	ROS_INFO("sending command %s", out.data.c_str());
	tumcompub.publish(out);
	return;
}

bool LocateTarget::isTUMComControllingMessage(const std::string msg)
{
	//compare chars 0 to 15 of msg to given string:
	int comp = msg.compare(0, 15, "u c Controlling");
	return (comp == 0);
}

bool LocateTarget::isTUMComCurrentCommandAutoInit(const std::string cmd)
{
	std::size_t found = cmd.find("autoInit");
	//find returns std::string::npos if not found
	return (found != std::string::npos);
}

bool LocateTarget::isTUMComCurrentCommandLand(const std::string cmd)
{
	std::size_t found = cmd.find("land");
	//find returns std::string::npos if not found
	return (found != std::string::npos);
}

bool LocateTarget::isTUMComCurrentCommandNULL(const std::string cmd)
{
	std::size_t found = cmd.find("NULL");
	//find returns std::string::npos if not found
	return (found != std::string::npos);
}


