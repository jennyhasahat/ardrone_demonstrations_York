#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

#include "ardrone_autonomy/LedAnim.h"
#include "TargetAggregation.h"



/* The drone will publish information about any targets it detects on the
 * /ardrone/navdata_vision_detect topic.
 * The autopilot takes goto movement commands on the /tum_ardrone/cmd topic
 * state estimation publishes on /ardrone/predictedPose which the autopilot reads?
 *
 * */


TargetAggregationJoystickControl::TargetAggregationJoystickControl(void)
: TargetAggregation()
{
	tumcom_sub.shutdown();
	tumcom_sub = node.subscribe(rosnamespace+"/tum_ardrone/com", 50, &TargetAggregationJoystickControl::tumcomCallback, this);

	return;
}

TargetAggregationJoystickControl::~TargetAggregationJoystickControl(void)
{
}

void TargetAggregationJoystickControl::run(void)
{
	ros::Rate loop(50);	//update at 50Hz

	sendInitialisationCommandsToAutopilot();
	//send some other command so that we know when initialisation has finished
	sendTUMComCoordianteCommand("moveBy", 0,0,0,0, tumcom_pub);
	ROS_INFO("sent autopilot commands");
	//wait until our tumcom_sub callback tells us that the drone is airborne
	while(state == NOT_STARTED)
	{
		ros::spinOnce();
		loop.sleep();
	}
	ROS_INFO("drone is now airborne");
	ROS_INFO("stopping autopilot");
	//stop the autopilot
	sendTUMComStringCommand("c stop", tumcom_pub);

	//start monitoring detection tags
	vision_sub = node.subscribe(rosnamespace+"/ardrone/navdata_vision_detect", 1, &TargetAggregationJoystickControl::targetDetectedCallback, this);
	ROS_INFO("started subscription to target detection on topic %s", vision_sub.getTopic().c_str());

	while(state != LANDING)
	{
		ros::spinOnce();
		loop.sleep();
	}
	return;
}


void TargetAggregationJoystickControl::targetDetectedCallback(const ardrone_autonomy::navdata_vision_detect::ConstPtr& msg)
{
	/*ROS_INFO("target detect callback");
	static int targetSeenCount = 0;
	//this function will be called, even if there are no targets detected
	if(msg->nb_detected > 0)
	{
		ROS_INFO("can see the target! seen it %d times in a row", targetSeenCount);
		//canSeeTarget = true;
		//only start doing something if a target has been seen more than some number
		if(targetSeenCount > 30)
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
	}*/

	return;
}


void TargetAggregationJoystickControl::tumcomCallback(const std_msgs::String::ConstPtr& msg)
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
	ROS_INFO("current command is %s", currentCmd.c_str());

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
		if(approachCount > 300)
		{
			ROS_WARN("Finished approaching target. Be careful with joystick");
			//clear all autopilot commands
			sendTUMComClearCommand(tumcom_pub);
			//start recruiting
			vision_sub.shutdown();
			//recruitOtherDrones();
		}
		//if the robot isn't hovering we'll just wait until we're bored
		approachCount++;
		break;
	}

	return;
}


void TargetAggregationJoystickControl::approachTarget(void)
{
	ROS_INFO("Target detected!\nAborting search and approaching target.");
	state = APPROACHING;
	//set LEDs
	setLEDSignal(2); //blink red

	//wake up others so by the time they're initialised the target is approached
	wakeUpFollowers();

	//edge forward a tiny bit
	ROS_INFO("In approaching state but purposefully not moving forwards");
	//sendTUMComCoordianteCommand("moveBy", 0, 0.2, 0, 0);
	return;
}


