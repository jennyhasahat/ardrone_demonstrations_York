#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <string>
#include "ardrone_autonomy/navdata_vision_detect.h"
#include "tum_ardrone/filter_state.h"

class TargetAggregation {
public:
	enum droneState{NOT_STARTED, SEARCHING, APPROACHING, RECRUITING, LANDING};

	TargetAggregation(void);
	virtual ~TargetAggregation(void);

	void run(void);
	droneState state;


protected:
	ros::NodeHandle node;
	ros::Publisher tumcom_pub;
	ros::Publisher otherDrones_pub;
	ros::Subscriber vision_sub;
	ros::Subscriber tumcom_sub;
	ros::Subscriber position_sub;
	ros::ServiceClient leds_client;
	ros::Subscriber land_sub;
	ros::ServiceServer sendCoords_srv;
	ros::ServiceServer wakeUpFollowers_srv;

	std::string rosnamespace;

	//bool canSeeTarget;

	/**Publishes a series of movements to the autopilot to move up and down a wall.
	 * */
	void sendSearchWallCommandsToAutopilot(void);

	void sendInitialisationCommandsToAutopilot(void);

	virtual void approachTarget(void);

	void wakeUpFollowers(void);
	void recruitOtherDrones(void);
	void sendTargetCoordinatesToFollowers(double x, double y, double z);

	/**Type is the animation type, as listed in ardrone_autonomy/LedAnim.h*/
	void setLEDSignal(int type);

	//ROS subscriber callbacks
	void landSubscriptionCallback(const std_msgs::Empty::ConstPtr& msg);
	void predictedPoseCallback(const tum_ardrone::filter_state::ConstPtr& msg);
	void targetDetectedCallback(const ardrone_autonomy::navdata_vision_detect::ConstPtr& msg);

	//ROS service callbacks
	bool serviceSendCoordsCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& rep);
	bool serviceWakeUpFollowersCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& rep);

	//various TUM com channel callbacks and checks
	void tumcomCallback(const std_msgs::String::ConstPtr& msg);

	void sendTUMComCoordianteCommand(const char* cmd, double x, double y, double z, double yaw, ros::Publisher& tumcompub);
	void sendTUMComClearCommand(ros::Publisher& tumcompub);
	void sendTUMComStringCommand(const char* cmd, ros::Publisher& tumcompub);
	bool isTUMComControllingMessage(const std::string msg);
	bool isTUMComCurrentCommandAutoInit(const std::string cmd);
	bool isTUMComCurrentCommandLand(const std::string cmd);
	bool isTUMComCurrentCommandNULL(const std::string cmd);
};


class TargetAggregationJoystickControl : public TargetAggregation
{
public:
	TargetAggregationJoystickControl(void);
	virtual ~TargetAggregationJoystickControl(void);
	void run(void);
	void approachTarget(void);

	void tumcomCallback(const std_msgs::String::ConstPtr& msg);
	void targetDetectedCallback(const ardrone_autonomy::navdata_vision_detect::ConstPtr& msg);
};

