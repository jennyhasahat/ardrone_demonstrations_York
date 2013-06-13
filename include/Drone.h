#ifndef DRONE_H_
#define DRONE_H_

#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "ardrone_autonomy/navdata_demo.h"
#include "ardrone_autonomy/navdata_vision_detect.h"

class Drone {
public:
	enum led_animations{LED_BLINK_GREEN_RED =0,
				LED_BLINK_GREEN,
				LED_BLINK_RED,
				LED_BLINK_ORANGE,
				LED_SNAKE_GREEN_RED,
				LED_FIRE,		//5
				LED_STANDARD,
				LED_RED,
				LED_GREEN,
				LED_RED_SNAKE,
				LED_BLANK,		//10
				LED_LEFT_GREEN_RIGHT_RED,
				LED_LEFT_RED_RIGHT_GREEN,
				LED_BLINK_STANDARD};
	enum flight_animations{FLIGHT_PHI_M30_DEG=0,
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
				FLIGHT_FLIP_RIGHT};

	typedef void (*navdata_demo_function)(const ardrone_autonomy::navdata_demo::ConstPtr&);
	typedef void (*navdata_vision_detect_function)(const ardrone_autonomy::navdata_vision_detect::ConstPtr&);

	Drone();

	/**Initialises the drone inside a given ROS namespace.
	 * When you add a ros node to a group, all its data is in a different namespace.
	 * eg /ardrone/takeoff becomes /namespace/ardrone/takeoff
	 * This constructor prepends all the data IO things with a given namespace.
	 *
	 * This is useful if there are multiple drones that ROS must control at once.
	 * If the drones are NOT in different namespaces, commands will be sent to
	 * all drones at once and sensor data will arrive and you won't be able
	 * to tell which drone sent it.
	 * */
	Drone(std::string _rosNameSpace);
	virtual ~Drone();

	// CONTROLLING DRONE

	/**Takes off and blocks code for specified number of seconds whilst taking off.
	* */
	void takeOff(double takeoffTime);
	/**Takes off and blocks code for specified number of seconds whilst landing.
	 * */
	void land(double landTime);


	/**Sets the various roll, pitch, yaw and height speeds of the drone.
	 * All parameters should be a value between -1 and 1.
	 * @param roll 0 to 1 will strafe LEFT, 0 to -1 will strafe RIGHT.
	 * @param pitch 0 to 1 will move FORWARDS, 0 to -1 BACKWARDS.
	 * @param yaw 0 to 1 will turn LEFT, 0 to -1 will turn RIGHT.
	 * @param power 0 to 1 will move UPWARDS, 0 to -1 DOWNWARDS.
	 * @see https://github.com/AutonomyLab/ardrone_autonomy#sending-commands-to-ar-drone
	 * */
	void setSpeeds(double roll, double pitch, double yaw, double power);

	// GETTING SENSOR DATA

	/**Navdata demo gives you all kinds of movement data from the drone.
	 * For this to work, you need to set the enable_navdata_demo config in your
	 * launch file to true.
	 * eg:<br>
	 * <code><param name="enable_navdata_demo" value="true" /></code>
	 * @param callback a pointer to a function you want navdata information to be passed to.
	 * */
	void addNavdataDemoCallback(navdata_demo_function callback);

	/**Navdata vision detect, gives information about target detection.
	 * You need to set enable_navdata_vision_detect config in your
	 * launch file to true.
	 * eg:<br>
	 * <code><param name="enable_navdata_vision_detect" value="true" /></code>
	 * @param callback a pointer to a function you want navdata information to be passed to.
	 * */
	void addNavdataVisionDetectCallback(navdata_vision_detect_function callback);

	// SERVICES

	/**Runs LED animation at freq Hz for duration seconds.
	 * Type references the LED enum from led_animations.
	 * */
	bool doLEDAnimation(int type, float freq, int duration);

	/**Run a flight animation for the given duration (in seconds).
	 * type references the flight_animations enum.
	 * flight animations don't block control, so if you run a flight animation
	 * control will go straight back to your program WHILST the animation is running
	 * */
	bool doFlightAnimation(int type, int duration);

	/**Does flight animation for default animation time.
	 * @see #doFlightAnimation
	 * */
	bool doFlightAnimation(int type);

	// MISC FUNCTIONS
	void waitSeconds(double wait);
	void setAutoHoverEnable(bool newstate);

protected:
	ros::NodeHandle node;

	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	ros::Publisher reset_pub;
	ros::Publisher speeds_pub;

	ros::Subscriber navdata_demo_sub;
	ros::Subscriber vision_detect_sub;

	ros::ServiceClient led_client;
	ros::ServiceClient flight_client;

	double isAutoHoverDisabled;
	bool isSubscribedToNavdataDemo;
	bool isSubscribedToNavdataVisionDetect;
	std::string rosNameSpace;

	std::vector<navdata_demo_function> navdataCallbacks;
	std::vector<navdata_vision_detect_function> visionDetectCallbacks;

	void initialiseROS(void);
	void navdataDemoCallback(const ardrone_autonomy::navdata_demo::ConstPtr& msg);
	void navdataVisionDetectCallback(const ardrone_autonomy::navdata_vision_detect::ConstPtr& msg);

private:
	std_msgs::Empty emptyMsg;
};

#endif /* DRONE_H_ */
