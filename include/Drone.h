#ifndef DRONE_H_
#define DRONE_H_

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "ardrone_autonomy/LedAnim.h"

class Drone {
public:
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
	Drone(std::string rosNameSpace);
	virtual ~Drone();

	/**Takes off and blocks code for specified number of seconds whilst taking off.
	* */
	void takeOff(double takeoffTime);
	/**Takes off and blocks code for specified number of seconds whilst landing.
	 * */
	void land(double landTime);

	// SERVICES

	/**Runs LED animation at freq Hz for duration seconds.
	 * Type references the LED enum from led animation:
		-# 0 : BLINK_GREEN_RED
		-# 1 : BLINK_GREEN
		-# 2 : BLINK_RED
		-# 3 : BLINK_ORANGE
		-# 4 : SNAKE_GREEN_RED
		-# 5 : FIRE
		-# 6 : STANDARD
		-# 7 : RED
		-# 8 : GREEN
		-# 9 : RED_SNAKE
		-# 10: BLANK
		-# 11: LEFT_GREEN_RIGHT_RED
		-# 12: LEFT_RED_RIGHT_GREEN
		-# 13: BLINK_STANDARD
	 * */
	bool doLEDAnimation(int type, float freq, int duration);

	/**Run a flight animation for the given duration (in seconds).
	 * type references the flight animation enum from config.h in the sdk:
		-# 0 : ARDRONE_ANIM_PHI_M30_DEG
		-# 1 : ARDRONE_ANIM_PHI_30_DEG
		-# 2 : ARDRONE_ANIM_THETA_M30_DEG
		-# 3 : ARDRONE_ANIM_THETA_30_DEG
		-# 4 : ARDRONE_ANIM_THETA_20DEG_YAW_200DEG
		-# 5 : ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG
		-# 6 : ARDRONE_ANIM_TURNAROUND
		-# 7 : ARDRONE_ANIM_TURNAROUND_GODOWN
		-# 8 : ARDRONE_ANIM_YAW_SHAKE
		-# 9 : ARDRONE_ANIM_YAW_DANCE
		-# 10: ARDRONE_ANIM_PHI_DANCE
		-# 11: ARDRONE_ANIM_THETA_DANCE
		-# 12: ARDRONE_ANIM_VZ_DANCE
		-# 13: ARDRONE_ANIM_WAVE
		-# 14: ARDRONE_ANIM_PHI_THETA_MIXED
		-# 15: ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED
		-# 16: ARDRONE_ANIM_FLIP_AHEAD
		-# 17: ARDRONE_ANIM_FLIP_BEHIND
		-# 18: ARDRONE_ANIM_FLIP_LEFT
		-# 19: ARDRONE_ANIM_FLIP_RIGHT
	 * */
	bool doFlightAnimation(int type, int duration);

	/**Does flight animation for default animation time.
	 * @see #doFlightAnimation
	 * */
	bool doFlightAnimation(int type);

	// MISC FUNCTIONS
	void waitSeconds(double wait);

protected:
	ros::NodeHandle node;

	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	ros::Publisher reset_pub;

	ros::ServiceClient led_client;
	ros::ServiceClient flight_client;

	void initialiseROS(std::string rosNameSpace);

private:
	std_msgs::Empty emptyMsg;
};

#endif /* DRONE_H_ */
