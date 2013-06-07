#include <ros/ros.h>
#include "ardrone_autonomy/LedAnim.h"

/* List of the LED animations and their enum number.
 * See ardrone_autonomy/srv/LedAnim.srv
# 0 : BLINK_GREEN_RED
# 1 : BLINK_GREEN
# 2 : BLINK_RED
# 3 : BLINK_ORANGE
# 4 : SNAKE_GREEN_RED
# 5 : FIRE
# 6 : STANDARD
# 7 : RED
# 8 : GREEN
# 9 : RED_SNAKE
# 10: BLANK
# 11: LEFT_GREEN_RIGHT_RED
# 12: LEFT_RED_RIGHT_GREEN
# 13: BLINK_STANDARD

See http://www.ros.org/wiki/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
for an example of how services and clients work in ROS.
*/

int main(int argc, char** argv)
{
	ROS_INFO("Flash LEDs");
	ros::init(argc, argv, "Flash LEDs");

	ros::NodeHandle node;
    ros::Rate loop_rate(1);	//update at 1Hz

    //sets up the LED service so I can set the LED animations
    //ardrone_autonomy::LedAnim is the service description set in the ardrone_autonomy/srv folder
    //"/ardrone/setledanimation" is the roscore name of the led animation service
    ros::ServiceClient ledClient = node.serviceClient<ardrone_autonomy::LedAnim>("/ardrone/setledanimation");

    ardrone_autonomy::LedAnim ledMsg;
    ledMsg.request.type = 7; //RED
    ledMsg.request.freq = 1.0; //1Hz
    ledMsg.request.duration = 1; //1 second

    //while (ros::ok())
    //{
    	ROS_INFO("sending led request");

    	if( ledClient.call(ledMsg) ) //if service call successful
    	{
    		ROS_INFO("successfully set leds?. Response was %d", ledMsg.response.result? 1:0);
    	}
    	else
    	{
    		ROS_INFO("unsuccessful set leds request");
    	}

    //	ros::spinOnce();	//refreshes subscriptions
    //	loop_rate.sleep();
    //}
    return 0;
}
