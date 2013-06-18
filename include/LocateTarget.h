#include <ros/ros.h>

class LocateTarget {
public:
	static const double searchSpaceHeight = 3;
	static const double searchSpaceWidth = 3;

	/**Time spent hovering at each spot in the search space*/
	static const double hoverTime = 0.5;

	static const double distanceBetweenSearchSpots = 0.5;

	/**distance to view wall from. Vision tracking is good 2-10m away*/
	static const double distanceToViewWallFrom = 2.5;

	/**The distance to the wall when drone is on the ground before takeoff*/
	static const double distanceToWall = 3;

	/**Assumes drone is to left of search space right edge*/
	static const double distanceToSearchSpaceRightEdge = 1;

	/**distance from ground of the lower edge of the wall search space*/
	static const double searchSpaceDistanceFromGround = 1;


	LocateTarget(std::string rosnamespace);
	virtual ~LocateTarget(void);

	void run(void);

protected:
	ros::NodeHandle node;
	ros::Publisher cmd_pub;

	void searchWall(void);
	std_msgs::String createCoordianteCommand(const char* cmd, double x, double y, double z, double yaw);
	void sendAutopilotInitialisationCommands(void);
};

