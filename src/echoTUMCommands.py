#!/usr/bin/env python

import rospy
from std_msgs.msg import String		 # for TUM commands


	
def echoCommand(TUMCmd):
	rospy.logdebug("echoCommand publishing command "+TUMCmd.data)
	pubTUM.publish(TUMCmd)

# Setup the application
if __name__=='__main__':
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('echoTUMCommands')
	subAllDrones	= rospy.Subscriber('/followerdrones', String, echoCommand)
	pubTUM		= rospy.Publisher(rospy.get_namespace()+'tum_ardrone/com', String)
	rospy.loginfo("publishing TUM commands to "+pubTUM.name )
	
	# update the subscriber forever
	rospy.spin()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('closing TUM command echoer')
	
