#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback
#
# This program has been modified for the TargetAggregation demo using the TUM ardrone drivers.

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import rospy

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty      	 # for land/takeoff/emergency
from std_msgs.msg import String		 # for TUM commands
from std_srvs.srv import Empty as srvEmpty      	 # for resend coordinates
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from sensor_msgs.msg import Joy		 # Import the joystick message


# Some Constants
COMMAND_PERIOD = 100 #ms

class DroneStatus(object):
	Emergency = 0
	Inited    = 1
	Landed    = 2
	Flying    = 3
	Hovering  = 4
	Test      = 5
	TakingOff = 6
	GotoHover = 7
	Landing   = 8
	Looping   = 9


class TUMDroneController(object):
	def __init__(self):
		# Holds the current drone status
		self.status = -1

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    	= rospy.Publisher('/ardrone/land',Empty)
		self.pubTakeoff 	= rospy.Publisher('/ardrone/takeoff',Empty)
		self.pubReset		= rospy.Publisher('/ardrone/reset',Empty)
		self.pubFollowersTUM	= rospy.Publisher('/followerdrones',String)	
		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist)

		# Setup regular publishing of control packets
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)
		
		# added on stuff. Essentially for basic mutex so TUM commands aren't sent repeatedly
		self.isRequestingCoordinates = False
		self.wakingUpOthers = False

		# Land the drone if we are shutting down
		rospy.on_shutdown(self.SendLand)

	def ReceiveNavdata(self,navdata):
		# Although there is a lot of data in this packet, we're only interested in the state at the moment	
		self.status = navdata.state

	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.status == DroneStatus.Landed):
			self.pubTakeoff.publish(Empty())

	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubLand.publish(Empty())

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())
		self.pubTUM.publish(String("c land"))
	
	def SendFollowersLand(self):
		# Sends a message to follower drone autopilots to return home and land
		self.pubTUM.publish(String("c clearCommands"))
		self.pubTUM.publish(String("c setInitialReachDist 0.5"))
		self.pubTUM.publish(String("c setStayWithinDist 0.5"))
		self.pubTUM.publish(String("c goto 0 0 0 0"))
		self.pubTUM.publish(String("c land"))
	
	def SendResetPTAM(self):
		# resets the camera feature tracking (or should do)
		self.pubTUM.publish(String("p reset"))
	
	def SendCoordinates(self):
		if not self.isRequestingCoordinates:
			self.isRequestingCoordinates = True
			#rospy.wait_for_service('/resendCoords')
			try:
				rospy.loginfo("attempting to call resend coordinates service")
				resendcoords = rospy.ServiceProxy('/sendCoords', srvEmpty)
				resp1 = resendcoords()
			except rospy.ServiceException, e:
				print "send coordinates service call failed: %s"%e
			self.isRequestingCoordinates = False
		
	
	def WakeUpFollowers(self):
		# this will get called multiple times otherwise
		if not self.wakingUpOthers:
			self.wakingUpOthers = True
			#rospy.wait_for_service('/wakeupFollowers')
			try:
				rospy.loginfo("attempting to call wakeup followers service")
				wakeywakey = rospy.ServiceProxy('/wakeupFollowers', srvEmpty)
				resp1 = wakeywakey()
			except rospy.ServiceException, e:
				print "wakeup followers service call failed: %s"%e
			self.isRequestingCoordinates = False
		
			
		

	def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
		# Called by the main program to set the current command
		self.command.linear.x  = pitch
		self.command.linear.y  = roll
		self.command.linear.z  = z_velocity
		self.command.angular.z = yaw_velocity

	def SendCommand(self,event):
		# The previously set command is then sent out periodically if the drone is flying
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)



# define the default mapping between joystick buttons and their corresponding actions
ButtonEmergency = 0
ButtonLand      = 1
ButtonTakeoff   = 2
ButtonSendCoords = 6
ButtonResetPTAM = 7
ButtonFollowersLeave = 8
ButtonWakeUpFollowers = 9


# define the default mapping between joystick axes and their corresponding directions
AxisRoll        = 0
AxisPitch       = 1
AxisYaw         = 3
AxisZ           = 4

# define the default scaling to apply to the axis inputs. useful where an axis is inverted
ScaleRoll       = 1.0
ScalePitch      = 1.0
ScaleYaw        = 1.0
ScaleZ          = 1.0

# handles the reception of joystick packets
def ReceiveJoystickMessage(data):
	if data.buttons[ButtonEmergency]==1:
		rospy.loginfo("Emergency Button Pressed")
		controller.SendEmergency()
	elif data.buttons[ButtonLand]==1:
		rospy.loginfo("Land Button Pressed")
		controller.SendLand()
	elif data.buttons[ButtonTakeoff]==1:
		rospy.loginfo("Takeoff Button Pressed")
		controller.SendTakeoff()
	elif data.buttons[ButtonFollowersLeave]==1:
		rospy.loginfo("Followers Leave Button Pressed")
		controller.SendFollowersLeave()
	elif data.buttons[ButtonSendCoords]==1:
		rospy.loginfo("send Coordinates Button Pressed")
		controller.SendCoordinates()
	elif data.buttons[ButtonWakeUpFollowers]==1:
		rospy.loginfo("Wake Up Followers Button Pressed")
		controller.WakeUpFollowers()	
	else:
		controller.SetCommand(data.axes[AxisRoll]/ScaleRoll,data.axes[AxisPitch]/ScalePitch,data.axes[AxisYaw]/ScaleYaw,data.axes[AxisZ]/ScaleZ)


# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_TargetAggregation_joystick_interpreter')

	# Next load in the parameters from the launch-file
	ButtonEmergency = int (   rospy.get_param("~ButtonEmergency",ButtonEmergency) )
	ButtonLand      = int (   rospy.get_param("~ButtonLand",ButtonLand) )
	ButtonTakeoff   = int (   rospy.get_param("~ButtonTakeoff",ButtonTakeoff) )
	AxisRoll        = int (   rospy.get_param("~AxisRoll",AxisRoll) )
	AxisPitch       = int (   rospy.get_param("~AxisPitch",AxisPitch) )
	AxisYaw         = int (   rospy.get_param("~AxisYaw",AxisYaw) )
	AxisZ           = int (   rospy.get_param("~AxisZ",AxisZ) )
	ScaleRoll       = float ( rospy.get_param("~ScaleRoll",ScaleRoll) )
	ScalePitch      = float ( rospy.get_param("~ScalePitch",ScalePitch) )
	ScaleYaw        = float ( rospy.get_param("~ScaleYaw",ScaleYaw) )
	ScaleZ          = float ( rospy.get_param("~ScaleZ",ScaleZ) )

	# Now we construct our Qt Application and associated controllers and windows
	controller = TUMDroneController()

	# subscribe to the /joy topic and handle messages of type Joy with the function ReceiveJoystickMessage
	subJoystick = rospy.Subscriber('/joy', Joy, ReceiveJoystickMessage)
	
	rospy.spin()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	
