#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image 			#for recieving video feed
from geometry_msgs.msg import Twist			# controlling the movements
from ardrone_autonomy.msg import Navdata 		#drone state
from std_msgs.msg import Empty 				#send empty message for takeoff and landing
from cv_bridge import CvBridge, CvBridgeError

from math import radians
import numpy 
import cv2
import time

bridge = CvBridge()

def ToOpenCV(ros_image):
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        return cv_image
    except CvBridgeError, e:
        print e
        raise Exception("Failed to convert to OpenCV image")

def depthToOpenCV(ros_image):
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "32FC1")
        return cv_image
    except CvBridgeError, e:
        print e
        raise Exception("Failed to convert to OpenCV image")

def ToRos(cv_image):
    try:
        ros_image = bridge.cv_to_imgmsg(cv_image, "bgr8")
        return ros_image
    except CvBridgeError, e:
        print e
        raise Exception("Failed to convert to ROS image")

class QuadcopterController:
	def __init__(self):
		
		self.subNavData = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavdata);
		self.image_sub = rospy.Subscriber('/ardrone/image_raw', Image, self.image_callback)
		#self.bottom_img = rospy.Subscriber('/ardrone/bottom/image_raw', Image, self.captured_callback)
		self.commandTimer = rospy.Timer(rospy.Duration(1), self.SendCommand)
		self.movement_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

		self.takeoff_time = time.time() + 5
		self.image = None
		self.status = None # number
		self.state = None  # words
		self.okToFly = False


	def ReceiveNavdata(self, navdata):
		self.status = navdata.state

	def SendCommand(self, event):

		if time.time() < self.takeoff_time or self.image == None:
			return

		image = ToOpenCV(self.image)
		location, magnitude = self.get_target_location(image)
		move = Twist()
		print("magnitude: ", magnitude)
		if location == None or magnitude != None and magnitude < 100000:
			move.angular.z = radians(50)
			move.linear.x = 0
			self.movement_pub.publish(move)
		elif magnitude < 38000000:
				 					# If we do, let's go toward it
			move.angular.z = radians((((location[0] - 320) / 5.0)*-1))
			move.linear.x = 0.03
			self.movement_pub.publish(move)
		else:
			move.angular.z = 0
			move.linear.x = 0
			self.movement_pub.publish(move)



	def image_callback(self, image):
		
		self.image = image

	def get_target_location(self, image):
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    		#lower_bound = numpy.array([64, 100, 100]) # lower HSV for green
		lower_bound = numpy.array([64, 100, 75]) # lower HSV for green #good one

		#lower_bound = numpy.array([64, 100, 70]) # lower HSV for green
    		upper_bound = numpy.array([150, 255, 255]) # upper HSV for green
		mask = cv2.inRange(hsv, lower_bound, upper_bound)
		cv2.imshow("image_mask", mask)
		M = cv2.moments(mask)
		location = None
		magnitude = 0
		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			magnitude = M['m00']
			location = (cx, cy)
			cv2.circle(image, (cx,cy), 3, (0,0,255), -1)

		cv2.imshow("camera", image)
		cv2.waitKey(1)
		return location, magnitude

#320, 230, RED forward Y increase, left red x increase


	#callback function that gets run when node shuts down
def shutdown_callback():
	print "shutting down..."
	drone_land_pub.publish(Empty()) #make the drone land
	cv2.destroyAllWindows() #cleanup opencv windows
	print "done!"


if __name__ == "__main__":
	rospy.init_node("quadcopter_controller")
	#ardrone uses specialized topics for landing and taking off
	drone_takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
	drone_land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
	controller = QuadcopterController()

	#register callback function to land drone if we kill the program
	rospy.on_shutdown(shutdown_callback) 

	rospy.sleep(1) #wait for a second to wait for node to fully connect

	drone_takeoff_pub.publish(Empty()) #command drone to takeoff
	print("Take off")

	#this function loops and waits for node to shutdown
	#all logic happens in the image_callback function
	rospy.spin()

	'''
	if self.status == 0:
		self.state = "Unknown"
	if self.status == 1:
		self.state = "Initialized"
	if self.status == 2:
		self.state = "Landed"
	if self.status == 3 or self.state == 7:
		self.state = "Flying"
	if self.status == 4:
		self.state = "Hovering"
		self.okToFly = True
	if self.status == 5:
		self.state = "Test"
	if self.status == 6:
		self.state = "Taking Off"
	if self.status == 8:
		self.state = "Landing"
	if self.status == 9:
		self.state = "Looping"
	
	print self.status
	'''


