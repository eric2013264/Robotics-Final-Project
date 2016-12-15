import rospy
import time
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from math import radians

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


def process_image(image):
	hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    	lower_bound = np.array([25, 100, 100]) # lower HSV for yellow
    	upper_bound = np.array([35, 255, 255]) # upper HSV for yellow
	mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
	cv2.imshow("image_mask", mask)
	M = cv2.moments(mask)
	location = None
	magnitude = 0
	if M['m00'] > 0:
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		magnitude = M['m00']
		location = (cx-320, cy-240)
		cv2.circle(image, (cx,cy), 3, (0,0,255), -1)
	cv2.imshow("processing result", image)
	cv2.waitKey(1)
	return location, magnitude




class Run():

	def __init__(self):

		rospy.init_node('Run', anonymous=False)
		rospy.loginfo("To stop TurtleBot CTRL + C") 
		rospy.on_shutdown(self.shutdown)

		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=1)
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		self.location = (0,0)
		self.magnitude = 0
		self.move = Twist()

		self.r = rospy.Rate(10);

		while True:
			pass

	#THIS FUNCTION GETS CALLED WHEN RECEIVES NEW IMAGE, DELEGATE (LIKE SEPEARATE THREAD)
	def image_callback(self, ros_image):
		cv_image = np.asarray(ToOpenCV(ros_image))
		loc, mag= process_image(cv_image)

		captured = 0
		print(mag)

		if loc == None or mag < 500000:
			self.move.angular.z = radians(-50)
			self.move.linear.x = 0
			self.cmd_vel.publish(self.move)
			     
		elif mag < 8000000 and captured  == 0 :
			self.move.linear.x = 0.2
			self.move.angular.z = radians((loc[0]*-1)/5.0)

			self.cmd_vel.publish(self.move)
		else: 
			captured = 1
			if mag <  1000000 and (0 < loc[0]):
				captured = 0
			self.move.linear.x = 0
			self.move.angular.z = radians(-90)
			self.cmd_vel.publish(self.move)


		self.r.sleep()
			

	def shutdown(self):
		rospy.loginfo("Stop TurtleBot")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)
 
if __name__ == '__main__':
	try:
		tbot = Run()
		tbot.seek()
	except:
		pass






