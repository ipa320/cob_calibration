#!/usr/bin/env python
PKG = "cob_laser_calibration"
NODE = "laser_scan_calibration"

### Details ###
# Script: 
# Author: Daniel Maeki (ipa-fxm-dm)
# Supervisor: Felix Messmer (ipa-fxm)

### Imports ###
import roslib; roslib.load_manifest(PKG); roslib.load_manifest('gazebo')
import rospy
import numpy as np
import tf
import cv
import cv2
from math import sqrt, sin, cos, tanh
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from gazebo.srv import GetModelState
from visualize_laser_scan import Get_laserscan, Visualize_laserscan, Detect_calibration_object

### Global variables ###
cal_obj_pose = ((1,0,0),(0,0,0,1)) #												set approximate position of calibration object to test results
scanner_location = 'front' #														Specify 'front' or 'rear' for the location of the laser scanner
scan_amount = 30 # 																	Amount of scans to be merged
resolution = 200 # 																	resolution of the laser scan image
border = 20 # 																		border of pixels around the image
max_laser_point_dist = 4.0 # 																maximum distance for detecting calibration object
frustum = {} #																		frustum properties
frustum["dim"] = (0.1*resolution, 0.05*resolution, 0.2*resolution) #				dimensions of frustum (bottom_radius, top_radius, height)
frustum["amount"] = 3 # 															Amount of frustums on calibration object
frustum["dist"] = 0.38*resolution #													distance in meters between centers of frustums
frustum["min_dist"] = frustum["dist"]-(frustum["dim"][0]-frustum["dim"][1]) #		minimum distance in centimeters between detected frustums
frustum["max_dist"] = frustum["dist"]+(frustum["dim"][0]-frustum["dim"][1]) #		maximum distance in centimeters between detected frustums


class Listener():
	def __init__(self):
		rospy.init_node(NODE)
		scan = None
		image = None

	def callback(self, data):
		self.laserscan = data

	def run_in_simulation_mode(self):
		print "Simulation mode activated"
		try:
			get_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
			cal_obj_pose_sim = get_pose('calibration_object', '')
			cal_obj_pose = PoseStamped()
			cal_obj_pose.header.stamp = rospy.Time.now()
			cal_obj_pose.header.frame_id = '/base_footprint'
			cal_obj_pose.pose.position = cal_obj_pose_sim.pose.position
			cal_obj_pose.pose.orientation = cal_obj_pose_sim.pose.orientation
		except(rospy.ROSException), e:
			print "Calibration_object not spawned, aborting..."
			print e
			exit()
		return cal_obj_pose

	def run(self):
		tf_listener = tf.TransformListener()
		while not rospy.is_shutdown():
			
			# get real pose of calibration object with respect to base when running in simulation
			try:
				rospy.wait_for_service('/gazebo/get_model_state', timeout=1)
				self.cal_obj_pose = self.run_in_simulation_mode()
				#print "self.cal_obj_pose: ", self.cal_obj_pose
				if self.cal_obj_pose is not None:
					real_cal_obj_pose = ((self.cal_obj_pose.pose.position.x,
										self.cal_obj_pose.pose.position.y,
										self.cal_obj_pose.pose.position.z),
										(self.cal_obj_pose.pose.orientation.x,
										self.cal_obj_pose.pose.orientation.y,
										self.cal_obj_pose.pose.orientation.z,
										self.cal_obj_pose.pose.orientation.w))
			except(rospy.ROSException):
				real_cal_obj_pose = cal_obj_pose
				print "Running in real-live mode"
			
			# get pose of laser scanner with respect to base
			try:
				tf_listener.waitForTransform('/base_laser_'+scanner_location+'_link', '/base_footprint', rospy.Time(), rospy.Duration(1))
				laserscan_pose = tf_listener.lookupTransform('/base_laser_'+scanner_location+'_link', '/base_footprint', rospy.Time())
				print "laserscan_pose: ", laserscan_pose
			except(tf.Exception), e:
				print "Laser scan transform not available, aborting..."
				print e
				exit()
			
			# get raw data from laser scanner
			try:
				rospy.Subscriber('/scan_'+scanner_location, LaserScan, self.callback)
				rospy.wait_for_message('/scan_'+scanner_location, LaserScan, timeout=2)
			except(rospy.ROSException), e:
				print "Laser scan topic not available, aborting..."
				print e
				exit()
			
			# get average scan over the amount of scans specified in scan_amount and remove all null detections
			laserscan_data = Get_laserscan()
			while laserscan_data.get_count() < scan_amount:
				laserscan_data.append_laserscan(self.laserscan)
			scan = laserscan_data.get_result()
			print "-> Scan received"
			
			# create and draw an image from laser scan data
			visualize = Visualize_laserscan(resolution, border, max_laser_point_dist, laserscan_pose)
			image, origin = visualize.convert_to_image(scan)
			print "-> Image received"
			
			# find calibration object from the image and determine it's cartesian pose with respect to the laser scanner
			detect = Detect_calibration_object(resolution, origin, frustum)
			image, detected_cal_obj_pose = detect.detect_cal_object(image)
			#print "detected_cal_obj_pose: ", detected_cal_obj_pose
			print "-> Calibration object detected"
			
			#set quaternions of calibration object pose with respect to the laser scanner
			if detected_cal_obj_pose is not None:
				cal_obj_pose_to_laser = PoseStamped()
				cal_obj_pose_to_laser.header.stamp = rospy.Time.now()
				cal_obj_pose_to_laser.header.frame_id = '/base_laser_'+scanner_location+'_link'
				cal_obj_pose_to_laser.pose.position.x = detected_cal_obj_pose[0][0]
				cal_obj_pose_to_laser.pose.position.y = detected_cal_obj_pose[0][1]
				cal_obj_pose_to_laser.pose.position.z = detected_cal_obj_pose[0][2]
				cal_obj_pose_quaternions = tf.transformations.quaternion_from_euler(detected_cal_obj_pose[1][0], detected_cal_obj_pose[1][1], detected_cal_obj_pose[1][2])
				cal_obj_pose_to_laser.pose.orientation.x = cal_obj_pose_quaternions[0]
				cal_obj_pose_to_laser.pose.orientation.y = cal_obj_pose_quaternions[1]
				cal_obj_pose_to_laser.pose.orientation.z = cal_obj_pose_quaternions[2]
				cal_obj_pose_to_laser.pose.orientation.w = cal_obj_pose_quaternions[3]
				#print "cal_obj_pose_to_laser: ", cal_obj_pose_to_laser
					
				#get calibration object pose with respect to base
				try:
					tf_listener.waitForTransform('/base_laser_'+scanner_location+'_link', '/base_footprint', rospy.Time.now(), rospy.Duration(1))
					cal_obj_pose_to_base = tf_listener.transformPose('/base_footprint', cal_obj_pose_to_laser)
					#print "cal_obj_pose_to_base: ", cal_obj_pose_to_base
					detected_cal_obj_pose = ((cal_obj_pose_to_base.pose.position.x,
											cal_obj_pose_to_base.pose.position.y,
											cal_obj_pose_to_base.pose.position.z),
											(cal_obj_pose_to_base.pose.orientation.x,
											cal_obj_pose_to_base.pose.orientation.y,
											cal_obj_pose_to_base.pose.orientation.z,
											cal_obj_pose_to_base.pose.orientation.w))
				except(tf.Exception), e:
					print "Was not able to transform pose between calibration object and base, aborting..."
					print e
					exit()
				
				#print results
				print "COMPARISON BETWEEN REAL AND DETECTED x,y"
				print "real pose with respect to base ="
				print real_cal_obj_pose
				print "detected pose with respect to base ="
				print detected_cal_obj_pose
			
			if resolution <= 200: # Image becomes too large for viewing when resolution is above 200.
				visualize.show_image(image)
			else:
				print ""
				raw_input("Press enter to recalculate calibration object pose with respect to base")
			print ""
			print ""


if __name__ == "__main__":
	l = Listener()
	l.run()
