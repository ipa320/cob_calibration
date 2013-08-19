#!/usr/bin/env python

### DETAILS ###
# Script: Save results from determine_cal_obj_pose.py to a file
# Author: Daniel Maeki (ipa-fxm-dm)
# Supervisor: Felix Messmer (ipa-fxm)

### NOTES ###
# This script does not work, I have only copied code that I used in another script into here for easy access and potential usage later on.


class Save_detections():
	
	def write_detection():
		print "write_detection"
		fileHandle = open('../README/detections.txt', 'a')
		detection = "x=%s, y=%s, yaw=%s\n" %(str(avg_calibration_object_pose[0][0]), str(avg_calibration_object_pose[0][1]), str(avg_calibration_object_pose[1][2]))
		fileHandle.write(detection)
		fileHandle.close()
	
	def read_detections():
		print "read_detection"
		fileHandle = open('../README/detections.txt', 'r')
		lines = fileHandle.readlines()
		fileHandle.close()
		detections = []
		for line in lines:
			if 'x' in line:
				cal_obj_position = [float(line[line.find('x=')+2:line.find('y=')-2]), float(line[line.find('y=')+2:line.find('yaw=')-2]), 0]
				cal_obj_rotation = [0, 0, float(line[line.find('yaw=')+4:-1])]
				cal_obj_pose = [cal_obj_position, cal_obj_rotation]
				detections.append(cal_obj_pose)
	
	def modify_detections():
		print "modify_detection"
		avg_calibration_object_pose = [[0,0,0],[0,0,0]]
		counter = 0
		for detection in detections:
			for i in range(0,len(detection)):
				for j in range(0,len(detection[i])):
					avg_calibration_object_pose[i][j] = (avg_calibration_object_pose[i][j] * counter + detection[i][j]) / (counter + 1)
			counter += 1

		deviation = [[0,0,0],[0,0,0]]
		counter = 0
		for detection in detections:
			for i in range(0,len(detection)):
				for j in range(0,len(detection[i])):
					deviation[i][j] = (deviation[i][j] * counter + (detection[i][j] - avg_calibration_object_pose[i][j])) / (counter + 1)
			counter += 1
