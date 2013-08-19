#!/usr/bin/env python

### DETAILS ###
# Script: Write all comment lines of all files specified below into a .txt file
# Author: Daniel Maeki (ipa-fxm-dm)
# Supervisor: Felix Messmer (ipa-fxm)

### NOTES ###
# Run script from this file's directory


file_names = ['determine_cal_obj_pose', 'visualize_laser_scan', 'detect_cylinders', 'save_detections_to_file']
open('../README/comment_lines.txt', 'w').close() # <-- create or purge file

for name in file_names:
	fileHandle = open(name+'.py', 'r')
	all_lines = fileHandle.readlines()
	fileHandle.close()
	comment_lines = []
	for line in all_lines:
		for char in line:
			# If the character '#' is in the current line, copy everything behind it.
			if char == '#':
				comment_lines.append(line[line.find('#'):])
				break
	fileHandle = open('../README/comment_lines.txt', 'a')
	for comment in comment_lines:
		# Write each comment line into the file specified above
		fileHandle.write(comment)
	fileHandle.close()
