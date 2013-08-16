#!/usr/bin/env python
# note: run script from this files directory

files = ['determine_cal_obj_pose', 'visualize_laser_scan', 'detect_cylinders', 'save_detections_to_file']
open('../README/comment_lines.txt', 'w').close()

for f in files:
	fileHandle = open(f+'.py', 'r')
	all_lines = fileHandle.readlines()
	fileHandle.close()
	comment_lines = []
	for line in all_lines:
		for char in line:
			if char == '#':
				comment_lines.append(line[line.find('#'):])
				break
	fileHandle = open('../README/comment_lines.txt', 'a')
	for comment in comment_lines:
		fileHandle.write(comment)
	fileHandle.close()
