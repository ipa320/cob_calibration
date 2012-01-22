#!/usr/bin/env python
PKG = 'cob_camera_calibration'
import roslib; roslib.load_manifest(PKG)
import unittest

import numpy as np
import cv2
from cob_camera_calibration import Checkerboard

class TestCalibrationObject(unittest.TestCase):
    def test_checkerboard_object_points_2x2(self):
        c = Checkerboard((2,2), 1)
        points = c.get_pattern_points()
        points_ = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]]
        self.assertTrue(np.allclose(points, points_))
    
    def test_checkerboard_object_points_2x2_scale(self):
        c = Checkerboard((2,2), 1.5)
        points = c.get_pattern_points()
        points_ = [[0, 0, 0], [1.5, 0, 0], [0, 1.5, 0], [1.5, 1.5, 0]]
        self.assertTrue(np.allclose(points, points_)) 
        
    def test_checkerboard_object_points_4x4(self):
        c = Checkerboard((4,4), 1)
        points = c.get_pattern_points()
        points_ = [[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0],
                   [0, 1, 0], [1, 1, 0], [2, 1, 0], [3, 1, 0],
                   [0, 2, 0], [1, 2, 0], [2, 2, 0], [3, 2, 0],
                   [0, 3, 0], [1, 3, 0], [2, 3, 0], [3, 3, 0]]
        self.assertTrue(np.allclose(points, points_)) 
                
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestCalibrationObject', TestCalibrationObject)
