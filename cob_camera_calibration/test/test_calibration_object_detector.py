#!/usr/bin/env python
PKG = 'cob_camera_calibration'
import roslib; roslib.load_manifest(PKG)
import unittest

import numpy as np
import cv2
import yaml

from cob_camera_calibration import CheckerboardDetector, Checkerboard

CHECKERBOARD_IMAGE="test/data/cb1.jpg"
CHECKERBOARD_IMAGE_EXPECTED_POINTS_YAML = "test/data/cb1_points.yaml"

class TestCalibrationObjectDetector(unittest.TestCase):
    def test_detect_image_points_gray_image(self):
        # create Checkerboard instance and read image
        c = Checkerboard((9, 6), 0.03)
        image = cv2.imread(CHECKERBOARD_IMAGE, 1)
        
        # create CheckerboardDetector instance and detect image points
        cd = CheckerboardDetector(c)
        points = cd.detect_image_points(image, False)
        
        #dump = {'points': points.flatten().tolist()}
        #yaml.dump(dump, open(CHECKERBOARD_IMAGE_EXPECTED_POINTS_YAML, "w"))
        
        # load expected points
        points_expected = yaml.load(file(CHECKERBOARD_IMAGE_EXPECTED_POINTS_YAML))['points']
        self.assertTrue(np.allclose(points.flatten().tolist(), points_expected))
  
    def test_detect_image_points_color_image(self):
        # create Checkerboard instance and read image
        c = Checkerboard((9, 6), 0.03)
        image = cv2.imread(CHECKERBOARD_IMAGE, 0)
        
        # create CheckerboardDetector instance and detect image points
        cd = CheckerboardDetector(c)
        points = cd.detect_image_points(image, True)
        
        #dump = {'points': points.flatten().tolist()}
        #yaml.dump(dump, open(CHECKERBOARD_IMAGE_EXPECTED_POINTS_YAML, "w"))
        
        # load expected points
        points_expected = yaml.load(file(CHECKERBOARD_IMAGE_EXPECTED_POINTS_YAML))['points']
        self.assertTrue(np.allclose(points.flatten().tolist(), points_expected))
        
    def test_calculate_object_pose(self):
        # create Checkerboard instance and read image
        c = Checkerboard((9, 6), 0.03)
        image = cv2.imread(CHECKERBOARD_IMAGE, 0)
        
        # create CheckerboardDetector instance and detect object pose 
        # (only dummy matrices, does not correspond to real camera used)
        cd = CheckerboardDetector(c)
        camera_matrix   = np.matrix([1200, 0, 600, 0, 1200, 600, 0, 0, 1], dtype=np.float32).reshape((3,3))
        dist_coeffs     = np.matrix([0, 0, 0, 0, 0], dtype=np.float32).reshape((1,5))
        (rvec, tvec) = cd.calculate_object_pose(image, camera_matrix, dist_coeffs, True)
        
        rvec_expected = np.matrix([[0.99905897,  0.035207,   -0.02533079],
                                   [-0.0208742,   0.90224111,  0.43072642],
                                   [ 0.03801906, -0.42979234,  0.90212699]])
        tvec_expected = np.matrix([[-0.03181351], [-0.13593217], [ 0.7021291]])

        self.assertTrue(np.allclose(rvec_expected, rvec))
        self.assertTrue(np.allclose(tvec_expected, tvec))
                
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestCalibrationObjectDetector', TestCalibrationObjectDetector)
