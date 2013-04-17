#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_calibration_executive')
import unittest
import numpy as np
from cob_calibration_executive.generate_positions import calculate_angles


class Test_calculate_angle_test(unittest.TestCase):

    def test_pan_1(self):
        transformation = [0, 0, 1]
        self.assertAlmostEqual(calculate_angles(transformation)[0], 0)

    def test_pan_2(self):
        transformation = [0, 1, 1]
        self.assertAlmostEqual(calculate_angles(transformation)[0], np.pi / 4)

    def test_pan_3(self):
        transformation = [0, 1, 0]
        self.assertAlmostEqual(calculate_angles(transformation)[0], np.pi / 2)

    def test_pan_2_negative(self):
        transformation = [0, -1, 1]
        self.assertAlmostEqual(calculate_angles(transformation)[0], -np.pi / 4)

    def test_pan_4_negative(self):
        transformation = [0, 1, -1]
        self.assertAlmostEqual(calculate_angles(transformation)[0], 3 * np.pi / 4)

    def test_tilt_1(self):
        transformation = [0, 0, 1]
        self.assertAlmostEqual(calculate_angles(transformation)[1], 0)

    def test_tilt_2(self):
        transformation = [1, 0, 1]
        self.assertAlmostEqual(calculate_angles(transformation)[1], np.pi / 4)

    def test_tilt_3(self):
        transformation = [1, 0, 0]
        self.assertAlmostEqual(calculate_angles(transformation)[1], np.pi / 2)

if __name__ == '__main__':
    unittest.main()
