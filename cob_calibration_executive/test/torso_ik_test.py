#!/usr/bin/env python

import unittest
import roslib
import numpy as np
roslib.load_manifest('cob_calibration_executive')
from cob_calibration_executive.torso_ik import TorsoIK


class Test_Torso_IK(unittest.TestCase):
    """tests for Torso_IK"""

    def setUp(self):
        self.torso = TorsoIK()
        self.torso.set_camera_viewfield(np.pi/6)

    def test_torso_configuration(self):
        assert self.torso.configuration == ['t', 'p', 't']

    def  test_limits(self):
        assert self.torso.limits > [0] * len(self.torso.limits)
        assert self.torso.limits < [2 * np.pi] * len(self.torso.limits)

    def test_maximum_angles(self):
        assert self.torso.max_angles['t'] == self.torso.limits[0] + \
            self.torso.limits[2]
        assert self.torso.max_angles['p'] == self.torso.limits[1]

    def test_ik_pan_in_limit(self):
        target_angles = {'p': -0.2, 't': 0}
        while target_angles['p'] <= 0.2:
            for i, v in enumerate(self.torso.configuration):
                self.assertAlmostEqual(self.torso.calculate_ik(
                    target_angles)[i], target_angles[v] / self.torso.n_joints[v])
            target_angles['p'] += 0.1

    def test_ik_tilt_in_limit(self):
        target_angles = {'t': -0.2, 'p': 0}
        while target_angles['t'] <= 0.2:
            for i, v in enumerate(self.torso.configuration):
                self.assertAlmostEqual(self.torso.calculate_ik(
                    target_angles)[i], target_angles[v] / self.torso.n_joints[v])
            target_angles['t'] += 0.1

    def test_ik_pan_off_limit(self):
        target_angles = {'p': -9.2, 't': 0}
        for i, v in enumerate(self.torso.configuration):
            self.assertAlmostEqual(self.torso.calculate_ik(
                target_angles)[i], -self.torso.limits[i] if v == 'p' else 0)

    def test_ik_tilt_off_limit(self):
        target_angles = {'t': -9.2, 'p': 0}
        for i, v in enumerate(self.torso.configuration):
            self.assertAlmostEqual(self.torso.calculate_ik(
                target_angles)[i], -self.torso.limits[i] if v == 't' else 0)

    def test_ik_off_limit(self):
        target_angles = {'t': -9.2, 'p': 5}
        for i, v in enumerate(self.torso.configuration):
            self.assertAlmostEqual(self.torso.calculate_ik(
                target_angles)[i], -self.torso.limits[i] if v == 't' else self.torso.limits[i])

    def test_off_range(self):
        target_angles = {'t': -9, 'p': 3}
        assert (self.torso.in_range(target_angles) is False)

    def test_in_range(self):
        target_angles = {'t': .5, 'p': 0.4}
        assert self.torso.in_range(target_angles) is True


if __name__ == '__main__':
    unittest.main()
