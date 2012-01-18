#!/usr/bin/env python
PKG = 'cob_robot_calibration'
import roslib; roslib.load_manifest(PKG)
import unittest

import yaml
from cob_robot_calibration import camera_yaml_updater 

FILE_IN = "test/data/test_camera_cal.yaml"
FILE_OUT = "/tmp/test_camera_cal.yaml_out"
FILE_OUT_RES_POS = "test/data/test_camera_cal.yaml_corr-result_pos"
FILE_OUT_RES_NEG = "test/data/test_camera_cal.yaml_corr-result_neg"

class TestCameraYamlUpdater(unittest.TestCase):
    def test_update_baseline_zero(self):
        # do update
        updater = camera_yaml_updater.CameraYamlUpdater(FILE_IN, FILE_OUT)
        updater.update_baseline(0.0)
        
        # compare with correct results
        self.assertTrue(self._cmp_yaml_files(FILE_OUT, FILE_IN))
        
    def test_update_baseline_positive(self):
        # do update
        updater = camera_yaml_updater.CameraYamlUpdater(FILE_IN, FILE_OUT)
        updater.update_baseline(10.0)
        
        # compare with correct results
        self.assertTrue(self._cmp_yaml_files(FILE_OUT, FILE_OUT_RES_POS))

    def test_update_baseline_negative(self):
        # do update
        updater = camera_yaml_updater.CameraYamlUpdater(FILE_IN, FILE_OUT)
        updater.update_baseline(-10.0)
        
        # compare with correct results
        self.assertTrue(self._cmp_yaml_files(FILE_OUT, FILE_OUT_RES_NEG))

    def _cmp_yaml_files(self, f1, f2):
        return yaml.load(file(f1)) == yaml.load(file(f2))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'TestCameraYamlUpdater', TestCameraYamlUpdater)