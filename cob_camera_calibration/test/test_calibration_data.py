#!/usr/bin/env python
PKG = 'cob_camera_calibration'
import roslib; roslib.load_manifest(PKG)
import unittest

from cob_camera_calibration import CalibrationData

TEST_FILE = "test/data/calibration_data.yaml"
TEST_OUT = "/tmp/calibration_data.yaml"

class TestCalibrationData(unittest.TestCase):
    def test_read(self):
        data = CalibrationData("camera", "frame", "image_width", "image_height")
        data.read_camera_yaml_file(TEST_FILE)
        
    def test_read_write(self):
        data = CalibrationData("camera", "frame", "image_width", "image_height")
        data.read_camera_yaml_file(TEST_FILE)
        data.save_camera_yaml_file(TEST_OUT)
        
        # compare with correct results
        self.assertTrue(self._cmp_files(TEST_FILE, TEST_OUT))
        
    def _cmp_files(self, f1, f2):
        return open(f1).read() == open(f2).read()
                
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestCalibrationData', TestCalibrationData)
