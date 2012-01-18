#!/usr/bin/env python
PKG = 'cob_robot_calibration'
import roslib; roslib.load_manifest(PKG)
import unittest

from  cob_robot_calibration import calibration_urdf_updater 

FILE_IN = "test/data/test_calibration.urdf.xacro"
FILE_OUT1 = "/tmp/test_calibration.urdf.xacro_out1"
FILE_OUT2 = "/tmp/test_calibration.urdf.xacro_out2"
FILE_OUT_RES1 = "test/data/test_calibration.urdf.xacro_corr-result1"
FILE_OUT_RES2 = "test/data/test_calibration.urdf.xacro_corr-result2"

class TestCalibrationUrdfUpdater(unittest.TestCase):
    def test_update_one_param(self):
        # define params to update
        attributes2update = {'a': 1.0}
        
        # do update
        updater = calibration_urdf_updater.CalibrationUrdfUpdater(FILE_IN, FILE_OUT1)
        updater.update(attributes2update)
        
        # compare with correct results
        self.assertTrue(self._cmp_files(FILE_OUT1, FILE_OUT_RES1))
        
    def test_update_one_param_string(self):
        # define params to update
        attributes2update = {'a': "1.0"}
        
        # do update
        updater = calibration_urdf_updater.CalibrationUrdfUpdater(FILE_IN, FILE_OUT1)
        updater.update(attributes2update)
        
        # compare with correct results
        self.assertTrue(self._cmp_files(FILE_OUT1, FILE_OUT_RES1))
        
    def test_update_all_params(self):
        # define params to update
        attributes2update = {'a': 1.0}
        attributes2update = {'b': 2.0}
        attributes2update = {'c': 3.0}
        
        # do update
        updater = calibration_urdf_updater.CalibrationUrdfUpdater(FILE_IN, FILE_OUT2)
        updater.update(attributes2update)
        
        # compare with correct results
        self.assertTrue(self._cmp_files(FILE_OUT2, FILE_OUT_RES2))
        
    def test_update_all_params_string(self):
        # define params to update
        attributes2update = {'a': "1.0"}
        attributes2update = {'b': "2.0"}
        attributes2update = {'c': "3.0"}
        
        # do update
        updater = calibration_urdf_updater.CalibrationUrdfUpdater(FILE_IN, FILE_OUT2)
        updater.update(attributes2update)
        
        # compare with correct results
        self.assertTrue(self._cmp_files(FILE_OUT2, FILE_OUT_RES2))

    def _cmp_files(self, f1, f2):
        return open(f1).read() == open(f2).read()

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'TestCalibrationUrdfUpdater', TestCalibrationUrdfUpdater)
