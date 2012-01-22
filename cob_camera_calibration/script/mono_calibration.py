#!/usr/bin/env python
PKG  = 'cob_camera_calibration'
NODE = 'mono_calibration_node'
import roslib; roslib.load_manifest(PKG)
import rospy

import numpy as np
from cob_camera_calibration import Checkerboard, CheckerboardDetector, MonoCalibrator, CalibrationData

class MonoCalibrationNode():
    '''
    Monocular Calibration ROS Node

    Runs monocular calibration on a set of images. All settings are configurable via
    the ROS parameter server.
    '''

    def __init__(self):
        '''
        Configures the calibration node

        Reads configuration from parameter server or uses default values
        '''
        rospy.init_node(NODE)
        print "==> started " + NODE
        
        # get parameter from parameter server or set defaults
        self.folder        = rospy.get_param('~folder',             ".")
        self.pattern_size  = rospy.get_param('~pattern_size',       "9x6")
        self.square_size   = rospy.get_param('~square_size',        0.03)
        
        self.image_prefix  = rospy.get_param('~image_prefix',       "camera")
        self.camera_name   = rospy.get_param('~camera_name',        "camera")
        self.frame_id      = rospy.get_param('~frame_id',           "/camera")
        self.output_file   = rospy.get_param('~output_file',        self.camera_name+".yaml")
        
        self.alpha         = rospy.get_param('~alpha',              0.0)
        self.verbose       = rospy.get_param('~verbose',            False)
        
        # split pattern_size string into tuple, e.g '9x6' -> tuple(9,6)
        self.pattern_size = tuple((int(self.pattern_size.split("x")[0]), int(self.pattern_size.split("x")[1])))

    def run_mono_calibration(self):
        '''
        Runs the calibration
        '''
        print "==> starting monocular calibration"
        
        # set up Checkerboard, CheckerboardDetector and MonoCalibrator
        board       = Checkerboard(self.pattern_size, self.square_size)
        detector    = CheckerboardDetector(board)
        calibrator  = MonoCalibrator(board, detector, self.folder, self.image_prefix)
        
        # run calibration
        (rms, camera_matrix, projection_matrix, dist_coeffs, (h, w), _, _) = calibrator.calibrate_monocular_camera(self.alpha)
        print "==> successfully calibrated, reprojection RMS (in pixels):", rms
        
        # create CalibrationData object with results
        camera_info                         = CalibrationData(self.camera_name, self.frame_id, w, h)
        camera_info.camera_matrix           = camera_matrix
        camera_info.distortion_coefficients = dist_coeffs
        camera_info.projection_matrix       = projection_matrix
        camera_info.dist_coeffs             = dist_coeffs
        
        # save results
        camera_info.save_camera_yaml_file(self.output_file)
        print "==> saved results to:", self.output_file
    
        # verbose mode
        if self.verbose:
            print "--> results:"
            np.set_printoptions(suppress=1)
            print "camera matrix:\n", camera_matrix
            print "distortion coefficients:\n", dist_coeffs            
            print "projection matrix:\n", projection_matrix

if __name__ == '__main__':
    node = MonoCalibrationNode()
    node.run_mono_calibration()
    print "==> done! exiting..."
