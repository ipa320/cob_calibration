#!/usr/bin/env python
PKG  = 'cob_camera_calibration'
NODE = 'stereo_calibration_node'
import roslib; roslib.load_manifest(PKG)
import rospy

import numpy as np
import tf
from cob_camera_calibration import Checkerboard, CheckerboardDetector, StereoCalibrator, CalibrationData, CalibrationUrdfUpdater

class StereoCalibrationNode():
    '''
    Stereo Calibration ROS Node

    Runs stereo calibration on a set of images. All settings are configurable via
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
        self.folder               = rospy.get_param('~folder',               ".")
        self.pattern_size         = rospy.get_param('~pattern_size',         "9x6")
        self.square_size          = rospy.get_param('~square_size',          0.03)
                                  
        self.image_prefix_l       = rospy.get_param('~image_prefix_l',       "left")
        self.camera_name_l        = rospy.get_param('~camera_name_l',        "left")
        self.frame_id_l           = rospy.get_param('~frame_id_l',           "/left")
        self.output_file_l        = rospy.get_param('~output_file_l',        self.camera_name_l+".yaml")
                                  
        self.image_prefix_r       = rospy.get_param('~image_prefix_r',       "right")
        self.camera_name_r        = rospy.get_param('~camera_name_r',        "right")
        self.frame_id_r           = rospy.get_param('~frame_id_r',           "/right")
        self.output_file_r        = rospy.get_param('~output_file_r',        self.camera_name_r+".yaml")
        
        self.calibration_urdf_in  = rospy.get_param('~calibration_urdf_in',  "")
        self.calibration_urdf_out = rospy.get_param('~calibration_urdf_out', "")
        self.baseline_prop_prefix = rospy.get_param('~baseline_prop_prefix', "cam_r_")
        
        self.alpha                = rospy.get_param('~alpha',                0.0)
        self.verbose              = rospy.get_param('~verbose',              False)
        
        # split pattern_size string into tuple, e.g '9x6' -> tuple(9,6)
        self.pattern_size = tuple((int(self.pattern_size.split("x")[0]), int(self.pattern_size.split("x")[1])))

    def run_stereo_calibration(self):
        '''
        Runs the calibration
        '''
        # set up Checkerboard, CheckerboardDetector and MonoCalibrator
        board       = Checkerboard(self.pattern_size, self.square_size)
        detector    = CheckerboardDetector(board)
        calibrator  = StereoCalibrator(board, detector, self.folder, self.image_prefix_l, self.image_prefix_r)
        
        # run calibration
        ((rms_l, rms_r, rms_stereo), camera_matrix_l, dist_coeffs_l, rectification_matrix_l, projection_matrix_l,
                                     camera_matrix_r, dist_coeffs_r, rectification_matrix_r, projection_matrix_r,
                                     (h, w), R, T) = calibrator.calibrate_stereo_camera(self.alpha)
        print "==> successfully calibrated, stereo reprojection RMS (in pixels):", rms_stereo
        
        # create CalibrationData object with results
        camera_info_l                         = CalibrationData(self.camera_name_l, self.frame_id_l, w, h)
        camera_info_l.camera_matrix           = camera_matrix_l
        camera_info_l.rectification_matrix    = rectification_matrix_l
        camera_info_l.projection_matrix       = projection_matrix_l
        camera_info_l.distortion_coefficients = dist_coeffs_l
        
        camera_info_r                         = CalibrationData(self.camera_name_r, self.frame_id_r, w, h)
        camera_info_r.camera_matrix           = camera_matrix_r
        camera_info_r.rectification_matrix    = rectification_matrix_r
        camera_info_r.projection_matrix       = projection_matrix_r
        camera_info_r.distortion_coefficients = dist_coeffs_r
        
        # save results
        camera_info_l.save_camera_yaml_file(self.output_file_l)
        print "==> saved left results to:", self.output_file_l
        
        camera_info_r.save_camera_yaml_file(self.output_file_r)
        print "==> saved right results to:", self.output_file_r
    
        # convert baseline (invert transfrom as T and R bring right frame into left 
        # and we need transform from left to right for urdf!)
        M = np.matrix(np.vstack((np.hstack((R, T)), [0.0, 0.0, 0.0, 1.0]))) # 4x4 homogeneous matrix
        M_inv = M.I
        T_inv = np.array(M_inv[:3,3]).flatten().tolist() # T as list (x, y, z)
        R_inv = list(tf.transformations.euler_from_matrix(M_inv[:3,:3])) # convert R to (roll, pitch, yaw)
    
        # save baseline
        if (self.calibration_urdf_in != "" and self.calibration_urdf_out != ""):
            attributes2update = {self.baseline_prop_prefix+'x':     T_inv[0],
                                 self.baseline_prop_prefix+'y':     T_inv[1],
                                 self.baseline_prop_prefix+'z':     T_inv[2],
                                 self.baseline_prop_prefix+'roll':  R_inv[0],
                                 self.baseline_prop_prefix+'pitch': R_inv[1],
                                 self.baseline_prop_prefix+'yaw':   R_inv[2]}
            urdf_updater = CalibrationUrdfUpdater(self.calibration_urdf_in, self.calibration_urdf_out, self.verbose)
            urdf_updater.update(attributes2update)
            print "==> updated baseline in:", self.calibration_urdf_out
        else:
            print "==> NOT saving baseline to urdf file! Parameters 'calibration_urdf_in' and/or 'calibration_urdf_out' are empty..."     
        
        # verbose mode
        if self.verbose:
            print "--> results:"
            np.set_printoptions(suppress=1)
            print "left\n----"
            print "rms left monocular calibration:", rms_l
            print "camera matrix:\n", camera_matrix_l
            print "distortion coefficients:\n", dist_coeffs_l
            print "rectification matrix:\n", rectification_matrix_l
            print "projection matrix:\n", projection_matrix_l
            print 
            print "right\n-----"
            print "rms right monocular calibration:", rms_r
            print "camera matrix:\n", camera_matrix_r
            print "distortion coefficients:\n", dist_coeffs_r
            print "rectification matrix:\n", rectification_matrix_r
            print "projection matrix:\n", projection_matrix_r
            print 
            print "baseline (transform from left to right camera)\n--------"
            print "R (in rpy):\n", R_inv
            print "T (in xyz):\n", T_inv

if __name__ == '__main__':
    node = StereoCalibrationNode()
    node.run_stereo_calibration()
    print "==> done! exiting..."
