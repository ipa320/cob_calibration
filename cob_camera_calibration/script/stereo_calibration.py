#!/usr/bin/env python
#################################################################
##\file
#
# \note
#   Copyright (c) 2011-2012 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_calibration
# \note
#   ROS package name: cob_camera_calibration
#
# \author
#   Author: Sebastian Haug, email:sebhaug@gmail.com
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: November 2011
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################
PKG = 'cob_camera_calibration'
NODE = 'stereo_calibration_node'
import roslib
roslib.load_manifest(PKG)
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
        self.folder = rospy.get_param(
            '~folder', ".")
        self.pattern = rospy.get_param(
            '~calibration_pattern')
        self.output_path = rospy.get_param(
            '~output_file_path')
        self.cameras = rospy.get_param(
            '~cameras')

        # Reference Camera
        self.image_prefix_ref = self.cameras["reference"]["file_prefix"]
        self.camera_name_ref = self.cameras["reference"]["name"]
        self.frame_id_ref = self.cameras["reference"]["frame_id"]
        self.output_file_ref = self.output_path + self.cameras["reference"][
            "calibration_data_file"]

        # Dependent Cameras
        self.image_prefixes_dep = []
        self.camera_names_dep = []
        self.frame_ids_dep = []
        self.output_files_dep = []
        for cam in self.cameras["further"]:
            self.image_prefixes_dep.append(cam.get("file_prefix", None))
            self.camera_names_dep.append(cam.get("name", None))
            self.frame_ids_dep.append(cam.get("frame_id", None))
            self.output_files_dep.append(
                self.output_path + cam.get("calibration_data_file", None) if "calibration_data_file" in cam else None)

        self.calibration_urdf_in = rospy.get_param(
            '~calibration_urdf_in', "")
        self.calibration_urdf_out = rospy.get_param(
            '~calibration_urdf_out', "")

        self.baseline_prop_prefixes_dep = [rospy.get_param(
            '~baseline_prop_prefix', "cam_r_"),
            rospy.get_param('~baseline_prop_prefix', "cam3d_"),
            rospy.get_param('~baseline_prop_prefix', "cam3d_ir_")]

        self.alpha = rospy.get_param(
            '~alpha', 0.0)
        self.verbose = rospy.get_param(
            '~verbose', False)

        # split pattern_size string into tuple, e.g '9x6' -> tuple(9,6)
        self.pattern_size = tuple((int(self.pattern["pattern_size"].split(
            "x")[0]), int(self.pattern["pattern_size"].split("x")[1])))

    def run_stereo_calibration(self):
        '''
        Runs the calibration
        '''
        # set up Checkerboard, CheckerboardDetector and MonoCalibrator
        board = Checkerboard(self.pattern_size, self.pattern["square_size"])
        detector = CheckerboardDetector(board)
        attributes2update = {}
        for image_prefix_dep, camera_name_dep, frame_id_dep, output_file_dep, baseline_prop_prefix in zip(self.image_prefixes_dep, self.camera_names_dep, self.frame_ids_dep, self.output_files_dep, self.baseline_prop_prefixes_dep):
            if image_prefix_dep is not None:
                calibrator = StereoCalibrator(board, detector, self.folder,
                                              self.image_prefix_ref, image_prefix_dep)

                # run calibration for stereo pair
                (
                    (rms_ref, rms_dep, rms_stereo),
                    camera_matrix_ref, dist_coeffs_ref, rectification_matrix_ref, projection_matrix_ref,
                    camera_matrix_dep, dist_coeffs_dep, rectification_matrix_dep, projection_matrix_dep,
                    ((h_ref, w_ref), (h_dep, w_dep)), R, T) = calibrator.calibrate_stereo_camera(self.alpha)
                print "==> successfully calibrated, stereo reprojection RMS (in pixels):", rms_stereo

                # create CalibrationData object with results
                camera_info_ref = CalibrationData(
                    self.camera_name_ref, self.frame_id_ref, w_ref, h_ref)
                camera_info_ref.camera_matrix = camera_matrix_ref
                camera_info_ref.rectification_matrix = rectification_matrix_ref
                camera_info_ref.projection_matrix = projection_matrix_ref
                camera_info_ref.distortion_coefficients = dist_coeffs_ref

                camera_info_dep = CalibrationData(
                    camera_name_dep, frame_id_dep, w_dep, h_dep)
                camera_info_dep.camera_matrix = camera_matrix_dep
                camera_info_dep.rectification_matrix = rectification_matrix_dep
                camera_info_dep.projection_matrix = projection_matrix_dep
                camera_info_dep.distortion_coefficients = dist_coeffs_dep

                if output_file_dep is not None:

                    # save results
                    camera_info_ref.save_camera_yaml_file(self.output_file_ref)
                    print "==> saved left results to:", self.output_file_ref

                    camera_info_dep.save_camera_yaml_file(output_file_dep)
                    print "==> saved " + camera_name_dep + " results to:", output_file_dep

                # convert baseline (invert transfrom as T and R bring right frame into left
                # and we need transform from left to right for urdf!)
                M = np.matrix(np.vstack((np.hstack(
                    (R, T)), [0.0, 0.0, 0.0, 1.0])))  # 4x4 homogeneous matrix
                M_inv = M.I
                T_inv = np.array(
                    M_inv[:3, 3]).flatten().tolist()  # T as list (x, y, z)
                R_inv = list(tf.transformations.euler_from_matrix(
                    M_inv[:3, :3]))  # convert R to (roll, pitch, yaw)
                baseline_prop_prefix = 'offset_' + baseline_prop_prefix
                attributes2update[baseline_prop_prefix + 'x'] = T_inv[0]
                attributes2update[baseline_prop_prefix + 'y'] = T_inv[1]
                attributes2update[baseline_prop_prefix + 'z'] = T_inv[2]
                attributes2update[baseline_prop_prefix + 'roll'] = R_inv[0]
                attributes2update[baseline_prop_prefix + 'pitch'] = R_inv[1]
                attributes2update[baseline_prop_prefix + 'yaw'] = R_inv[2]
        # save baseline
        if (self.calibration_urdf_in != "" and self.calibration_urdf_out != ""):
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
            print "rms left monocular calibration:", rms_ref
            print "camera matrix:\n", camera_matrix_ref
            print "distortion coefficients:\n", dist_coeffs_ref
            print "rectification matrix:\n", rectification_matrix_ref
            print "projection matrix:\n", projection_matrix_ref
            print
            print "right\n-----"
            print "rms right monocular calibration:", rms_dep
            print "camera matrix:\n", camera_matrix_dep
            print "distortion coefficients:\n", dist_coeffs_dep
            print "rectification matrix:\n", rectification_matrix_dep
            print "projection matrix:\n", projection_matrix_dep
            print
            print "baseline (transform from left to right camera)\n--------"
            print "R (in rpy):\n", R_inv
            print "T (in xyz):\n", T_inv

if __name__ == '__main__':
    node = StereoCalibrationNode()
    node.run_stereo_calibration()
    print "==> done! exiting..."
