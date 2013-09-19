#!/usr/bin/env python
#
# \file
#
# \note
#   Copyright (c) 2011-2012 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#
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
#
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
#

import cv2
import os
import numpy as np

from . import cv2util


class Calibrator:

    '''
    Base class for specific camera calibrators
    '''
    def __init__(self):
        raise NotImplementedError(
            "Instantiate either MonoCalibrator or StereoCalibrator!")

    def _list_folder(self, folder):
        '''
        Return list with all files/folders in 'folder'

        @param folder: List files of this folder
        @type  folder: string
        '''
        files = sorted(os.listdir(folder))
        return map(lambda x: folder + '/' + x, files)

    def _load_images(self, folder, image_file_prefix, grayscale=False):
        '''
        Load set of images (jpg or png)

        Returns list of images stored as cv2 compatible numpy image.

        @param folder: Folder from where the images are located
        @type  folder: string

        @param image_file_prefix: Prefix the image files which are
                                  loaded must have
        @type  image_file_prefix: string

        @param grayscale: If true load images as grayscale
        @type  grayscale: bool
        '''
        images = []
        for all_files in self._list_folder(folder):
            filename = all_files.rsplit('/')[-1]

            # check if current file is image (i.e. png or jpg suffix)
            if filename.startswith(image_file_prefix) \
                    and (filename.lower().endswith('png')
                         or filename.lower().endswith('jpg')):
                if grayscale:
                    # load grayscale image
                    images.append(cv2.imread(
                        all_files, cv2.CV_LOAD_IMAGE_GRAYSCALE))
                else:
                    # load color image
                    images.append(cv2.imread(
                        all_files, cv2.CV_LOAD_IMAGE_COLOR))

        if len(images) == 0:
            raise Exception("Did not find any pictures with prefix",
                            image_file_prefix, "in folder", folder)
        return images

    def _detect_points(self, images, is_grayscale=True):
        '''
        Detect image and object points in set of images

        Returns list of image points and list of object points

        @param is_grayscale: If true images are assumend grayscale
        @type  is_grayscale: bool

        @param images: list of images stored as cv2 compatible numpy image
        @type  images: bool
        '''
        pattern_points = self.calibration_object.get_pattern_points()
        object_points = []
        image_points = []
        for image in images:
            try:
                corners = self.calibration_object_detector.detect_image_points(
                    image, is_grayscale)
            except self.calibration_object_detector.NoPatternFoundException:
                image_points.append(None)
                object_points.append(None)
            else:
                image_points.append(corners.reshape(-1, 2))
                object_points.append(pattern_points)
            assert(len(image_points) == len(object_points))
        if len(image_points) == 0:
            raise Exception(
                "Calibration object could not be detected in any image")
        return (image_points, object_points)


class MonoCalibrator(Calibrator):

    '''
    Calibrates a monocular camera
    '''
    def __init__(self, calibration_object, calibration_object_detector,
                 folder, image_prefix):
        '''
        Initialize object

        @param calibration_object: Calibration object to work with.
        @type  calibration_object: must implement CalibrationObject interface

        @param calibration_object_detector: Calibration object detector to work
                                            with. Must be compatible to
                                            CalibrationObject
        @type  calibration_object_detector: must implement
                                            CalibrationObjectDetector interface

        @param folder: Folder from where the images are located
        @type  folder: string

        @param image_file_prefix: Prefix the image files which are loaded must
                                  have
        @type  image_file_prefix: string
        '''
        self.calibration_object = calibration_object
        self.calibration_object_detector = calibration_object_detector
        self.folder = folder
        self.image_prefix = image_prefix

    def calibrate_monocular_camera(self, alpha=0.0):
        '''
        Run monocular calibration

        Returns calibration rms error (float), image height, width
        and camera_matrix, dist_coeffs, projection_matrix, rvecs, tvecs
        (as np.ndarray() type)

        @param alpha: Set zoom, alpha = 0.0 results in cropped image with all
                      pixels valid in undistorted image, alpha = 1.0 preserves
                      all pixels from original image
        @type  alpha: float (0.0 < alpha < 1.0)
        '''

        # load images
        print "--> loading images from folder '%s' with prefix '%s'" %  \
            (self.folder, self.image_prefix)
        images = self._load_images(
            self.folder, self.image_prefix, grayscale=True)
        print "--> loaded %i images" % len(images)

        # get width and height of images and check all
        (h, w) = images[0].shape[:2]
        for i in images:
            assert i.shape[:2] == (h, w)

        # detect image and object points in all images
        print "--> detecting calibration object in all images..."
        (image_points, object_points) = self._detect_points(
            images, is_grayscale=True)

        # prepare matrices
        camera_matrix = np.zeros((3, 3), np.float32)
        dist_coeffs = np.zeros((1, 5), np.float32)

        # flags
        flags = 0
        # flags |= cv2.CALIB_FIX_K3

        # calibrate camera
        print "--> calibrating..."
        rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            object_points, image_points, (w, h), camera_matrix, dist_coeffs,
            flags=flags)

        # set zoom value alpha, if alpha = 0.0 no black border from undistortion
        # will be visible for alpha = 1.0 everything from original image is
        # preserved
        (projection_matrix, _) = cv2.getOptimalNewCameraMatrix(
            camera_matrix, dist_coeffs, (w, h), alpha)
        projection_matrix = np.array(np.hstack((
            projection_matrix, np.matrix([0, 0, 0]).reshape(3, 1))))

         # DEBUG: check types -> should all be np.ndarray
         # print type(camera_matrix)
         # print type(projection_matrix)
         # print type(dist_coeffs)
         # print type(rvecs)
         # print type(tvecs)

        return (rms, camera_matrix, projection_matrix, dist_coeffs, (h, w),
                rvecs, tvecs)


class StereoCalibrator(Calibrator):

    '''
    Calibrates a stereo camera pair with parallel cameras
    '''
    def __init__(self, calibration_object, calibration_object_detector, folder,
                 image_prefix_left, image_prefix_right):
        '''
        Initialize object

        @param calibration_object: Calibration object to work with.
        @type  calibration_object: must implement CalibrationObject interface

        @param calibration_object_detector: Calibration object detector to work
                                            with. Must be compatible to
                                            CalibrationObject
        @type  calibration_object_detector: must implement
                                            CalibrationObjectDetector interface

        @param folder: Folder from where the images are located
        @type  folder: string

        @param image_prefix_left: Prefix the left camera's image files which
                                  are loaded must have
        @type  image_prefix_left: string

        @param image_prefix_right: Prefix the right camera's image files which
                                   are loaded must have
        @type  image_prefix_right: string
        '''
        self.calibration_object = calibration_object
        self.calibration_object_detector = calibration_object_detector
        self.folder = folder
        self.image_prefix_left = image_prefix_left
        self.image_prefix_right = image_prefix_right

    def calibrate_stereo_camera(self, alpha=0.0):
        '''
        Run stereo calibration

        Returns calibration rms error (float), image height, width
        and camera_matrix, dist_coeffs, projection_matrix and
        rectification_matrix
        for right and left camera (as np.ndarray() type) as well as baseline
        R and T

        @param alpha: Set zoom, alpha = 0.0 results in cropped image with all
        pixels valid in undistorted image, alpha = 1.0 preserves all pixels
        from original image
        @type  alpha: float (0.0 < alpha < 1.0)
        '''
        # load images
        print "--> loading left  images from folder '%s' with prefix '%s'" %  \
            (self.folder, self.image_prefix_left)
        print "--> loading right images from folder '%s' with prefix '%s'" %  \
            (self.folder, self.image_prefix_right)
        images_l = self._load_images(
            self.folder, self.image_prefix_left, grayscale=True)
        images_r = self._load_images(
            self.folder, self.image_prefix_right, grayscale=True)
        print "--> loaded %i left  images" % len(images_l)
        print "--> loaded %i right images" % len(images_r)

        # get width and height of images and check all
        (h_ref, w_ref) = images_l[0].shape[:2]
        (h_dep, w_dep) = images_r[0].shape[:2]

        # detect image and object points in all images
        print "--> detecting calibration object in all images..."
        (image_points_l, object_points_l) = self._detect_points(
            images_l, is_grayscale=True)
        (image_points_r, object_points_r) = self._detect_points(
            images_r, is_grayscale=True)

        impl = []
        obpl = []
        impr = []
        obpr = []

        for ipl, opl, ipr, opr in zip(image_points_l, object_points_l,
                                      image_points_r, object_points_r):
            if ipl is not None and opl is not None and ipr is not None and  \
                    opr is not None:

                impl.append(ipl)
                obpl.append(opl)
                impr.append(ipr)
                obpr.append(opr)
        image_points_l = impl
        object_points_l = obpl
        image_points_r = impr
        object_points_r = obpr
        # sanity checks for image and object points
        print "image_points_l = " + str(len(image_points_l)) + \
            ", image_points_r = " + str(len(image_points_r))
        print "object_points_l = " + str(len(object_points_l)) + \
            ", object_points_r = " + str(len(object_points_r))
        assert (len(image_points_l) == len(image_points_r))
        assert (len(object_points_l) == len(object_points_r))
        object_points = object_points_l

        # prepare matrices
        camera_matrix_l = np.zeros((3, 3), np.float32)
        dist_coeffs_l = np.zeros((1, 5), np.float32)
        camera_matrix_r = np.zeros((3, 3), np.float32)
        dist_coeffs_r = np.zeros((1, 5), np.float32)

        # mono flags
        mono_flags = 0
        # mono_flags |= cv2.CALIB_FIX_K3

        # run monocular calibration on each camera to get intrinsic parameters
        (rms_l, camera_matrix_l, dist_coeffs_l, _, _) = \
            cv2.calibrateCamera(object_points, image_points_l, (w_ref, h_ref),
                                camera_matrix_l, dist_coeffs_l,
                                flags=mono_flags)
        (rms_r, camera_matrix_r, dist_coeffs_r, _, _) =  \
            cv2.calibrateCamera(object_points, image_points_r, (w_dep, h_dep),
                                camera_matrix_r, dist_coeffs_r,
                                flags=mono_flags)

        # set stereo flags
        stereo_flags = 0
        stereo_flags |= cv2.CALIB_FIX_INTRINSIC

         # More availabe flags...
         # stereo_flags |= cv2.CALIB_USE_INTRINSIC_GUESS    \
                 # Refine intrinsic parameters
         # stereo_flags |= cv2.CALIB_FIX_PRINCIPAL_POINT    \
                 # Fix the principal points during the optimization.
         # stereo_flags |= cv2.CALIB_FIX_FOCAL_LENGTH       \
                 # Fix focal length
         # stereo_flags |= cv2.CALIB_FIX_ASPECT_RATIO       \
                 # fix aspect ratio
         # stereo_flags |= cv2.CALIB_SAME_FOCAL_LENGTH      \
                 # Use same focal length
         # stereo_flags |= cv2.CALIB_ZERO_TANGENT_DIST      \
                 # Set tangential distortion to zero
         # stereo_flags |= cv2.CALIB_RATIONAL_MODEL         \
                 # Use 8 param
         # rational distortion model instead of 5 param plumb bob model

        # run stereo calibration
        res = cv2.stereoCalibrate(
            object_points, image_points_l, image_points_r,
            (w_ref,
             h_ref), camera_matrix_l, dist_coeffs_l, camera_matrix_r,
            dist_coeffs_r, flags=stereo_flags)
        (rms_stereo, camera_matrix_l, dist_coeffs_l,
         camera_matrix_r, dist_coeffs_r, R, T, E, F) = res

        # run stereo rectification
        res = self._rectify(camera_matrix_l, dist_coeffs_l, camera_matrix_r,
                            dist_coeffs_r, (w_ref, h_ref), R, T, alpha)
        (rectification_matrix_l, rectification_matrix_r,
         projection_matrix_l, projection_matrix_r) = res

         # DEBUG: check types -> should all be np.ndarray
         # print type(camera_matrix_l)
         # print type(dist_coeffs_l)
         # print type(rectification_matrix_l)
         # print type(projection_matrix_l)
         # print type(camera_matrix_r)
         # print type(dist_coeffs_r)
         # print type(rectification_matrix_r)
         # print type(projection_matrix_r)
         # print type(R)
         # print type(T)

        return ((rms_l, rms_r, rms_stereo),
                camera_matrix_l, dist_coeffs_l, rectification_matrix_l, projection_matrix_l,
                camera_matrix_r, dist_coeffs_r, rectification_matrix_r, projection_matrix_r,
                ((h_ref, w_ref), (h_dep, w_dep)), R, T)

    def _rectify(self, camera_matrix_l, dist_coeffs_l, camera_matrix_r,
                 dist_coeffs_r, xxx_todo_changeme, R, T, alpha):
        '''
        Helper function, wraps cv2.cv.StereoRectify with cv2 input/output types
        as this function is missing in new opencv python cv2 interface

        see https://code.ros.org/trac/opencv/changeset/6843
        '''
        (h, w) = xxx_todo_changeme
        cameraMatrix1 = cv2util.np2cvmat(camera_matrix_l)
        cameraMatrix2 = cv2util.np2cvmat(camera_matrix_r)
        distCoeffs1 = cv2util.np2cvmat(dist_coeffs_l)
        distCoeffs2 = cv2util.np2cvmat(dist_coeffs_r)
        R = cv2util.np2cvmat(R)
        T = cv2util.np2cvmat(T.T)
        # initialize result cvmats
        R1 = cv2.cv.CreateMat(3, 3, cv2.CV_64FC1)
        R2 = cv2.cv.CreateMat(3, 3, cv2.CV_64FC1)
        P1 = cv2.cv.CreateMat(3, 4, cv2.CV_64FC1)
        P2 = cv2.cv.CreateMat(3, 4, cv2.CV_64FC1)

        # do rectification
        cv2.cv.StereoRectify(cameraMatrix1, cameraMatrix2, distCoeffs1,
                             distCoeffs2, (h, w), R, T, R1, R2, P1, P2,
                             alpha=alpha)

        # convert results back to np data types
        R1 = cv2util.cvmat2np(R1).reshape((3, 3))
        R2 = cv2util.cvmat2np(R2).reshape((3, 3))
        P1 = cv2util.cvmat2np(P1).reshape((3, 4))
        P2 = cv2util.cvmat2np(P2).reshape((3, 4))
        return (R1, R2, P1, P2)
