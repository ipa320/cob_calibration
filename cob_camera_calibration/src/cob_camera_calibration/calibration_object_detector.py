#!/usr/bin/env python
import cv2

DEBUG_OUTPUT = True

class CalibrationObjectDetector:
    '''
    Base Class for calibration object detector.
    
    A specific calibration calibration detector implementation can detect
    the specific calibration object and calculate its 3D pose.
    '''
    
    def __init__(self, calibration_object):
        '''
        @param calibration_object: Calibration object to work with. Must be compatible to CalibrationObjectDetector
        @type  calibration_object: CalibrationObject
        '''
        self.calibration_object = calibration_object
        
    def detect_image_points(self, image, is_grayscale):
        raise NotImplementedError()
    
    def calculate_object_pose(self, image_raw, camera_matrix, dist_coeffs, is_grayscale):
        '''
        Calculate 3D pose of calibration object in image given the camera's
        camera matrix and distortion coefficients. 
        
        Returns rotation matrix and translation vector.
        
        @param image_raw: input image, not distortion corrected
        @type  image_raw: cv2 compatible numpy image
        
        @param camera_matrix: camera matrix of camera
        @type  camera_matrix: numpy matrix
        
        @param dist_coeffs: distortion coefficients of camera
        @type  dist_coeffs: numpy matrix
        
        @param is_grayscale: set to true if image is grayscale
        @type  is_grayscale: bool
        '''
        # get image and object points
        image_points = self.detect_image_points(image_raw, is_grayscale)
        object_points = self.calibration_object.get_pattern_points()
        
        # get object pose in raw image (not yet distortion corrected)
        (rvec, tvec) = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
        
        # convert rvec to rotation matrix
        rmat = cv2.Rodrigues(rvec)[0]
        return (rmat, tvec)

class CheckerboardDetector(CalibrationObjectDetector):
    '''
    Detects a checkerboard calibration object
    '''
    def detect_image_points(self, image, is_grayscale):
        '''
        Detect the pixels at which the checkerboards corners are. Returns
        list of (x,y) coordinates.
        
        @param image_raw: input image with checkerboard
        @type  image_raw: cv2 compatible numpy image
        
        @param is_grayscale: set to true if image is grayscale
        @type  is_grayscale: bool
        '''
        # detect checkerboard
        found, corners = cv2.findChessboardCorners(image, self.calibration_object.pattern_size)
        if found:
            # create gray image if image is not grayscale
            if not is_grayscale:
                gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray_image = image
            
            # refine checkerboard corners to subpixel accuracy
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            cv2.cornerSubPix(gray_image, corners, (5, 5), (-1, -1), term)
        else:
            # could not find checkerboard
            if DEBUG_OUTPUT:
                print 'checkerboard not found'
            return None
        return corners
