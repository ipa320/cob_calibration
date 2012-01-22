#!/usr/bin/env python
import yaml
import numpy as np

class CalibrationData:
    '''
    Calibration data class stores intrinsic camera parameters 
    and some additional parameters (name and frame id)
    
    Provides methods to save and read parameters from yaml file.
    '''
    
    def __init__(self, camera_name, frame_id, image_width, image_height):
        '''
        Initialize object
        '''
        self.camera_name    = camera_name
        self.frame_id       = frame_id
        self.image_width    = image_width
        self.image_height   = image_height
        
        # initialize matrices
        self.camera_matrix              = np.identity(3)
        self.rectification_matrix       = np.identity(3)
        self.projection_matrix          = np.hstack((np.identity(3), np.zeros((3,1))))
        self.distortion_model           = "plumb_bob"
        self.distortion_coefficients    = np.zeros((1,5))
 
#        # DEBUG       
#        print self.camera_matrix              
#        print self.rectification_matrix       
#        print self.projection_matrix          
#        print self.distortion_model           
#        print self.distortion_coefficients    

    
    def read_camera_yaml_file(self, filename):
        '''
        Load camera calibration info from the yaml file located at 'filename'.
        '''
        calib = yaml.load(file(filename))
        
        # checks
        assert calib['camera_matrix']['rows'] == 3
        assert calib['camera_matrix']['cols'] == 3
        assert calib['rectification_matrix']['rows'] == 3
        assert calib['rectification_matrix']['cols'] == 3
        assert calib['projection_matrix']['rows'] == 3
        assert calib['projection_matrix']['cols'] == 4
        assert calib['distortion_model'] == 'plumb_bob'
        assert calib['distortion_coefficients']['rows'] == 1
        assert calib['distortion_coefficients']['cols'] == 5
        
        # import
        self.camera_name             = calib['camera_name']
        self.frame_id                = calib['frame_id']
        self.camera_matrix           = np.matrix(calib['camera_matrix']['data']).reshape((3,3))
        self.rectification_matrix    = np.matrix(calib['rectification_matrix']['data']).reshape((3,3))
        self.projection_matrix       = np.matrix(calib['projection_matrix']['data']).reshape((3,4))
        self.distortion_model        = calib['distortion_model']
        self.distortion_coefficients = np.matrix(calib['distortion_coefficients']['data']).reshape((1,5))
        self.image_width             = calib['image_width']
        self.image_height            = calib['image_height']        
        pass
    
    def save_camera_yaml_file(self, filename):
        '''
        Save camera calibration info to the yaml file located at 'filename'.
        '''
        open(filename, "w").writelines(self._as_yaml_string())
    
    def _as_yaml_string(self):
        '''
        Convert calibration data to yaml string.
        '''
        return yaml_template % (self.camera_name, 
                                self.frame_id, 
                                self.image_width, 
                                self.image_height, 
                                np.array(self.camera_matrix).flatten().tolist(), 
                                np.array(self.distortion_coefficients).flatten().tolist(), 
                                np.array(self.rectification_matrix).flatten().tolist(), 
                                np.array(self.projection_matrix).flatten().tolist())

yaml_template='''camera_name: %s
frame_id: %s
image_width: %d
image_height: %d
camera_matrix:
  rows: 3
  cols: 3
  data: %s
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: %s
rectification_matrix:
  rows: 3
  cols: 3
  data: %s
projection_matrix:
  rows: 3
  cols: 4
  data: %s
'''
