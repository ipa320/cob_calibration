#!/usr/bin/env python
#PKG  = 'cob_robot_calibration'
#NODE = ''
#import roslib; roslib.load_manifest(PKG)

import yaml

class CameraYamlUpdater():
    '''
    @summary: Parses a camera calibration yaml file and provides a method to update it
    '''
    
    def __init__(self, yaml_in, yaml_out, debug=False):
        '''
        Init object with paths to input and output yaml
        '''
        self.file_yaml_in =  yaml_in
        self.file_yaml_out = yaml_out
        self.debug = debug
    
    def update_baseline(self, baseline_offset_x):
        '''
        Opens yaml file (self.file_yaml_in) and adjusts stereo baseline in x direction 
        by baseline_offset and saves result to output yaml file (self.file_yaml_out)
        
        @param baseline_offset: baseline offset
        @type  baseline_offset: float
        '''
        # open yaml
        print "--> loading camera yaml file from '%s'" % self.file_yaml_in
        camera_calibration = yaml.load(file(self.file_yaml_in))
        
        # adjust baseline
        camera_calibration['projection_matrix']['data'][3] += baseline_offset_x
        
        # save yaml
        print "--> saving results to camera yaml file '%s'" % self.file_yaml_out
        yaml.dump(camera_calibration, open(self.file_yaml_out, "w")) 
