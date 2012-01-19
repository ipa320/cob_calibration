#!/usr/bin/env python
PKG  = 'cob_robot_calibration'
NODE = 'update_cob_calibration_urdf'
import roslib; roslib.load_manifest(PKG)

import rospy
import tf

import sys
import yaml
from math import pi
from xml.dom import minidom, Node

from cob_robot_calibration_est import single_transform
from cob_robot_calibration import calibration_urdf_updater

 # Default values for files and debug output
DEFAULT_CALIB_URDF_XACRO_IN  = "/tmp/cal/calibration.urdf.xacro"
DEFAULT_CALIB_URDF_XACRO_OUT = "/tmp/cal/calibration.urdf.xacro_updated" 
DEFAULT_YAML_CALIB_SYSTEM    = "/tmp/cal/v0/result_step_3.yaml"
DEFAULT_YAML_INITIAL_SYSTEM  = "/home/fmw-sh/git/cob_calibration/cob_robot_calibration/config/params_v0/system.yaml"
ENABLE_DEBUG_OUTPUT = False

class UpdateCobCalibrationUrdf():
    '''
    @summary: Updates calibration urdf file with calibration results
    
    The results of the cob calibration process are read from the calibrated_system yaml
    file and propagated to the urdf robot calibration file.
    '''
    
    def __init__(self):
        '''
        Get file locations from parameter server (or use defaults) and setup dictionary
        which specifies which values are updated. 
        '''
        rospy.init_node(NODE)
        print "==> started " + NODE
        
        # get file names from parameter server
        self.file_urdf_in =             rospy.get_param('~urdf_in',        DEFAULT_CALIB_URDF_XACRO_IN)
        self.file_urdf_out =            rospy.get_param('~urdf_out',       DEFAULT_CALIB_URDF_XACRO_OUT)
        self.file_yaml_calib_system =   rospy.get_param('~calib_system',   DEFAULT_YAML_CALIB_SYSTEM)
        self.file_yaml_init_system =    rospy.get_param('~initial_system', DEFAULT_YAML_INITIAL_SYSTEM)
        self.debug =                    rospy.get_param('~debug',          ENABLE_DEBUG_OUTPUT)
        
        # tfs2update stores the transform names [which need to be converted to (x, y, z, roll, pitch, yaw) 
        # and updated in the urdf] and their corresponding property name prefixes as used in calibration.urdf.xarco
        self.tfs2update = {'arm_0_joint':                        'arm_', 
                           'torso_0_joint':                      'torso_',  
                           'head_color_camera_l_joint':          'cam_l_', 
                           #'head_color_camera_r_joint':          'cam_r_', 
                           'head_cam3d_rgb_optical_frame_joint': 'cam3d_'}
        
        # chains2process stores the dh chain names of chains which need to be updated and their corresponding
        # property names/segments as used in calibration.urdf.xarco
        self.chains2update = {'arm_chain':   ['arm_1_ref',
                                              'arm_2_ref',
                                              'arm_3_ref',
                                              'arm_4_ref',
                                              'arm_5_ref',
                                              'arm_6_ref',
                                              'arm_7_ref'], 
                              'torso_chain': ['torso_lower_neck_tilt_cal_offset',
                                              'torso_pan_cal_offset',
                                              'torso_upper_neck_tilt_cal_offset']}
        
    def run(self):
        '''
        Start the update process. Values are read from yaml files and are preprocessed for 
        writing to xml
        '''
        # load yaml files
        print "--> loading calibrated system from '%s'" % self.file_yaml_calib_system
        calib_system   = yaml.load(file(self.file_yaml_calib_system))
        print "--> loading initial system from '%s'" % self.file_yaml_init_system
        initial_system = yaml.load(file(self.file_yaml_init_system))
    
        attributes2update = {}
        
        # process transforms
        for tf_name in self.tfs2update.keys():
            prefix = self.tfs2update[tf_name]
            (x, y, z, roll, pitch, yaw) = self._convert_transform(calib_system['transforms'][tf_name])
    
            # add to attributes2update dict as "attribute name -> new_value" entries
            attributes2update[prefix+"x"]       = x
            attributes2update[prefix+"y"]       = y
            attributes2update[prefix+"z"]       = z
            attributes2update[prefix+"roll"]    = roll
            attributes2update[prefix+"pitch"]   = pitch
            attributes2update[prefix+"yaw"]     = yaw
            
        # process dh chains
        for chain in self.chains2update:
            segments = self.chains2update[chain]
            for id in range(len(segments)):
                # process segment with id
                segment = segments[id]
                
                # process segment of chain
                initial_value = initial_system["dh_chains"][chain]["dh"][id][0] # get initial theta value of segment for current chain
                calib_value   =   calib_system["dh_chains"][chain]["dh"][id][0] # get calibr. theta value of segment for current chain
                new_value = float(calib_value) - float(eval(str(initial_value)))
                new_value = round(new_value, 10)
                
                # add to attributes2update dict as "attribute name -> new_value" entries
                attributes2update[segment] = new_value
                
        # update calibration xml based on attributes2update dict
        urdf_updater = calibration_urdf_updater.CalibrationUrdfUpdater(self.file_urdf_in, self.file_urdf_out, self.debug)
        urdf_updater.update(attributes2update)
        
    def _convert_transform(self, t):
        ''' 
        Convert transform notation from (x, y, z, rotation_vector) as used in calibrated_system 
        and initial_system to (x, y, z, roll, pitch, yaw) notation used in calibration.urdf.xacro
        
        @param t: (x, y, z, rotation_vector)
        @type  t: tuple
        
        @return: (x, y, z, roll, pitch, yaw)
        @rtype:  tuple
        '''
        matrix = single_transform.SingleTransform(t).transform # convert to 4x4 transformation matrix 
        roll, pitch, yaw = tf.transformations.euler_from_matrix(matrix)
        x, y, z = matrix[0,3], matrix[1,3], matrix[2,3]
        return (x, y, z, roll, pitch, yaw)

if __name__ == "__main__":
    # start main
    updateUrdf = UpdateCobCalibrationUrdf()
    updateUrdf.run()
    
    # shutdown
    rospy.signal_shutdown(rospy.Time.now())
    print "==> done! exiting..."
