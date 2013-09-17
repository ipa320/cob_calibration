#!/usr/bin/env python
import rospy
from urdf_parser_py.urdf import URDF
import numpy as np


class TorsoIK():
    """TorsoIK estimates an ik solution for the Torso"""
    def __init__(self, parameter_name='/torso_controller/joints'):
        self.parameter_name = parameter_name
        self.joint_names = rospy.get_param(self.parameter_name)
        self.get_torso_configuration()
        self.get_torso_limits()
        self.compute_maximum_angles()
        self.camera_viewfield = 0

    def set_camera_viewfield(self, vf):
        self.camera_viewfield = vf

    def get_torso_configuration(self):
        self.configuration = []
        self.n_joints = {'p': 0, 't': 0}
        for jn in self.joint_names:
            try:
                assert any(['pan' in jn, 'tilt' in jn])
            except AssertionError:
                print 'ERROR joint configuration received unknown names'
                return
            if 'pan' in jn:
                self.configuration.append('p')
                self.n_joints['p'] += 1
            elif 'tilt' in jn:
                self.configuration.append('t')
                self.n_joints['t'] += 1

    def get_torso_limits(self):
        robot = URDF.load_from_parameter_server()
        self.limits = []
        for jn in self.joint_names:
            self.limits.append(
                (robot.joints[jn].limits.lower, robot.joints[jn].limits.upper))

        print self.limits

    def compute_maximum_angles(self):
        """docstring for compute_maximum_angles"""
        joint_info = zip(self.configuration, self.limits)
        self.max_angles = {'p': 0, 't': 0}

        for info in joint_info:
            self.max_angles[info[0]] += info[1][1]

    #def calculate_ik(self, target_angles):
        #return self.limit_ik(self._calculate_ik_unlimited(target_angles))

    def valid_ik(self, ik):
        for index, angle in enumerate(ik):
            if self.limits[index][0] > ik[index] or \
               self.limits[index][1] < ik[index]:
                return False
        return True

    def _calculate_ik_unlimited(self, target_angles):
        return [target_angles[x] / self.n_joints[x] for x in self.configuration]

    def in_range(self, angles):
        for k, v in angles.iteritems():
            if np.abs(v) - self.max_angles[k] > self.camera_viewfield:
                return False
        return True

    def sgn(self, x):
        return np.abs(x) / x
