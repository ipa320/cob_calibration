# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# author: Vijay Pradeep

# Define a camera sensor attached to a chain
#
#       before_chain_Ts -- camera_chain -- after_chain_Ts -- camera
#      /
#   root
#      \
#       checkerboard


from numpy import matrix, reshape, array, zeros, real, float64, asarray, diag, ones

import roslib
roslib.load_manifest('cob_robot_calibration_est')
import rospy
from cob_robot_calibration_est.full_chain import FullChainRobotParams
from cob_robot_calibration_est.sensors.chain_sensor import ChainBundler, ChainSensor
#from cob_robot_calibration_est.ChainMessage import ChainMessage
import yaml
import cv
#import code
from copy import deepcopy


class CameraChainBundler:
    """
    Tool used to generate a list of CameraChain sensors from a single calibration sample message
    """
    def __init__(self, configs):
        """
        Inputs:
        - valid_configs: A list of dictionaries, where each list elem stores the configuration of a potential
                         camera chain that we might encounter.
          Example (in yaml format):
            - camera_id: forearm_right_rect
              sensor_id: forearm_right_rect
              chain:
                before_chain: [r_shoulder_pan_joint]
                chain_id:     right_arm_chain
                after_chain:  [r_forearm_roll_adj, r_forearm_cam_frame_joint,
                               r_forearm_cam_optical_frame_joint]
                dh_link_num:  4
        """
        self._configs = configs
        self._valid_configs = configs['camera_chains']

    # Construct a CameraChainSensor for every camera chain sensor that exists in the given robot measurement
    def build_blocks(self, M_robot):
        """
        Input:
        - M_robot: A calibration sample of type calibration_msgs/RobotMeasurement
        Returns:
        - sensors: A list of CameraChainSensor objects that corresponds to all camera chain sensors found
                   in the calibration message that was passed in.
        """
        sensors = []
        for cur_config in self._valid_configs:
            if cur_config["camera_id"] in [x.camera_id for x in M_robot.M_cam]:
                if all([chain in [x.header.frame_id for x in M_robot.M_chain] for chain in cur_config["chain"]["chains"]]):
                    M_cam = M_robot.M_cam[[x.camera_id for x in M_robot.M_cam]
                                          .index(cur_config["camera_id"])]
                    M_chain = [c for c in M_robot.M_chain if c.header.frame_id in cur_config["chain"]["chains"]]
                else:
                    print "else cur_config[chain][chain_id]: ", cur_config["chain"]["chain_id"]
                    break
                cur_sensor = CameraChainSensor(
                    cur_config, M_cam, M_chain, self._configs)
                sensors.append(cur_sensor)
            else:
                rospy.logdebug("  Didn't find block")
        return sensors


class CameraChainSensor():
    def __init__(self, config_dict, M_cam, M_chain, config):
        """
        Generates a single sensor block for a single configuration
        Inputs:
        - config_dict: The configuration for this specific sensor. This looks exactly like
                       a single element from the valid_configs list passed into CameraChainBundler.__init__
        - M_cam: The camera measurement of type calibration_msgs/CameraMeasurement
        - M_chain: The chain measurement of type calibration_msgs/ChainMeasurement
        """

        self.sensor_type = "camera"
        self.sensor_id = config_dict["camera_id"]

        self._config_dict = config_dict
        self._config = config
        self._M_cam = M_cam
        self._M_chain = M_chain

        self._chain = FullChainRobotParams(config_dict["chain"], self._config)
        self.camera_info_name = None

        self.info_used = self.sensor_id in ['left', 'right', 'kinect_rgb']
        path = rospy.get_param(
            '/calibration_config/camera_parameter') + self.sensor_id + '.yaml'
        with open(path) as f:
            self._yaml = yaml.load(f)
        self._distortion = self._yaml['distortion_coefficients']['data']
        self._camera_matrix = self._yaml['projection_matrix']['data']

        self.terms_per_sample = 2

    def update_config(self, robot_params):
        """
        On each optimization cycle the set of system parameters that we're optimizing over will change. Thus,
        after each change in parameters we must call update_config to ensure that our calculated residual reflects
        the newest set of system parameters.
        """
        self._camera = robot_params.rectified_cams[
            self._config_dict["camera_id"]]

        if self._chain is not None:
            self._chain.update_config(robot_params)

    def compute_residual(self, target_pts):
        """
        Computes the measurement residual for the current set of system parameters and target points
        Input:
        - target_pts: 4XN matrix, storing features point locations in world cartesian homogenous coordinates.
        Output:
        - r: 2N long vector, storing pixel residuals for the target points in the form [u1, v1, u2, v2, ..., uN, vN]
        """
        z_mat = self.get_measurement()[:, 0, :]
        h_mat = self.compute_expected(target_pts)
        #code.interact(local=locals())
        assert(z_mat.shape[1] == 2)
        assert(h_mat.shape[1] == 2)
        assert(z_mat.shape[0] == z_mat.shape[0])
        r = array(reshape(h_mat - z_mat, [-1, 1]))[:, 0]
        return r

    def compute_residual_scaled(self, target_pts):
        """
        Computes the residual, and then scales it by sqrt(Gamma), where Gamma
        is the information matrix for this measurement (Cov^-1).
        """
        r = self.compute_residual(target_pts)
        gamma_sqrt = self.compute_marginal_gamma_sqrt(target_pts)
        r_scaled = gamma_sqrt * matrix(r).T
        return array(r_scaled.T)[0]

    def compute_marginal_gamma_sqrt(self, target_pts):
        """
        Calculates the square root of the information matrix for the measurement of the
        current set of system parameters at the passed in set of target points.
        """
        import scipy.linalg
        cov = self.compute_cov(target_pts)
        gamma = matrix(zeros(cov.shape))
        num_pts = self.get_residual_length()/2

        for k in range(num_pts):
            #print "k=%u" % k
            first = 2*k
            last = 2*k+2
            sub_cov = matrix(cov[first:last, first:last])
            sub_gamma_sqrt_full = matrix(scipy.linalg.sqrtm(sub_cov.I))
            sub_gamma_sqrt = real(sub_gamma_sqrt_full)
            assert(scipy.linalg.norm(sub_gamma_sqrt_full - sub_gamma_sqrt) < 1e-6)
            gamma[first:last, first:last] = sub_gamma_sqrt
        return gamma

    def get_residual_length(self):
        N = len(self._M_cam.image_points)
        return N * 2

    # Get the observed measurement in a Nx2 Matrix
    def get_measurement(self):
        """
        Get the target's pixel coordinates as measured by the actual sensor
        """
        cm = cv.CreateMat(3, 3, cv.CV_32F)
        cv.Set(cm, 0)
        cm_t = reshape(matrix(self._camera_matrix, float64), (3, 4))[:3, :3]
        for x in range(3):
            for y in range(3):
                cm[x, y] = cm_t[x, y]
        dc = cv.CreateMat(5, 1, cv.CV_32F)
        for i in range(len(self._distortion)):
            dc[i, 0] = self._distortion[i]

        camera_pix = cv.fromarray(float64(
            [[[pt.x, pt.y]] for pt in self._M_cam.image_points]))
        dst = cv.CreateMat(camera_pix.rows, camera_pix.cols, camera_pix.type)
        cv.UndistortPoints(camera_pix, dst, cm, dc, P=cm)

        return asarray(dst)

    def compute_expected(self, target_pts):
        """
        Compute the expected pixel coordinates for a set of target points.
        target_pts: 4xN matrix, storing feature points of the target, in homogeneous coords
        Returns: target points projected into pixel coordinates, in a Nx2 matrix
        """
        if self._M_chain is not None:
            return self._compute_expected(target_pts)
        else:
            return self._compute_expected(target_pts, None)

    def _compute_expected(self, target_pts, M_chain=None):
        """
        Compute what measurement we would expect to see, based on the current system parameters
        and the specified target point locations.

        Inputs:
        - chain_state: The joint angles of this sensor's chain of type sensor_msgs/JointState.
        - target_pts: 4XN matrix, storing features point locations in world cartesian homogenous coordinates.

        Returns: target points projected into pixel coordinates, in a Nx2 matrix
        """
        M_chain = self._M_chain if M_chain is None else M_chain
        # Camera pose in root frame
        camera_pose_root = self._chain.calc_block.fk(M_chain)
        cam_frame_pts = camera_pose_root.I * target_pts
        # Do the camera projection
        pixel_pts = self._camera.project(self._camera_matrix, cam_frame_pts)

        #code.interact(local=locals())
        return pixel_pts.T

    def compute_expected_J(self, target_pts):
        """
        The output Jacobian J shows how moving target_pts in cartesian space affects
        the expected measurement in (u,v) camera coordinates.
        For n points in target_pts, J is a 2nx3n matrix
        Note: This doesn't seem to be used anywhere, except maybe in some drawing code
        """
        epsilon = 1e-8
        N = len(self._M_cam.image_points)
        Jt = zeros([N * 3, N * 2])
        for k in range(N):
            # Compute jacobian for point k
            sub_Jt = zeros([3, 2])
            x = target_pts[:, k].copy()
            f0 = self.compute_expected(x)
            for i in [0, 1, 2]:
                x[i, 0] += epsilon
                fTest = self.compute_expected(x)
                sub_Jt[i, :] = array((fTest - f0) / epsilon)
                x[i, 0] -= epsilon
            Jt[k * 3:(k + 1) * 3, k * 2:(k + 1) * 2] = sub_Jt
        return Jt.T

    def compute_cov(self, target_pts):
        '''
        Computes the measurement covariance in pixel coordinates for the given
        set of target points (target_pts)
        Input:
         - target_pts: 4xN matrix, storing N feature points of the target, in homogeneous coords
        '''
        #chain cov
        chain_cov = self.compute_chain_cov(target_pts)
        cam_cov = matrix(
            zeros([self.get_residual_length(), self.get_residual_length()]))

        # Convert StdDev into variance
        var_u = self._camera._cov_dict['u'] * self._camera._cov_dict['u']
        var_v = self._camera._cov_dict['v'] * self._camera._cov_dict['v']
        for k in range(cam_cov.shape[0] / 2):
            cam_cov[2 * k, 2 * k] = var_u
            cam_cov[2 * k + 1, 2 * k + 1] = var_v

        # Both chain and camera covariances are now in measurement space, so we can simply add them together
        if self._M_chain is not None:
            cov = chain_cov + cam_cov
        else:
            cov = cam_cov
        return cov
    def compute_chain_cov(self, target_pts):

        _ones = ones([7, self.get_residual_length()])
        cov = matrix(diag([0.01] * self.get_residual_length()))

        epsilon = 1e-8



        chain_cov = None
        if self._M_chain is not None:
            i_joints = []
            num_joints = 0
            for i,c in enumerate(self._M_chain):
                l = len(c.actual.positions)
                i_joints.extend([i] * l)
                num_joints+=l

            #num_joints = sum(num_joints)
            #num_joints = len(self._M_chain.actual.position)
            Jt = zeros([num_joints, self.get_residual_length()])

            x = deepcopy(self._M_chain)
            # Compute the Jacobian from the chain's joint angles to pixel residuals
            f0 = reshape(array(self._compute_expected(target_pts, x)), [-1])
            for i in range(num_joints):
                x[i_joints[i]].header.frame_id = self._M_chain[i_joints[i]].header.frame_id
                x[i_joints[i]].actual.positions = list(self._M_chain[i_joints[i]].actual.positions[:])
                x[i_joints[i]].actual.positions[i] += epsilon
                fTest = reshape(array(self._compute_expected(target_pts, x)), [-1])
                Jt[i] = (fTest - f0)/epsilon
            cov_angles = []

            for c in self._chain.calc_block._chains:
                cov_angles.extend([ x * x for x in c._chain._cov_dict])

            # Transform the chain's covariance from joint angle space into pixel space using the just calculated jacobian
            chain_cov = matrix(Jt).T * matrix(diag(cov_angles)) * matrix(Jt)
        return chain_cov
    def build_sparsity_dict(self):
        """
        Build a dictionary that defines which parameters will in fact affect this measurement.

        Returned dictionary should be of the following form:
          transforms:
            my_transformA: [1, 1, 1, 1, 1, 1]
            my_transformB: [1, 1, 1, 1, 1, 1]
          dh_chains:
            my_chain:
              dh:
                - [1, 1, 1, 1]
                - [1, 1, 1, 1]
                       |
                - [1, 1, 1, 1]
              gearing: [1, 1, ---, 1]
          rectified_cams:
            my_cam:
              baseline_shift: 1
              f_shift: 1
              cx_shift: 1
              cy_shift: 1
        """
        sparsity = dict()
        sparsity['transforms'] = {}
        chain_transform_names = [chain['before_chain'] + chain['after_chain'] for chain in self._config['chains']
                                 if chain['chain_id'] in self._config_dict['chain']['chains']][0]
        for cur_transform_name in (self._config_dict['chain']['before_chain'] + self._config_dict['chain']['after_chain'] + chain_transform_names):
            sparsity['transforms'][cur_transform_name] = [1, 1, 1, 1, 1, 1]

        sparsity['dh_chains'] = {}
        chain_ids = self._config_dict["chain"]['chains']
        for chain in self._chain.calc_block._chains:
            chain_id = chain._config_dict["chain_id"]
            num_links = chain._chain._M
            #num_links = self._full_chain.calc_block._chains._M
            #assert(num_links == len(self._M_chain.chain_state.position))
            sparsity['dh_chains'][chain_id] = {}
            sparsity['dh_chains'][chain_id]['dh'] = [[1, 1, 1, 1,1,1]] * num_links
            sparsity['dh_chains'][chain_id]['gearing'] = [1] * num_links

        sparsity['rectified_cams'] = {}
        sparsity['rectified_cams'][self.sensor_id] = dict(
            [(x, 1) for x in self._camera.get_param_names()])

        return sparsity
