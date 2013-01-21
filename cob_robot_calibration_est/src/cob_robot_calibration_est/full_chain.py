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

# A convenience object that wraps a dh_chain, along with transforms
# before and after the chain.
#
# The transforms can be thought of as follows:
#
#       before_chain_T[0] ... before_chain_T[N] -- chain -- after_chain_T[0] ... after_chain_T[N]
#      /
#   root

#from sensor_msgs.msg import JointState
from numpy import matrix
import numpy
import tf.transformations


class FullChainRobotParams:

    # Initialize with dictionary of the current configuration.
    # Dictionary should have the following:
    #   - before_chain: List of transform ids to apply before the chain
    #   - chain_id: Chain ID for the dh_chain
    #   - dh_link_num: Specifies which dh elem should be the last one to apply.
    #   - after_chain: List of transform ids to apply after the chain
    def __init__(self, config_dict, configuration):
        self._config_dict = config_dict
        self.calc_block = FullChainCalcBlock()
        self._full_config = configuration
        self.build_chains(self._config_dict["chains"])

    def build_chains(self, chain_ids):
        self.chains = [None] * len(chain_ids)
        for chain in self._full_config["chains"]:
            if chain["chain_id"] in chain_ids:
                idx = chain_ids.index(chain["chain_id"])
                self.chains[idx] = SingleChainCalc(chain)

    def update_config(self, robot_params):
        # for transform_name in self._config_dict["before_chain"]:
        #     print "transform_name: ", transform_name
        # for transform_name2 in robot_params.transforms:
        #     print "transform_name2: ", transform_name2
        for c in self.chains:
            c.update_config(robot_params)

        before_chain_Ts = [robot_params.transforms[transform_name]
                           for transform_name in self._config_dict["before_chain"]]
        after_chain_Ts = [robot_params.transforms[transform_name]
                          for transform_name in self._config_dict["after_chain"]]

        self.calc_block.update_config(
            self.chains, before_chain_Ts, after_chain_Ts)


class SingleChainCalc:
    def __init__(self, config_dict):
        self._config_dict = config_dict

    def update_config(self, robot_params):
        self._before_chain_Ts = [robot_params.transforms[transform_name]
                                 for transform_name in self._config_dict["before_chain"]]
        self._after_chain_Ts = [robot_params.transforms[transform_name]
                                for transform_name in self._config_dict["after_chain"]]

    def fk(self, transformation):
        pose = matrix(numpy.eye(4))
        trans = transformation.translation
        rot = transformation.rotation

        # Apply the 'before chain' transforms
        for before_chain_T in self._before_chain_Ts:
            pose = pose * before_chain_T.transform

        # Apply the Chain

        euler = tf.transformations.euler_from_quaternion(rot)
        mat = tf.transformations.compose_matrix(
            translate=trans, angles=euler)

        pose = pose * mat

        # Apply the 'after chain' transforms
        for after_chain_T in self._after_chain_Ts:
            pose = pose * after_chain_T.transform

        return pose

    def __getitem__(self, key):
        return self._config_dict[key]


class FullChainCalcBlock:
    def update_config(self, chains, before_chain_Ts, after_chain_Ts):
        self._before_chain_Ts = before_chain_Ts
        self._after_chain_Ts = after_chain_Ts
        self._chains = chains
        self._chain_ids = [c['chain_id'] for c in self._chains]

        #print "before: ", self._before_chain_Ts
        #print "chains: ", self._chain_ids
        #print "after: ", self._after_chain_Ts

    def fk(self, m_chain):
        pose = matrix(numpy.eye(4))

        # Apply the 'before chain' transforms
        for before_chain_T in self._before_chain_Ts:
            pose = pose * before_chain_T.transform
        # Apply the Chain

        for chain in self._chains:
            for transformation in m_chain:
                if transformation.chain_id == chain._config_dict["chain_id"]:
                    t = transformation
            mat = chain.fk(t)
            pose = pose * mat

        # Apply the 'after chain' transforms
        for after_chain_T in self._after_chain_Ts:
            pose = pose * after_chain_T.transform

        return pose
