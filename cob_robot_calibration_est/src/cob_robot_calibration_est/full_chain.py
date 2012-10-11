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

from sensor_msgs.msg import JointState
from kinematics_msgs.srv import *
from numpy import matrix
import numpy
import rospy
import tf.transformations


class FullChainRobotParams:

    # Initialize with dictionary of the current configuration.
    # Dictionary should have the following:
    #   - before_chain: List of transform ids to apply before the chain
    #   - chain_id: Chain ID for the dh_chain
    #   - dh_link_num: Specifies which dh elem should be the last one to apply.
    #   - after_chain: List of transform ids to apply after the chain
    def __init__(self, config_dict):
        self._config_dict = config_dict
        self.calc_block = FullChainCalcBlock()

    def update_config(self, robot_params):
        # for transform_name in self._config_dict["before_chain"]:
        #     print "transform_name: ", transform_name
        # for transform_name2 in robot_params.transforms:
        #     print "transform_name2: ", transform_name2
        before_chain_Ts = [robot_params.transforms[transform_name] for transform_name in self._config_dict["before_chain"]]
#        if self._config_dict["chain_id"] == 'NULL':
        chain = None
        dh_link_num = None
        if self._config_dict["chain_id"] != None:

            chain           = robot_params.dh_chains[ self._config_dict["chain_id"] ]
	    try:
	      	dh_link_num     = self._config_dict["dh_link_num"]
	    except:
		pass
        after_chain_Ts  = [robot_params.transforms[transform_name] for transform_name in self._config_dict["after_chain"]]
        first_link=self._config_dict['before_chain'][-1]
	last_link=None
        dh_chain=None
	try:
            last_link=self._config_dict['last_link']
            dh_chain=self._config_dict['dh_chain']
	except:
	    pass

        self.calc_block.update_config(before_chain_Ts, chain, dh_link_num, after_chain_Ts, first_link,last_link,dh_chain,chain_id=self._config_dict["chain_id"])

class FullChainCalcBlock:
    def update_config(self, before_chain_Ts, chain, dh_link_num, after_chain_Ts,first_link,last_link,dh_chain,chain_id):
	if dh_link_num is None and dh_chain is None:
	    rospy.logerror("forward kinematic can not be calculated. Either DH Parameter in system.yaml or get_fk service missing")
	    return 
        self._before_chain_Ts = before_chain_Ts
        self._chain = chain
        self._after_chain_Ts = after_chain_Ts        
        self._chain_id=chain_id
	
        if dh_link_num is None:
	    self._group=self._chain_id.split('_')[0]
	    fk_s_name=rospy.get_param('fk_service','/cob_%s_kinematics/get_fk')
            self._fks = rospy.ServiceProxy((fk_s_name%self._group), GetPositionFK)
	    self._dh_link_names=dh_chain
	    self._last_link=last_link
            self._first_link=first_link
	else:
	    self._dh_link_num = dh_link_num

    def fk(self, chain_state):
        pose = matrix(numpy.eye(4))

        # Apply the 'before chain' transforms
        for before_chain_T in self._before_chain_Ts:
            pose = pose * before_chain_T.transform

        # Apply the DH Chain
	
        if self._chain is not None:
            if self._dh_link_names is not None:
                #print chain_state

                req = GetPositionFKRequest()
                req.header.stamp = rospy.Time.now()
                req.header.frame_id= self._first_link
                req.fk_link_names = [self._last_link]
                req.robot_state.joint_state.name = self._dh_link_names
                req.robot_state.joint_state.position = chain_state.position
                #print '*'*20,' Request ','*'*20
                #print req
                    
                res = self._fks(req)
               # print '*'*20,' Response ','*'*20
               # print res
                if  res.error_code.val!=1:
                    return
                #print res.pose_stamped[0].pose
                '''
                convert Pose to 4x4 matrix
                '''
                pose_fk=res.pose_stamped[0].pose
                quat = [pose_fk.orientation.x, pose_fk.orientation.y, pose_fk.orientation.z, pose_fk.orientation.w] 
                pos = matrix([pose_fk.position.x, pose_fk.position.y, pose_fk.position.z]).T 
                mat = matrix(tf.transformations.quaternion_matrix(quat)) 
                mat[0:3, 3] = pos 
                #print mat

               # print 'Error Code: ', res.error_code.val

                if  res.error_code.val!=1:
                    return
                pose = pose * mat

            
            else:
                dh_T = self._chain.fk(chain_state, self._dh_link_num)
                pose = pose * dh_T

        # Apply the 'after chain' transforms
        for after_chain_T in self._after_chain_Ts:
            pose = pose * after_chain_T.transform

        return pose
