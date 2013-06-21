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

import numpy
from numpy import matrix, vsplit, sin, cos, reshape, pi, array
import rospy
from PyKDL import Chain, Segment, Joint, Frame, Vector, Rotation, ChainFkSolverPos_recursive, JntArray, RigidBodyInertia
from functools import reduce

class DhChain:
    def __init__(self, config = [[0, 0, 0, 0]]):
        # Determine number of links

        # import code; code.interact(local=locals())
        # try:
        self._M = len(config['dh'])
        rospy.logdebug("Initializing dh chain with [%u] links", self._M)

        param_mat = config['dh']

        # self._length = param_mat.size       # The number of params needed to configure this chain
        # self._config = param_mat            # Mx4 matrix. Each row is a link, containing [theta, alpha, a, d]

        self._length = len(param_mat * 6)
        self._config = param_mat

        self._cov_dict = config['cov']
        self._gearing = config['gearing']
        # except:
            # self._cov_dict=config['cov']
            # self._M=len(self._cov_dict['joint_angles'])
            # param_mat = numpy.matrix([[0]*4]*self._M, float)
            # self._length = param_mat.size       # The number of params needed to configure this chain
            # self._config = param_mat            # Mx4 matrix. Each row is a link, containing [theta, alpha, a, d]
            # self._gearing=[0]*self._M



    def calc_free(self, free_config):
        # import code; code.interact(local=locals())
        # print len(free_config['dh'])
        # print self._M
        if free_config is None:
            free_config = {}
        if 'dh' not in free_config:
            free_config['dh'] = [[0]*6]*self._M
        #else:
            #print free_config["dh"]
            #print [f for f in free_config["dh"]]
            #free_config['dh'] = [f["xyzrpy"] for f in free_config["dh"]]
        if 'gearing' not in free_config:
            free_config['gearing'] = [0]*self._M
        assert( len(free_config['dh']) == self._M)
        assert( len(free_config['gearing']) == self._M )

        # Flatten the config
        flat_config = sum(free_config['dh'], [])

        assert( len(flat_config) == self._M * 6)

        flat_config = flat_config + free_config['gearing']

        # Convert int list into bool list
        flat_free = [x == 1 for x in flat_config]

        return flat_free

    def params_to_config(self, param_vec):
        assert(param_vec.shape == (self._M * 7, 1)) # 6 pose params + 1 gearing per joint
        dh_param_vec = param_vec[0:(self._M * 6), 0]
        gearing_param_vec = param_vec[(self._M * 6):, 0]

        param_mat = reshape( dh_param_vec.T, (-1, 6))
        config_dict = {}
        config_dict['dh'] = self._config
        for sc, c in zip(config_dict["dh"], param_mat):
            sc["xyzrpy"] = c.tolist()[:][0]
        config_dict['gearing'] = (array(gearing_param_vec)[:, 0]).tolist()
        config_dict['cov'] = self._cov_dict
        return config_dict

    # Convert column vector of params into config
    def inflate(self, param_vec):
        param_mat = param_vec[:self._M*6,:]
        config = reshape(param_mat, (-1, 6))
        for sc, c in zip(self._config, config):
            sc["xyzrpy"] = c.tolist()[:][0]
        gearing = array(param_vec[self._M*6:,:])[:, 0].tolist()
        if len(gearing) is len(self._gearing):
            self._gearing = gearing

    # Return column vector of config
    def deflate(self):
        # param_vec = reshape(self._config, (-1,1))
        # param_vec = numpy.concatenate([param_vec, matrix(self._gearing).T])
        param_vec = []
        for joint in self._config:
            param_vec.extend(joint["xyzrpy"])
        param_vec.extend(self._gearing)
        param_vec = matrix(param_vec).T
        return param_vec

    # Returns # of params needed for inflation & deflation
    def get_length(self):
        return self._M*7

    # Returns 4x4 numpy matrix of the pose of the tip of
    # the specified link num. Assumes the last link's tip
    # when link_num < 0
    def fk(self, chain_state, link_num=-2):
        if link_num<0:
            link_num -=1
        link_num += 1
        if link_num < 0:
            link_num = self._M


        pos_scaled = [cur_pos * cur_gearing for cur_pos, cur_gearing in zip(chain_state.actual.positions,self._gearing)]

        #pos_scaled = JntArray(pos_scaled)
        jnt_states = JntArray(len(pos_scaled))
        for i,s in enumerate(pos_scaled):
            jnt_states[i] = s
        self.build_chain()
        F1=Frame()

        assert(0==self.fksolverpos.JntToCart(jnt_states,F1,link_num))


        out = []
        for i in range(3):
            line=[]
            for j in range(3):
                line.append(F1.M[i,j])
            line.append(F1.p[i])
            out.append(line)
        out.append([0,0,0,1])
        out= matrix(out)
        return out
    def build_chain(self):
        self.chain = Chain()
        for e in self._config:
            v = Vector(e["xyzrpy"][0], e["xyzrpy"][1], e["xyzrpy"][2])
            r = Rotation.RPY(e["xyzrpy"][3], e["xyzrpy"][4], e["xyzrpy"][5])
            j = Joint(e["name"], Joint.None)

            f = Frame(r, v)

            if e["type"] == "rotx":
                axis = Vector(1.0, 0.0, 0.0)
                j = Joint(e["name"],f.p, f.M * axis, Joint.RotAxis)
            elif e["type"] == "roty":
                axis = Vector(0.0, 1.0, 0.0)
                j = Joint(e["name"],f.p, f.M * axis, Joint.RotAxis)
            elif e["type"] == "rotz":
                axis = Vector(0.0, 0.0, 1.0)
                j = Joint(e["name"],f.p, f.M * axis, Joint.RotAxis)
            elif e["type"] == "transx":
                axis = Vector(1.0, 0.0, 0.0)
                j = Joint(e["name"],f.p, f.M * axis, Joint.TransAxis)
            elif e["type"] == "transy":
                axis = Vector(0.0, 1.0, 0.0)
                j = Joint(e["name"],f.p, f.M * axis, Joint.TransAxis)
            elif e["type"] == "transz":
                axis = Vector(0.0, 0.0, 1.0)
                j = Joint(e["name"],f.p, f.M * axis, Joint.TransAxis)



            self.chain.addSegment(Segment(e["name"]+"seg", j, f, RigidBodyInertia()))
        self.fksolverpos = ChainFkSolverPos_recursive(self.chain)


# Computes the transform for a chain
# dh_params: Mx4 matrix, where M is the # of links in the model
#            Each row represents a link [theta, alpha, a, d]
# joint_pos:
# returns: 4x4 numpy matrix
def chain_T(dh_params, joint_pos):
    if numpy.mod(dh_params.size, 4) != 0:
        raise Exception("input size must be of length multiple of 4. Current is %s = %u" % (dh_params.shape, dh_params.size))
    M = dh_params.size/4

    if len(joint_pos) != M:
        raise Exception("joint_positions must length must be [%u]. Currently is [%u]" % (M, len(joint_pos)))

    # Reshape input to be Nx4
    dh_final = numpy.array(dh_params.copy())
    dh_final.shape = (M, 4)

    for m in range(0, M):
        dh_final[m, 0] = dh_final[m, 0]+joint_pos[m]

    # Mutliply all the dh links together (T0 * T1 * T2 * ... * TN)
    return reduce( lambda x, y: x*y, [ link_T(r[0]) for r in vsplit(dh_final, M)] )

# Computes the 4x4 transformation matrix for a given set of dh
# parameters.
# dh_params - params of the form (theta(i), alpha(i), a(i), d(i))
# Output, a 4x4 transform from link (i-1) to (i)
def link_T(dh_params):
    # print "DH: %s" % dh_params
    theta = dh_params[0]
    alpha = dh_params[1]
    a = dh_params[2]
    d = dh_params[3]

    T_theta = matrix([ [ cos(theta), -sin(theta), 0, 0 ],
                       [ sin(theta),  cos(theta), 0, 0 ],
                       [    0,             0,     1, 0 ],
                       [    0,             0,     0, 1 ] ])

    T_alpha = matrix([ [ 1,     0,            0,     0 ],
                       [ 0, cos(alpha), -sin(alpha), 0 ],
                       [ 0, sin(alpha),  cos(alpha), 0 ],
                       [ 0,     0,            0,     1 ] ])

    T_trans = matrix([ [ 1, 0, 0, a ],
                       [ 0, 1, 0, 0 ],
                       [ 0, 0, 1, d ],
                       [ 0, 0, 0, 1 ] ])

    return T_theta * T_trans * T_alpha
