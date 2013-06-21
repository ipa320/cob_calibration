#!/usr/bin/env python
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


import roslib; roslib.load_manifest('cob_robot_calibration_est')

import sys
import unittest
import rospy
import time
import numpy

from cob_robot_calibration_est.dh_chain import chain_T
from cob_robot_calibration_est.dh_chain import DhChain
from sensor_msgs.msg import JointState

class LoadDhChain(unittest.TestCase):
    def setUp(self):
        print("")
        #params = [ [ 0, 0, 1, 0 ],
                   #[ 0, 0, 1, 0 ],
                   #[ 0, 0, 1, 2 ] ]
        params = [{"type" : "rotz", "xyzrpy":[1,0,0, 0,0,0]},
                  {"type" : "rotz", "xyzrpy":[1,0,0, 0,0,0]},
                  {"type" : "rotz", "xyzrpy":[1,0,2, 0,0,0]}]


        self.dh_chain = DhChain({'dh':params, 'gearing':[1,1,1], 'cov':{'joint_angles':[1,1,1]}})

class TestDhChain(LoadDhChain):
    def test_init(self):
        pass


    def test_get_length(self):
        self.assertEqual(self.dh_chain.get_length(), 21)

    def test_free(self):
        free_config = [ {'xyzrpy' : [ 1, 0, 0, 0, 0, 0 ]},
                        {'xyzrpy' : [ 1, 0, 0, 0, 0, 0 ]},
                        {'xyzrpy' : [ 1, 0, 1, 0, 0, 0 ]} ]

        free_list = self.dh_chain.calc_free({'dh':free_config, 'gearing':[0,0,0]})

        self.assertEqual(free_list[0],  True)
        self.assertEqual(free_list[1],  False)
        self.assertEqual(free_list[2],  False)
        self.assertEqual(free_list[12], True)
        self.assertEqual(free_list[14], True)
        self.assertEqual(free_list[17], False)

    def test_deflate(self):
        param_vec = self.dh_chain.deflate()
        self.assertEqual(param_vec[0,0],  1)
        self.assertEqual(param_vec[12,0], 1)
        self.assertEqual(param_vec[14,0], 2)
        self.assertEqual(param_vec[18,0], 1)

    def test_inflate(self):
        param_vec = numpy.reshape(numpy.matrix(numpy.arange(18)),(3,6))
        self.dh_chain.inflate(param_vec)
        print self.dh_chain._config[0]["xyzrpy"]
        self.assertEqual(self.dh_chain._config[0]["xyzrpy"][0], 0)
        self.assertEqual(self.dh_chain._config[1]["xyzrpy"][1], 7)
        self.assertEqual(self.dh_chain._config[2]["xyzrpy"][2], 14)

    def test_to_params(self):
        param_vec = self.dh_chain.deflate()
        param_vec[0,0] = 10
        config = self.dh_chain.params_to_config(param_vec)
        self.assertAlmostEqual(config['dh'][0]["xyzrpy"][0], 10, 6)
        self.assertAlmostEqual(config['dh'][2]["xyzrpy"][2], 2, 6)

    def test_fk_easy1(self):
        chain_state = JointState()
        chain_state.position = [0, 0, 0]
        eef = self.dh_chain.fk(chain_state, 0)
        eef_expected = numpy.matrix( [[ 1, 0, 0, 1],
                                      [ 0, 1, 0, 0],
                                      [ 0, 0, 1, 0],
                                      [ 0, 0, 0, 1]] )
        self.assertAlmostEqual(numpy.linalg.norm(eef-eef_expected), 0.0, 6)

    def test_fk_easy2(self):
        chain_state = JointState()
        chain_state.position = [numpy.pi/2, 0, 0]
        eef = self.dh_chain.fk(chain_state, 0)
        eef_expected = numpy.matrix( [[ 0,-1, 0, 0],
                                      [ 1, 0, 0, 1],
                                      [ 0, 0, 1, 0],
                                      [ 0, 0, 0, 1]] )
        print eef
        print eef_expected
        self.assertAlmostEqual(numpy.linalg.norm(eef-eef_expected), 0.0, 6)


    def test_fk_easy3(self):
        chain_state = JointState()
        chain_state.position = [numpy.pi/2, numpy.pi/2, 0]
        eef = self.dh_chain.fk(chain_state, 1)
        #print eef
        eef_expected = numpy.matrix( [[-1, 0, 0,-1],
                                      [ 0,-1, 0, 1],
                                      [ 0, 0, 1, 0],
                                      [ 0, 0, 0, 1]] )
        self.assertAlmostEqual(numpy.linalg.norm(eef-eef_expected), 0.0, 6)

    def test_fk_easy4(self):
        chain_state = JointState()
        chain_state.position = [0, 0, 0]
        eef = self.dh_chain.fk(chain_state, -1)
        #print eef
        eef_expected = numpy.matrix( [[ 1, 0, 0, 3],
                                      [ 0, 1, 0, 0],
                                      [ 0, 0, 1, 2],
                                      [ 0, 0, 0, 1]] )
        self.assertAlmostEqual(numpy.linalg.norm(eef-eef_expected), 0.0, 6)


class TestChainT(unittest.TestCase):

    def test_easy_identity(self):
            dh_elems     = numpy.matrix([0, 0, 0, 0])
            joints_elems = numpy.matrix([0])

            result = chain_T( dh_elems, joints_elems)
            expected_result = numpy.matrix(numpy.eye(4))
            expected_result.shape = 4,4

            self.assertAlmostEqual(numpy.linalg.norm(result-expected_result), 0.0, 4)

    def test_easy_theta(self):
            dh_elems     = numpy.matrix([numpy.pi, 0, 0, 0])
            joints_elems = numpy.matrix([0])

            result = chain_T( dh_elems, joints_elems)
            expected_result = numpy.matrix( [[-1, 0, 0, 0],
                                             [ 0,-1, 0, 0],
                                             [ 0, 0, 1, 0],
                                             [ 0, 0, 0, 1]])

            #import code; code.interact(local=locals())

            self.assertAlmostEqual(numpy.linalg.norm(result-expected_result), 0.0, 4)

    def test_easy_joint(self):
            print "TEST EASY JOINT"
            dh_elems     = numpy.matrix([0, 0, 0, 0], float)
            joints_elems = numpy.matrix([numpy.pi])

            result = chain_T( dh_elems, joints_elems)
            expected_result = numpy.matrix( [[-1, 0, 0, 0],
                                             [ 0,-1, 0, 0],
                                             [ 0, 0, 1, 0],
                                             [ 0, 0, 0, 1]])

            print result
            self.assertAlmostEqual(numpy.linalg.norm(result-expected_result), 0.0, 4)
            print "DONE WITH TEST"

    '''
    def test_hard(self):
        sample_nums = range(1,6)
        dh_filenames = ["test/data/chain_T_data/dh_%02u.txt" % n for n in sample_nums]
        joints_filenames = ["test/data/chain_T_data/joints_%02u.txt" % n for n in sample_nums]
        output_filenames = ["test/data/chain_T_data/output_%02u.txt" % n for n in sample_nums]

        for dh_filename, joints_filename, output_filename in zip(dh_filenames, joints_filenames, output_filenames):
            dh_elems     = [float(x) for x in open(dh_filename).read().split()]
            joints_elems = [float(x) for x in open(joints_filename).read().split()]
            output_elems = [float(x) for x in open(output_filename).read().split()]

            result = chain_T( numpy.array(dh_elems), numpy.array(joints_elems))
            expected_result = numpy.matrix(output_elems)
            expected_result.shape = 4,4

            self.assertAlmostEqual(numpy.linalg.norm(result-expected_result), 0.0, 4, "Failed on %s" % dh_filename)
    '''
if __name__ == '__main__':
    import rostest
    rostest.unitrun('cob_robot_calibration_est', 'test_ChainT', TestChainT, coverage_packages=['cob_robot_calibration_est.dh_chain'])
    rostest.unitrun('cob_robot_calibration_est', 'test_DhChain', TestDhChain, coverage_packages=['cob_robot_calibration_est.dh_chain'])
