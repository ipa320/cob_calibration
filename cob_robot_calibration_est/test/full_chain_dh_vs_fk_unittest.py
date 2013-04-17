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
import numpy
import math
from sensor_msgs.msg import JointState
from cob_robot_calibration_est.full_chain import FullChainCalcBlock
from cob_robot_calibration_est.single_transform import SingleTransform
from cob_robot_calibration_est.dh_chain import DhChain

from numpy import *
import numpy

def loadSystem1():
    calc_block1 = FullChainCalcBlock()
    calc_block2 = FullChainCalcBlock()
    before_chain_Ts = [SingleTransform([10, 0, 0, 0, 0, 0])]
    chain1 = DhChain( {'dh':[ [0, math.pi/2, 0, 0],[0, -math.pi/2, 0, -0.2565],[0,0,0,0] ],
                      'gearing':[1,1,1],
                      'cov':{'joint_angles':[0.01]*3}} )
    
    dh_link_num1 = 3
    after_chain_Ts = [SingleTransform([ 0, 0, 20, 0, 0, 0])]
    last_link='torso_upper_neck_tilt_link'
    first_link='torso_0_link'
    dh_chain=['torso_lower_neck_tilt_joint',
        'torso_pan_joint',
        'torso_upper_neck_tilt_joint']

    
    
    
    chain2=DhChain({'cov':{'joint_angles':[0.01]*3}})
    

    calc_block1.update_config(before_chain_Ts, chain1, dh_link_num1, after_chain_Ts)
    calc_block2.update_config(before_chain_Ts, chain2, None, after_chain_Ts, first_link,last_link,dh_chain,'torso')
    return (calc_block1,calc_block2)


class TestFullChainCalcBlock(unittest.TestCase):
    def test_fk_1(self):
        print ""
        (calc_block1,calc_block2) = loadSystem1()
        chain_state = JointState(position=[0,0,0])
        result1 = calc_block1.fk(chain_state)
        result2 = calc_block2.fk(chain_state)
        print result1
        print '*'*20
        print result2
        self.assertAlmostEqual(numpy.linalg.norm(result1-result2), 0.0, 6)

    def test_fk_2(self):
        print ""
        (calc_block1,calc_block2) = loadSystem1()
        chain_state = JointState(position=[math.pi/2]*3)
        result1 = calc_block1.fk(chain_state)
        result2 = calc_block2.fk(chain_state)
        print result1
        print '*'*20
        print result2
        self.assertAlmostEqual(numpy.linalg.norm(result1-result2), 0.0, 6)
    def test_fk_3(self):
        print ""
        (calc_block1,calc_block2) = loadSystem1()
        chain_state = JointState(position=[-math.pi/2]*3)
        result1 = calc_block1.fk(chain_state)
        result2 = calc_block2.fk(chain_state)
        print result1
        print '*'*20
        print result2
        self.assertAlmostEqual(numpy.linalg.norm(result1-result2), 0.0, 6)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node')
    rostest.unitrun('cob_robot_calibration_est', 'test_FullChainCalcBlock',   TestFullChainCalcBlock,   coverage_packages=['cob_robot_calibration_est.full_chain'])
