#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       untitled.py
#       
#       Copyright 2012 Jannik Abbenseth <fmw-ja@cob-stud-103>
#       
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 2 of the License, or
#       (at your option) any later version.
#       
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#       
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.


import roslib; roslib.load_manifest('cob_robot_calibration')
import rospy
import sys
import numpy

from urdf_parser_py.urdf import URDF

robot = None

def usage():
    print("display_urdf <urdf file>")
    print("\tLoad an URDF file.")
    print("display_urdf")
    print("\tDisplay the parameter server current robot description.")
    sys.exit(1)
    
    
import math
 
from tf.transformations import *



origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
def matrix2dh(matrix):
    
    #matrix=numpy.matrix(matrix).getI().getA()
    d=matrix[2][3]
    
    costheta=matrix[0][0]
    sintheta=matrix[0][1]
    tantheta=sintheta/costheta
    theta=math.atan2(sintheta,costheta)
    if costheta!=0:
        a=matrix[0][3]/costheta
    else:
        a=matrix[1][3]/sintheta
    
    cosalpha=matrix[2][2]
    sinalpha=matrix[2][1]
    tanalpha=sinalpha/cosalpha
    alpha=math.atan2(sinalpha,cosalpha)
    return [theta,alpha,a,d]

def xyzrpy2matrix(t,r):
    c=r[0]
    b=r[1]
    a=r[2]
    ca=math.cos(a)
    sa=math.sin(a)
    cb=math.cos(b)
    sb=math.sin(b)
    cc=math.cos(c)
    sc=math.sin(c)
    R=numpy.array([[ca*cb,  ca*sb*sc-sa*cc, ca*cb*cc+sa*sc],
                    [sa*cb, sa*sb*sc+ca*cc, sa*sb*cc-ca*sc],
                    [-sb,   cb*sc,          cb*cc]])

    #print '*'*5,' Rotations ','*'*5
    R=euler_matrix(r[0],r[1],r[2])
    #print R


    #print '*'*2,' Translation ','*'*2
    T=translation_matrix(t)
    #print T
    
    M=numpy.array([[ca*cb,  ca*sb*sc-sa*cc, ca*cb*cc+sa*sc, t[0]],
                    [sa*cb, sa*sb*sc+ca*cc, sa*sb*cc-ca*sc, t[1]],
                    [-sb,   cb*sc,          cb*cc,          t[2]],
                    [0,     0,              0,              1]])

    #M=concatenate_matrices(T,R)
    
    M=compose_matrix(translate=t,angles=r)
    return M
    
def xyzrpy2dh(t,r):
    return matrix2dh(xyzrpy2matrix(t,r))
    
def dh2matrix(dh):
    a=numpy.array([[math.cos(dh[0]),-math.sin(dh[0])*math.cos(dh[1]),math.sin(dh[0])*math.sin(dh[1]),dh[2]*math.cos(dh[0])],
       [math.sin(dh[0]),math.cos(dh[0])*math.cos(dh[1]),-math.cos(dh[0])*math.sin(dh[1]),dh[2]*math.sin(dh[0])],
       [0,math.sin(dh[1]),math.cos(dh[1]),dh[3]],
       [0,0,0,1]])
    return a
    

def main():
    if len(sys.argv) > 2:
        usage()
    if len(sys.argv) == 2 and (sys.argv[1] == "-h" or sys.argv[1] == "--help"):
        usage()

    if (len(sys.argv) == 1):
        robot = URDF.load_from_parameter_server()
    else:
        robot = URDF.load_xml_file(sys.argv[1])

    #print robot
    
     
    links=robot.links
    joints=robot.joints
    print dh2matrix([ -2.8407,  math.pi/2,  0, 0.1915  ])
    
        
    chains=['torso','arm']
    
    transforms=[['arm_%s_joint'%(i+1),'arm_%s_link'%(i+1)] for i in range(7)]  #ToDo: joint number from description
    print links

    for transform in transforms:
        print '*'*20
        joint_key=transform[0]
        
        t_joint=joints[joint_key].origin.position
        r_joint=joints[joint_key].origin.rotation

        joint_transformation    =xyzrpy2matrix(t_joint,r_joint)
        
        link_key=transform[1]
        t_link=links[link_key].inertial.origin.position
        r_link=links[link_key].inertial.origin.rotation
        link_transformation     =xyzrpy2matrix(t_link,r_link)
        
        
        print link_key, joint_key

        final_transformation=numpy.dot(joint_transformation,link_transformation)
        print matrix2dh(final_transformation)
        '''
        Not yet correct:
        translation from pan to tilt joint cannot be expressed by Denavit-Hartenberg Parameters
        Needs Workaround:
        calculate Translation from tilt to tilt joint, calculate DH-Parameter for section tilt to pan without translation
        '''        
       
        


if __name__=='__main__':
    main()
