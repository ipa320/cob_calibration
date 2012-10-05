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
PKG  = 'cob_robot_calibration'
import roslib; roslib.load_manifest(PKG)
import rospy

import math
 
from tf.transformations import *



origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
def matrix2dh(matrix):
    d=matrix[2][3]
    cosphi=matrix[0][0]
    a=matrix[0][3]/cosphi
    phi=math.acos(cosphi)
    alpha=math.acos(matrix[2][2])
    return d,a,phi,alpha

def xyzrpy2matrix(t,r):
    Rx = rotation_matrix(r, xaxis)
    Ry = rotation_matrix(p, yaxis)
    Rz = rotation_matrix(y, zaxis)
    R = concatenate_matrices(Rx, Ry, Rz)
    T=translation_matrix(t)
    M=concatenate_matrices(R,T)
    print M
    return M
    
def xyzrpy2dh(t,r):
    return matrix2dh(xyzrpy2matrix(t,r))
    
    
    
    
    
    
def main():
    print matrix2dh(xyzrpy2matrix(( 0,0,-0.250),(0,0,3.1415926)))
    
    return 0

if __name__ == '__main__':
    main()

