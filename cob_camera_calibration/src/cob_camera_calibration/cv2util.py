#!/usr/bin/env python
#################################################################
##\file
#
# \note
#   Copyright (c) 2011-2012 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_calibration
# \note
#   ROS package name: cob_camera_calibration
#
# \author
#   Author: Sebastian Haug, email:sebhaug@gmail.com
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: November 2011
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import cv2
import numpy as np

'''
Provides helper functions to convert between opencv's cv and cv2 python interface data types
'''
def np2cvmat(np):
    '''
    Converts np.array (cv2 interface) to cvmat (cv interface)
    '''
    np_type2cvmat_type = {
        ('uint8', 1): cv2.CV_8UC1,
        ('uint8', 2): cv2.CV_8UC2,
        ('uint8', 3): cv2.CV_8UC3,
        ('uint8', 4): cv2.CV_8UC4,
        
        ('int8', 1): cv2.CV_8SC1,
        ('int8', 2): cv2.CV_8SC2,
        ('int8', 3): cv2.CV_8SC3,
        ('int8', 4): cv2.CV_8SC4,
        
        ('uint16', 1): cv2.CV_16UC1,
        ('uint16', 2): cv2.CV_16UC2,
        ('uint16', 3): cv2.CV_16UC3,
        ('uint16', 4): cv2.CV_16UC4,
        
        ('int16', 1): cv2.CV_16SC1,
        ('int16', 2): cv2.CV_16SC2,
        ('int16', 3): cv2.CV_16SC3,
        ('int16', 4): cv2.CV_16SC4,
        
        ('int32', 1): cv2.CV_32SC1,
        ('int32', 2): cv2.CV_32SC2,
        ('int32', 3): cv2.CV_32SC3,
        ('int32', 4): cv2.CV_32SC4,
        
        ('float32', 1): cv2.CV_32FC1,
        ('float32', 2): cv2.CV_32FC2,
        ('float32', 3): cv2.CV_32FC3,
        ('float32', 4): cv2.CV_32FC4,
        
        ('float64', 1): cv2.CV_64FC1,
        ('float64', 2): cv2.CV_64FC2,
        ('float64', 3): cv2.CV_64FC3,
        ('float64', 4): cv2.CV_64FC4
    }

    try:    channels = np.shape[2]
    except: channels = 1
        
    cvmat = cv2.cv.CreateMatHeader(np.shape[1],np.shape[0], np_type2cvmat_type[(str(np.dtype), channels)])
    cv2.cv.SetData(cvmat, np, cv2.cv.CV_AUTOSTEP)
    return cvmat

def cvmat2np(cvmat):
    '''
    Converts cvmat (cv interface) data to np.array (cv2 interface)
    '''
    cvmat_type2np_type = {
        cv2.CV_8UC1: 'uint8',
        cv2.CV_8UC2: 'uint8',
        cv2.CV_8UC3: 'uint8',
        cv2.CV_8UC4: 'uint8',
        
        cv2.CV_8SC1: 'int8',
        cv2.CV_8SC2: 'int8',
        cv2.CV_8SC3: 'int8',
        cv2.CV_8SC4: 'int8',
        
        cv2.CV_16UC1: 'uint16',
        cv2.CV_16UC2: 'uint16',
        cv2.CV_16UC3: 'uint16',
        cv2.CV_16UC4: 'uint16',
        
        cv2.CV_16SC1: 'int16',
        cv2.CV_16SC2: 'int16',
        cv2.CV_16SC3: 'int16',
        cv2.CV_16SC4: 'int16',
        
        cv2.CV_32SC1: 'int32',
        cv2.CV_32SC2: 'int32',
        cv2.CV_32SC3: 'int32',
        cv2.CV_32SC4: 'int32',
        
        cv2.CV_32FC1: 'float32',
        cv2.CV_32FC2: 'float32',
        cv2.CV_32FC3: 'float32',
        cv2.CV_32FC4: 'float32',
        
        cv2.CV_64FC1: 'float64',
        cv2.CV_64FC2: 'float64',
        cv2.CV_64FC3: 'float64',
        cv2.CV_64FC4: 'float64'
    }

    count = cvmat.width * cvmat.height * cvmat.channels
    nparray = np.fromstring(cvmat.tostring(), cvmat_type2np_type[cvmat.type], count=count)
    nparray.shape = (cvmat.height, cvmat.width, cvmat.channels)
    return nparray
