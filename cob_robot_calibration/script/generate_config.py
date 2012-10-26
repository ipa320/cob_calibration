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
#   ROS package name: cob_robot_calibration
#
# \author
#   Author: Sebastian Haug, email:sebhaug@gmail.com
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: January 2012
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
PKG  = 'cob_robot_calibration'
import roslib; roslib.load_manifest(PKG)
import rospy

from simple_script_server import simple_script_server
import yaml
import tf



def get_transformation_camera_chain(d,origin):
    return get_transformation_chain(d['chain'],origin)
    

def get_transformation_chain(d,origin):
    transformation=[]
    transformation.append(origin+d['before_chain'])
    ll=[d['last_link']]
    after_chain=d['after_chain']
    after_chain=ll+after_chain
    transformation+=[after_chain]
    to_calibrate=[d['before_chain'][-1],after_chain[-1]]
    return transformation, to_calibrate
    
def prettyprint(l):
    for x in l:
        print x
        
def get_chains(d,origin=['base_link']):
    print '[INFO]: %s was selected as root frame'%origin
    if not isinstance(d,dict):
        print "[ERROR]: d should be dict, is %s"%type(d)
        return
    if not isinstance(origin, list):
        origin=[origin]
        print "[INFO] origin should be list, is %s. Conversion done"%type(origin)
    to_calibrate= []
    transformations=[]
    for k,v in d.iteritems():
        if k=='camera_chains':
            for x in v:
                t,to_calib=get_transformation_camera_chain(x,origin)
                transformations.append(t)
                to_calibrate+=to_calib
        elif k=='chains':
            for x in v:
                t,to_calib=get_transformation_chain(x,origin)
                transformations.append(t)
                to_calibrate+=to_calib
    to_calibrate=list(set(to_calibrate))
    
    return transformations,to_calibrate

def generate_transformation_dict(transformation_list,listener):
    transformation_dict={}
    for t in transformation_list:
        for chain in t:
            index_list=range(len(chain))
            index_list.remove(0)
            for index in index_list:
                key=chain[index]
                if not transformation_dict.has_key(key):
                    
                    value=get_single_transform(chain[index-1],chain[index],listener)
                    transformation_dict[key]=value
    return transformation_dict
                
            

def get_single_transform(parent_link,child_link,listener):
    try:
        translation,rotation=listener.lookupTransform('/'+parent_link,'/'+child_link,rospy.Time(0))
        rotation=tf.transformations.euler_from_quaternion(rotation)
        transform=list(translation)+list(rotation)
    except tf.LookupException:
        transform=[0]*6
    return map(round,transform,[4]*len(transform))
    


def settozero(d):
    if not isinstance(d,dict):
        print "Error: expected dict, got %s"%type(d)
        return
    output={}
    for k,v in d.iteritems():
        if isinstance(v,dict):
            v=settozero(v)
        elif isinstance(v,list):
            v=[0]*len(v)
        else:
            v=0
        output[k]=v
    return output
        
    
    
            
def __main__():
    rospy.init_node('cob_robot_calibration_generate_cal')
    listener=tf.TransformListener()
    rospy.sleep(1)
    sss=simple_script_server()
    sss.move("head","back")
    minimal_system=rospy.get_param('minimal_system',None)
    sensors=rospy.get_param('sensors',None)
    output_system=rospy.get_param('output_system',None)
    free_system=rospy.get_param('free_system',None)
    
    z=yaml.load(open(sensors,'r'))
    transformations, to_calib=get_chains(z)
    print z
    transformation_dict= generate_transformation_dict(transformations,listener)
    system=yaml.load(open(minimal_system,'r'))
    
    
    
    if system.has_key('transforms'):
        
        t=dict(transformation_dict,**system['transforms'])
    else:
        t=transformation_dict
    
    system['checkerboards']=rospy.get_param('~checkerboards',None)
    if system['checkerboards'] is None:
        print '[ERROR]: Parameter checkerboards not found. Make sure it is set and try again'
        return
    system['transforms']=t
    
    
    free=system.copy()
    
    
    free=settozero(free)
    
    '''
    print yaml.dump(system)
    
    print "Calibration transformation suggestions are %s"%to_calib
    
    print yaml.dump(free)
    '''
    with open(output_system,'w') as f:
        f.write(yaml.dump(system))
    
    with open(free_system,'w') as f:
        f.write(yaml.dump(free))
    
if __name__=="__main__":
    __main__()


