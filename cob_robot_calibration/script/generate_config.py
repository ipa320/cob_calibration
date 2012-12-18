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
PKG = 'cob_robot_calibration'
import roslib
roslib.load_manifest(PKG)
import rospy

from simple_script_server import simple_script_server
import yaml
import tf


def get_chains(d, origin='base_link'):
    print '[INFO]: %s was selected as root frame' % origin
    if not isinstance(d, dict):
        print "[ERROR]: d should be dict, is %s" % type(d)
        return
    chains = transform_chain_dict(d['chains'])
    trees = []
    for k, v in d.iteritems():
        if k == 'camera_chains':
            for x in v:
                trees += build_tree(x['chain'], chains, origin)
        elif k == 'sensor_chains':
            for x in v:
                trees += build_tree(x, chains, origin)

    print " Trees: "
    print trees
    #to_calibrate = list(set(to_calibrate))
    return trees
    #return transformations, to_calibrate


def transform_chain_dict(d):
    new_dict = {}
    for chain in d:
        new_dict[chain['chain_id']] = chain
    return new_dict


def build_tree(chain, kinematic_chains, origin):
    tree = [origin]
    tree += [''.join(chain['before_chain'])]
    tree = cleanup_tree(tree)
    for chain_id in chain['chains']:
        kinematic_chain = kinematic_chains[chain_id]
        try:
            offset = -1
            if tree[offset] != kinematic_chain['parent_frame']:
                print 'There might be an Error in this tree due to undefined parent frame for kinematic chain'
            if tree[offset] == ''.join(kinematic_chain['before_chain']):
                tree += kinematic_chain['before_chain'][1:]
        except IndexError:
            tree += kinematic_chain['before_chain']
        tree += [''.join(kinematic_chain['links'][0])]
        tree += ['**']
        tree += [''.join(kinematic_chain['links'][1])]
        tree += [''.join(kinematic_chain['after_chain'])]
        tree = cleanup_tree(tree)
    tree += chain['after_chain']
    tree += ['**']
    print '*' * 3, ' New Tree ', '*' * 3
    print(tree)

    return tree


def cleanup_tree(tree):
    new_tree = []
    for v in tree:
        if v != '':
            new_tree.append(v)
    return new_tree


def generate_transformation_dict(transformation_list, listener):
    transformation_dict = {}
    for i in range(len(transformation_list)):
        if transformation_list[i] != '**' and transformation_list[i] != '':
            try:
                if transformation_list[i + 1] in ['', '**']:
                    continue
                transformation_dict[transformation_list[i + 1]] = get_single_transform(transformation_list[i], transformation_list[i + 1], listener)
            except IndexError:
                pass
    return transformation_dict


def get_single_transform(parent_link, child_link, listener):
    try:
        translation, rotation = listener.lookupTransform(
            '/' + parent_link, '/' + child_link, rospy.Time(0))
        rotation = tf.transformations.euler_from_quaternion(rotation)
        transform = list(translation) + list(rotation)
    except tf.LookupException:
        transform = [0] * 6
    return map(round, transform, [4] * len(transform))


def settozero(d):
    if not isinstance(d, dict):
        print "Error: expected dict, got %s" % type(d)
        return
    output = {}
    for k, v in d.iteritems():
        if isinstance(v, dict):
            v = settozero(v)
        elif isinstance(v, list):
            v = [0] * len(v)
        else:
            v = 0
        output[k] = v
    return output


def __main__():
    rospy.init_node('cob_robot_calibration_generate_cal')
    listener = tf.TransformListener()
    rospy.sleep(1)
    sss = simple_script_server()
    sss.move("head", "back")
    minimal_system = rospy.get_param('minimal_system', None)
    sensors = rospy.get_param('sensors', None)
    output_system = rospy.get_param('output_system', None)
    free_system = rospy.get_param('free_system', None)

    z = yaml.load(open(sensors, 'r'))
    transformations = get_chains(z)
    print z
    transformation_dict = generate_transformation_dict(
        transformations, listener)
    print transformation_dict
    system = yaml.load(open(minimal_system, 'r'))

    if 'transforms' in system:

        t = dict(transformation_dict, **system['transforms'])
    else:
        t = transformation_dict

    system['checkerboards'] = rospy.get_param('~checkerboards', None)
    if system['checkerboards'] is None:
        print '[ERROR]: Parameter checkerboards not found. Make sure it is set and try again'
        return
    system['transforms'] = t

    free = system.copy()

    free = settozero(free)

    '''
    print yaml.dump(system)

    print "Calibration transformation suggestions are %s"%to_calib

    print yaml.dump(free)
    '''
    with open(output_system, 'w') as f:
        f.write(yaml.dump(system))

    with open(free_system, 'w') as f:
        f.write(yaml.dump(free))

if __name__ == "__main__":
    __main__()
