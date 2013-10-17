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

from xml.dom import minidom
import numpy


class CalibrationUrdfUpdater():
    '''
    Parses calibration.urdf.xarco and provides method to update it
    '''

    def __init__(self, urdf_in, urdf_out, debug=False, urdf_default=None):
        '''
        Init object with paths to input and output xml
        '''
        self.file_urdf_default = urdf_default
        self.file_urdf_in = urdf_in
        self.file_urdf_out = urdf_out
        self.debug = debug

    def update_references(self, attributes2update):

        print "--> loading calibration xml file from '%s'" % self.file_urdf_in
        xml_dom = minidom.parse(file(self.file_urdf_in))

        for node in xml_dom.getElementsByTagName("property"):
            # sanity check, all property elements need name and value attributes
            assert node.hasAttribute("name") and node.hasAttribute("value")
            attr_name = node.getAttribute("name")
            attr_value = float(node.getAttribute("value"))

            if attr_name in attributes2update:
                attr_value_new = attributes2update[attr_name] + attr_value
                node.setAttribute("value", str(attr_value_new))
                if self.debug:
                    print "Updating '%s' from '%s' to '%s'" % (attr_name, attr_value, attr_value_new)
        # convert changed dom to pretty xml string
        xml_string = xml_dom.toprettyxml(indent="", newl="")

        # save string to xml_out
        print "--> saving results to calibration xml file '%s'" % self.file_urdf_out
        f = open(self.file_urdf_out, "w")
        f.writelines(xml_string)
        f.close()

    def update(self, attributes2update):
        '''
        Read urdf xml file (self.file_urdf_in) and replace values according to attributes2update dictionary.
        Save results to urdf xml file (self.file_urdf_out)

        @param attributes2update: names of arguments (of property tags) in urdf xml file which need to be updated -> new values
        @type  attributes2update: dictionary
        '''
        # parse xml file to dom
        print "--> loading calibration xml file from '%s'" % self.file_urdf_in
        xml_dom = minidom.parse(file(self.file_urdf_in))
        if self.file_urdf_default is not None:
            xml_dom_def = minidom.parse(file(self.file_urdf_default))
            default_property_nodes = xml_dom_def.getElementsByTagName(
                "property")

        # get all property elements
        for node in xml_dom.getElementsByTagName("property"):
            # sanity check, all property elements need name and value attributes
            assert node.hasAttribute("name") and node.hasAttribute("value")
            attr_name = node.getAttribute("name")
            attr_value = node.getAttribute("value")
            attr_name_def = attr_name.replace("offset", "def")
           # print attr_name, attr_name_def
            if self.file_urdf_default is not None:
                for node_def in default_property_nodes:

                    #print node_def.getAttribute("name"), attr_name_def
                    if node_def.getAttribute("name") == attr_name_def:
                        attr_value_def = float(node_def.getAttribute("value"))
                        break
            else:
                attr_value_def = 0
            #print attr_value_def
            # if attributes name is in attributes2update dict, update attributes value to new value (attributes2update[name])
            if attr_name in attributes2update:
                attr_value_new = attributes2update[attr_name] - attr_value_def

                if "roll" in attr_name or "pitch" in attr_name or "yaw" in attr_name:
                    attr_value_new = (
                        attr_value_new + numpy.pi) % (2 * numpy.pi) - numpy.pi
                node.setAttribute("value", str(attr_value_new))
                if self.debug:
                    print "Updating '%s' from '%s' to '%s'" % (attr_name, attr_value, attr_value_new)

        # convert changed dom to pretty xml string
        xml_string = xml_dom.toprettyxml(indent="", newl="")

        # save string to xml_out
        print "--> saving results to calibration xml file '%s'" % self.file_urdf_out
        f = open(self.file_urdf_out, "w")
        f.writelines(xml_string)
        f.close()
