#! /usr/bin/env python
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

import roslib; roslib.load_manifest('pr2_calibration_estimation')

import sys
import rospy
import time
import numpy
import rosrecord
import yaml
import os.path
import numpy

from pr2_calibration_estimation.robot_params import RobotParams
from pr2_calibration_estimation.blocks import robot_measurement_bundler

def usage():
    rospy.logerr("Not enough arguments")
    print "Usage:"
    print " ./proto1.py [bagfile] [output_dir]"
    sys.exit(0)

class ErrorCalc:
    def __init__(self, robot_params, expanded_params, free_list, blocks):
        self._robot_params = robot_params
        self._expanded_params = expanded_params
        self._free_list = free_list
        self._blocks = blocks

    # Take the set of optimization params, and expand it into the bigger param vec
    def calculate_full_param_vec(self, opt_param_vec):
        full_param_vec = self._expanded_params.copy()

        #import code; code.interact(local=locals())

        full_param_vec[numpy.where(self._free_list), 0] = opt_param_vec

        return full_param_vec

    def calculate_error(self, opt_param_vec):
        # print "x ",
        # sys.stdout.flush()

        full_param_vec = self.calculate_full_param_vec(opt_param_vec)

        # Update the primitives with the new set of parameters
        self._robot_params.inflate(full_param_vec)
        # Update all the blocks' configs (This might not be necessary, because everything should point to the correct object
        for block in self._blocks:
            block.update_config(self._robot_params)

        # Compute Errors
        errors = [numpy.reshape(block.compute_error()*block.error_scalar, (-1,1)) for block in self._blocks]

        error_vec = numpy.concatenate(errors, 0)

        error_array = numpy.array(error_vec).T[0]

        rms_error = numpy.sqrt( numpy.mean(error_array**2) )
        print "%.3f " % rms_error,
        sys.stdout.flush()

        return error_array.copy()

def opt_runner(robot_params_dict, free_dict, blocks):
    # Load the robot params
    robot_params = RobotParams()
    robot_params.configure(robot_params_dict)

    for block in blocks:
        block.update_config(robot_params)

    # Load the free configuration
    free_list = robot_params.calc_free(free_dict)
    expanded_param_vec = robot_params.deflate()

    error_calc = ErrorCalc(robot_params, expanded_param_vec, free_list, blocks)

    # Construct the initial guess
    opt_guess_mat = expanded_param_vec[numpy.where(free_list), 0].copy()
    opt_guess = numpy.array(opt_guess_mat)[0]

    import scipy.optimize
    x, cov_x, infodict, mesg, iter = scipy.optimize.leastsq(error_calc.calculate_error, opt_guess.copy(), full_output=1)

    # A hacky way to inflate x back into robot params
    full_param_vec = error_calc.calculate_full_param_vec(x)

    output_dict = error_calc._robot_params.params_to_config(full_param_vec)

    # Compute the rms error
    final_error = error_calc.calculate_error(x)
    rms_error = numpy.sqrt( numpy.mean(final_error**2) )
    print "RMS Error: %f" % rms_error

    return output_dict

if __name__ == '__main__':
    rospy.init_node("multi_step_estimator")

    print "Starting The Multi Step Estimator Node\n"

    if (len(rospy.myargv()) < 2):
        usage()
    elif (len(rospy.myargv()) < 3):
        bag_filename = rospy.myargv()[1]
        output_dir = "."
    else:
        bag_filename = rospy.myargv()[1]
        output_dir = rospy.myargv()[2]

    print "Using Bagfile: %s\n" % bag_filename
    if not os.path.isfile(bag_filename):
        rospy.logerr("Bagfile does not exist. Exiting...")
        sys.exit(1)

    config_param_name = "calibration_config"
    if not rospy.has_param(config_param_name):
        rospy.logerr("Could not find parameter [%s]. Please populate this namespace with the estimation configuration.", config_param_name)
        sys.exit(1)
    config = rospy.get_param(config_param_name)

    # Process all the block definitions that are on the parameter server
    blocks_name = "blocks"
    if blocks_name not in config.keys():
        rospy.logerr("Could not find namespace [%s/%s]. Please populate this namespace with blocks.", (config_param_name, blocks_name))
        sys.exit(1)
    blocks_dump = [yaml.load(x) for x in config[blocks_name].values()]

    # There could be multiple defitions of blocks in the blocks namespace. We
    # need to merge all of these into a single consistent dictionary
    all_blocks_dict = dict()
    #import code; code.interact(local=locals())
    for cur_blocks_file in blocks_dump:
        for cur_block_type, cur_block_list in cur_blocks_file.items():
            for cur_block in cur_block_list:
                # We want block_ids to be unique. Thus, we should warn the user if there are duplicate block IDs being loaded
                if cur_block['block_id'] in all_blocks_dict.keys():
                    rospy.logwarn("Loading a duplicate [%s]. Overwriting previous" % cur_block['block_id'])
                all_blocks_dict[cur_block['block_id']] = cur_block
                all_blocks_dict[cur_block['block_id']]['block_type'] = cur_block_type

    print "The following error blocks have been loaded into the estimator:\n"
    # We want to sort by the types of blocks
    all_block_types = list(set([x['block_type'] for x in all_blocks_dict.values()]))


    for cur_block_type in all_block_types:
        print "  %s Blocks" % cur_block_type
        cur_block_ids = [cur_block_id for cur_block_id,cur_block in all_blocks_dict.items() if cur_block['block_type'] == cur_block_type]
        cur_block_ids.sort()
        for cur_block_id in cur_block_ids:
            print "   - %s" % cur_block_id
        print ""

    # Load all the calibration steps.
    # We want to execute the calibration in alphabetical order, based on the key names
    step_keys = config["cal_steps"].keys()
    step_keys.sort()
    step_list = []
    for step_name in step_keys:
        # Add the step name to the dict (since we'll lose this info once we listify)
        config["cal_steps"][step_name]["name"] = step_name
        step_list.append(config["cal_steps"][step_name])
    print "Executing the calibration steps in the following order:"
    for cur_step in step_list:
        print " - %s" % cur_step['name']


    # Specify which system the first calibration step should use.
    # Normally this would be set at the end of the calibration loop, but for the first step,
    # this is grabbed from the param server
    previous_system = yaml.load(config["initial_system"])

    # Load all the blocks from the bagfile and config file
    for cur_step in step_list:
        print ""
        print "*"*30
        print "Beginning [%s]" % cur_step["name"]

        # Need to load only the blocks that we're interested in
        cur_blocks = dict([(x,[]) for x in all_block_types])
        for req_block_id in cur_step['blocks']:
            # We need to now find req_block_id in our library of blocks
            if req_block_id in all_blocks_dict.keys():
                cur_block_type = all_blocks_dict[req_block_id]['block_type']
                cur_blocks[cur_block_type].append(all_blocks_dict[req_block_id])
            else:
                rospy.logerr("Could not find [%s] in block library. Skipping this block")

        bundler = robot_measurement_bundler.RobotMeasurementBundler(cur_blocks)
        blocks = bundler.load_from_bag(bag_filename)

        print "Executing step with the following blocks:"
        for cur_block_type, cur_block_list in cur_blocks.items():
            print "  %s Blocks:" % cur_block_type
            cur_block_ids = [cur_block['block_id'] for cur_block in cur_block_list]
            cur_block_ids.sort()
            for cur_block_id in cur_block_ids:
                count = len([0 for x in blocks if x.block_id == cur_block_id])
                print "   - %s (%u)" % (cur_block_id, count)
            print ""

        if len(blocks) == 0:
            rospy.logwarn("No error blocks were generated for this optimization step. Skipping this step.  This will result in a miscalibrated sensor")
            output_dict = previous_system
        else:
            free_dict = yaml.load(cur_step["free_params"])
            output_dict = opt_runner(previous_system, free_dict, blocks)

        out_f = open(output_dir + "/" + cur_step["output_filename"], 'w')
        yaml.dump(output_dict, out_f)
        out_f.close()

        previous_system = output_dict

