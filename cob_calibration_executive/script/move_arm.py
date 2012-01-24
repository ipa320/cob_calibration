#!/usr/bin/env python
PKG  = 'cob_calibration_executive'
NODE = 'move_arm'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys

from simple_script_server import simple_script_server

# main
def main(args):
    rospy.init_node(NODE)
    print "==> started " + NODE

    # extract args
    pos = None
    allowed_pos = ["calib", "hand_eye", "stereo"]
    if len(args) > 0:
	if args[0] in allowed_pos: pos = args[0]
    if pos == None: pos = "calib"

    # init
    sss = simple_script_server()

    # movements
    print "==> starting movements"
    sss.sleep(1.0)   
 
    print "--> moving arm to pregrasp"
    sss.move("arm", "pregrasp")
    
    print "--> moving arm to %s" % pos
    sss.move("arm", pos)
    
if __name__ == '__main__':
    args = sys.argv
    main(args[1:])
    print "==> done exiting"
