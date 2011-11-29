#!/usr/bin/env python
PKG  = 'cob_calibration_capture'
NODE = 'say_test'
import roslib; roslib.load_manifest(PKG)
import rospy

from simple_script_server import simple_script_server

# main
def main():
    rospy.init_node(NODE)
    print "==> started " + NODE
    
    # init
    sss = simple_script_server()

    # movements
    print "==> starting speech output"
    sss.say(["Hello, my name is Care-o-Bot."])
    sss.say(["Its sunny so I am happy today!"])
    
    
if __name__ == '__main__':
    main()
    print "==> done exiting"
