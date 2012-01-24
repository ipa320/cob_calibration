#!/usr/bin/env python
PKG  = 'cob_calibration_executive'
NODE = 'move_arm'
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
    print "==> starting movements"
    
    stereo = rospy.get_param("/script_server/arm/stereo")
    print stereo

    for pos in stereo:
        print "--> moving arm to %s" % pos
        sss.move("arm", pos)
   
if __name__ == '__main__':
    main()
    print "==> done exiting"
