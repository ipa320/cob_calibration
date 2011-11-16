#!/usr/bin/env python
PKG  = 'cob_calibration_capture'
NODE = 'move_torso'
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
    print "--> moving torso home"
    sss.move("torso", "home")
    
#    print "--> moving torso calib_front"
#    sss.move("torso", "calib_front")
#    print "--> moving torso calib_back"
#    sss.move("torso", "calib_back")

#    print "--> moving torso calib_back_left2"
#    sss.move("torso", "calib_back_left2")
#    print "--> moving torso calib_back_right2"
#    sss.move("torso", "calib_back_right2")

    print "--> moving torso calib_front_left2"
    sss.move("torso", "calib_front_left2")
    print "--> moving torso calib_front_left"
    sss.move("torso", "calib_front_left")
    print "--> moving torso calib_front_right"
    sss.move("torso", "calib_front_right")
    print "--> moving torso calib_front_right2"
    sss.move("torso", "calib_front_right2")
    
if __name__ == '__main__':
    main()
    print "==> done exiting"
