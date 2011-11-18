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
    
    # moving to all calibration positions
    positions = ["calib_front_left2", "calib_front_left", "calib_front", "calib_front_right", "calib_front_right2", 
                 "calib_right2", "calib_right", "home", "calib_left", "calib_left2",
                 "calib_back_left2", "calib_back_left", "calib_back", "calib_back_right", "calib_back_right2"]
    for i in range(len(positions)):
        print "--> moving torso %s" % positions[i]
        sss.move("torso", positions[i])
        #sss.sleep(0.5)
    
    # move back home
    print "--> moving torso home"
    sss.move("torso", "home")
    
if __name__ == '__main__':
    main()
    print "==> done exiting"
