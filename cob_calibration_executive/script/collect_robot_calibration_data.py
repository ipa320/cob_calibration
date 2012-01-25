#!/usr/bin/env python
PKG  = 'cob_calibration_executive'
NODE = 'collect_robot_calibration_data_node'
import roslib; roslib.load_manifest(PKG)
import rospy

from simple_script_server import simple_script_server
from cob_calibration_srvs.srv import Capture

def capture_loop_arm(positions, sss, capture):
    '''
    Moves arm to all positions using script server instance sss 
    and calls capture() to capture samples
    '''
    for pos in positions:
        print "--> moving arm to pos %s sample" % pos
        sss.move("arm", pos)
        sss.sleep(1.5)
        capture()

def capture_loop_torso(positions, sss, capture):
    '''
    Moves torso to all positions using script server instance sss 
    and calls capture() to capture samples
    '''
    for pos in positions:
        print "--> moving torso to pos %s sample" % pos
        sss.move("torso", pos)
        sss.sleep(1.5)
        capture()

def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE
    
    # service client
    image_capture_service_name = "/image_capture/capture_images"
    capture = rospy.ServiceProxy(image_capture_service_name, Capture)
    rospy.wait_for_service(image_capture_service_name, 1)
    print "--> service client for capture images initialized"

    # init
    print "--> initializing sss"
    sss = simple_script_server()
    sss.init("base")
    sss.init("torso")
    sss.init("head")
    sss.recover("base")
    sss.recover("torso")
    sss.recover("head")
    
    print "--> setup care-o-bot for capture"
    sss.move("head", "back")

    # get position from parameter server
    positions_back = rospy.get_param("/script_server/arm/all_robot_back")
    positions_center = rospy.get_param("/script_server/arm/all_robot_center")
    positions_right = rospy.get_param("/script_server/arm/all_robot_right")
    positions_left = rospy.get_param("/script_server/arm/all_robot_left")
    torso_positions = ["calib_front_left2", "calib_front_right2", "calib_right2", "calib_left2", "calib_back_left2", "calib_back_right2", "home"]

    print "==> capturing images BACK"
    sss.move("torso", "back")
    capture_loop_arm(positions_back, sss, capture)
    
    print "==> capturing images LEFT"
    sss.move("torso", "left")
    capture_loop_arm(positions_left, sss, capture)
    
    print "==> capturing images RIGHT"
    sss.move("torso", "right")
    capture_loop_arm(positions_right, sss, capture)
    
    print "==> capturing images CENTER"
    sss.move("torso", "home")
    capture_loop_arm(positions_center, sss, capture)
    
    print "==> capturing TORSO samples"
    sss.move("arm", "calibration")
    capture_loop_torso(torso_positions, sss, capture)

if __name__ == '__main__':
    main()
    print "==> done exiting"
