#!/usr/bin/env python
PKG  = 'cob_calibration_executive'
NODE = 'collect_stereo_data_node'
import roslib; roslib.load_manifest(PKG)
import rospy

from simple_script_server import simple_script_server
from cob_calibration_srvs.srv import Capture

def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE
    
    # service client
    image_capture_service_name = "/image_capture/capture"
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
    sss.move("torso", "home")

    print "==> capturing images"
    positions = rospy.get_param("/script_server/arm/all_intrinsic")
    for pos in positions:
        print "--> moving arm to position '%s'" % pos
        sss.move("arm", pos)
        sss.sleep(1.5)
        print "--> capturing '%s' sample" % pos
        res = capture()
        if not res: print "--> ERROR during capture, skipping sample..."
        
    print "==> capturing finished, moving are to 'calibration' position"
    sss.move("arm", 'calibration')
    
if __name__ == '__main__':
    main()
    print "==> done exiting"
