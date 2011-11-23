#!/usr/bin/env python
PKG  = 'cob_calibration_capture'
NODE = 'collect_stereo_data_node'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import yaml
from simple_script_server import simple_script_server
from cob_image_capture.srv import CaptureImages

def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE
    
    # service client
    image_capture_service_name = "/image_capture/capture_images"
    captureImage = rospy.ServiceProxy(image_capture_service_name, CaptureImages)
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
    sss.sleep(1)
    sss.move("torso", "home")
    sss.sleep(1)

    print "==> capturing positions and images"
    positions = rospy.get_param("/script_server/arm/stereo")
    print positions
    for pos in positions:
        print "--> moving arm to pos %s sample" % pos
        sss.sleep(1)
        sss.move("arm", pos)
        sss.sleep(1)
        print "--> capturing %s sample" % pos
        captureImage()
    
if __name__ == '__main__':
    main()
    print "==> done exiting"
