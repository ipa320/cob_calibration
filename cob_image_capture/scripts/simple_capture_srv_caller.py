#!/usr/bin/env python
PKG  = 'cob_image_capture'
NODE = 'simple_capture_srv_caller'
import roslib; roslib.load_manifest(PKG)
import rospy

from cob_calibration_srvs.srv import Capture

SERVICE="/image_capture/capture_images"

def main():
    rospy.init_node(NODE)
    rospy.loginfo("started %s" % NODE)
    srv = rospy.ServiceProxy(SERVICE, Capture)
    
    while not rospy.is_shutdown():
        raw_input("Press enter to capture set of images by calling %s" %SERVICE) 
        srv.call()
    
if __name__ == "__main__":
    main()
