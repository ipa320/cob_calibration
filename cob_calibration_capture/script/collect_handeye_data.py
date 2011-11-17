#!/usr/bin/env python
PKG  = 'cob_calibration_capture'
NODE = 'collect_hand_eye_data_node'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import yaml
from simple_script_server import simple_script_server
from cob_image_capture.srv import CaptureImages

OUT_FOLDER="/tmp/"

def capturePosition(listener, sample_id):
    try:
        time = rospy.Time(0)
        (trans, quat) = listener.lookupTransform('/base_link', '/head_axis_link', time)
    except (tf.LookupException, tf.ConnectivityException):
        print "TF exception"
        return False
    data = {}
    data["time_sec"]=time.secs
    data["time_nsecs"]=time.nsecs
    data["quat"]=list(quat)
    data["trans"]=list(trans)
    f = open(OUT_FOLDER + "base_head_%s.yaml" % sample_id, "w")
    f.writelines(yaml.dump(data))

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

    print "--> setting up tf listener"
    listener = tf.TransformListener()
    listener.waitForTransform('/base_link', '/head_color_camera_r_link', rospy.Time(), rospy.Time(1.0))
    
    print "--> setup care-o-bot for capture"
    sss.move("head", "back")
    sss.sleep(1)
    sss.move("torso", "home")
    sss.sleep(1)

    print "==> capturing positions and images"
#    positions = ["calib_front_left2", "calib_front_left", "calib_front", "calib_front_right", "calib_front_right2", 
#                 "calib_right2", "calib_right", "home", "calib_left", "calib_left2",
#                 "calib_back_left2", "calib_back_left", "calib_back", "calib_back_right", "calib_back_right2"]
    positions = ["calib_front_left2", "calib_front", "calib_front_right2", 
                 "calib_right2", "home", "calib_left2",
                 "calib_back_left2", "calib_back", "calib_back_right2"]
    for i in range(len(positions)):
        sss.sleep(1)
        sss.move("torso", positions[i])
        sss.sleep(1)
        print "--> capturing %s sample" % positions[i]
        captureImage()
        capturePosition(listener, i)
    
if __name__ == '__main__':
    main()
    print "==> done exiting"
