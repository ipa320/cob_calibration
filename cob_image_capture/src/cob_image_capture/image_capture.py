#!/usr/bin/env python
PKG  = 'cob_image_capture'
NODE = 'image_capture'
import roslib; roslib.load_manifest(PKG)
import rospy

import cv
from sensor_msgs.msg import Image
from cob_image_capture.srv import *
from cv_bridge import CvBridge, CvBridgeError

class ImageCaptureNode():
    '''
    Captures Images from 1 or more cameras (Image message topics) to files.
    Number of cameras, output folder and file names are configurable via ROS parameters.
    
    After starting this node call "~capture_images" ROS service to dump images of all 
    cameras to output folder.
    '''
    def __init__(self, numCams=1): 
        '''
        Initializes storage, gets parameters from parameter server and logs to rosinfo
        '''
        rospy.init_node(NODE)
        self.numCams = numCams
        self.bridge = CvBridge()
        self.counter = 0
               
        # Get params from ros parameter server or use default
        self.numCams = int(rospy.get_param("~number_of_cameras", "1"))
        self.output_folder = rospy.get_param("~output_folder", "/tmp")
        self.camera = []
        self.file_prefix = []
        for id in range(self.numCams):
            self.camera.append(rospy.get_param("~camera%d" % id, "/stereo/left/image_raw"))
            self.file_prefix.append(rospy.get_param("~file_prefix%d" % id, "cam%d_" % id))

        # Init images
        self.image = []
        for id in range(self.numCams):
            self.image.append(Image())

        # Subscribe to images
        self.imageSub = []
        for id in range(self.numCams):
            self.imageSub.append(rospy.Subscriber(self.camera[id], Image, self._imageCallback, id))
        
        # Wait for image messages
        for id in range(self.numCams):
            rospy.wait_for_message(self.camera[id], Image, 1)
        
        # Report
        rospy.loginfo("started capture process...")
        rospy.loginfo("capturing images from")
        for id in range(self.numCams):
            rospy.loginfo(" %s -> files %s*" % (self.camera[id], self.file_prefix[id]))
        rospy.loginfo("to output folder %s" % self.output_folder)

    def _imageCallback(self, data, id):
        '''
        Copy image message to local storage
        '''
        #print "cb executed"
        self.image[id] = data
    
    def _convertAndSaveImage(self, rosImage, filenamePrefix, counter):
        '''
        Convert image to cvImage and store to file as jpg Image.
        '''   
        # save image
        cvImage = cv.CreateImage((1,1), 1 , 3)
        try:
            cvImage = self.bridge.imgmsg_to_cv(rosImage, "bgr8")
        except CvBridgeError, e:
            print e
        cv.SaveImage(self.output_folder+'/'+filenamePrefix+'%05d.jpg' % counter, cvImage)
                  
        # save header
        f = open(self.output_folder+'/'+filenamePrefix+'%05d_header.txt' % counter, "w")
        f.writelines(str(rosImage.header))
        f.close()
            
    def _captureImagesHandle(self, req):
        '''
        Service handle for "CaptureImages" Service
        Grabs images, converts them and saves them to files
        '''
        # grab image messages 
        localImages = []
        for id in range(self.numCams):
            if self.image[id].header.stamp > rospy.Time(0):
                localImages.append(self.image[id])
                rospy.loginfo("   header.stamp of cam %d:" % id, self.image[id].header.stamp)
        
        if len(localImages) == 0:
            rospy.loginfo("No sample captured, no image in queue.")
            return CaptureImagesResponse(False)
        
        # convert and save
        for id in range(self.numCams):
            self._convertAndSaveImage(localImages[id], self.file_prefix[id], self.counter)
        self.counter = self.counter + 1
        
        # log infos
        rospy.loginfo("-> %s sample(s) captured | captured %d set(s) of samples in total" % (self.numCams, self.counter))
        
        # return service response
        return CaptureImagesResponse(True)
        
    def run(self):
        # Start service
        srv = rospy.Service('~capture_images', CaptureImages, self._captureImagesHandle)
        rospy.loginfo("service of type 'CaptureImages' started, waiting for requests...")
        rospy.spin()

if __name__ == '__main__':
    node = ImageCaptureNode()
    node.run()
