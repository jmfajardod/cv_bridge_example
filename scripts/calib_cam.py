#!/usr/bin/env python

import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import skimage

class Camera_ORB:

    #------------------------------------------------------#
    # Callback function for image
    def cam_image_cb(self,data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, 'bgr8')
            #cv_image = self.bridge.imgmsg_to_cv2(data), self.encoding)
        except CvBridgeError as e:
            print(e)
            return

        #rospy.loginfo("Image found")
        
        # find the keypoints with ORB
        cv_image_gray = cv2.cvtColor(cv_image.copy(), cv2.COLOR_BGR2GRAY)

        # find the keypoints with ORB
        kp = self.orb.detect(cv_image_gray,None)

        # compute the descriptors with ORB
        kp, des = self.orb.compute(cv_image_gray, kp)

        #rospy.loginfo("Keypoints computed")

        # draw only keypoints location,not size and orientation
        cv_image2 = cv2.drawKeypoints(cv_image, kp, None, color=(0,255,0), flags=0)

        #cv2.imshow("Image window", cv_image2)
        #cv2.waitKey(3)

        try:
            #self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(cv_image2))
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image2, "bgr8"))
        except CvBridgeError as e:
            print(e)

    #------------------------------------------------------#
    #------------------------------------------------------#
    #------------------------------------------------------#

    def __init__(self):

        self.cameraTopic = None
        self.encoding = None

        # Load parameters from parameter server
        self.getParameters()

        # Check that the parameters where correctly loaded
        if(self.cameraTopic is None or self.encoding is None):
            rospy.signal_shutdown("Parameters not read")
        else:
            rospy.loginfo("Parameters found")

        # Create CV bridge
        self.bridge = CvBridge()

        # Create ORB detector
        self.orb = cv2.ORB_create()

        # Create image subscriber
        self.image_sub = rospy.Subscriber(self.cameraTopic, CompressedImage, self.cam_image_cb)

        # Create image publisher
        self.image_pub = rospy.Publisher("image_orb_features", Image, queue_size=10)
        

    #------------------------------------------------------#
    # Function to get parameters
    def getParameters(self):

        if rospy.has_param('~cam_topic'):   self.cameraTopic = rospy.get_param('~cam_topic')
        if rospy.has_param("~encoding"):    self.encoding = rospy.get_param("~encoding")


#-----------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------#

if __name__ == '__main__':

    # Firt init the node and then the object to correctly find the parameters
    rospy.init_node('image_features', anonymous=True)
    Camera_ORB()
    
        
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()