#!/usr/bin/env python
import numpy as np
import cv2
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge,  CvBridgeError
import threading


class challengeDetect:
    def __init__(self):
        self.node_name = "Challenge Tracker"
        self.thread_lock = threading.Lock()
        self.pub_image = rospy.Publisher("~echo_image",\
                Image, queue_size=1)
        self.pub_notification = rospy.Publisher("/exploring_challenge", String, queue_size=10)
        self.notification = String()

        self.image_count = 0

        self.debugging = False

        self.bridge = CvBridge()

        self.image0 = cv2.imread('image00.jpg', 0) # sertac
        #self.image1 = cv2.imread('image01.png', 0) # car
        #self.image2 = cv2.imread('image02.jpg', 0) # ari
        #self.image3 = cv2.imread('image03.jpg', 0) # cat
        #self.images = [self.image0, self.image1, self.image2, self.image3]
        
        self.testing = cv2.imread('image00.jpg', 0) # sertac
        #print self.testing
        self.cbImage(self.testing)
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbImage(self,image_cv):
        thread = threading.Thread(target=self.processImage,args=(image_cv,))
        thread.setDaemon(True)
        thread.start()


    def processImage(self, image_cv):
            if not self.thread_lock.acquire(False):
                return
            #image_cv = self.bridge.imgmsg_to_cv2(image_msg)
    
            self.detection(image_cv)
            
            if self.debugging:
                try:
                    self.pub_image.publish(\
                            self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))
                except CvBridgeError as e:
                    print(e)
            self.thread_lock.release()
    
    
    def detection(self,  image_cv):
            image_cv_ = image_cv.copy()
            result = cv2.matchTemplate(image_cv, self.image0, cv2.TM_CCOEFF)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
            top_left = min_loc
            w, h = self.image0.shape[::-1]
            bottom_right = (top_left[0] + w, top_left[1] + h)
            print top_left
            print bottom_right
            plt.imshow(image_cv_)
            plt.show()
            cv2.rectangle(image_cv_,(0, 0),(200, 200),  bottom_right, 2, 2)
            plt.subplot(121),plt.imshow(image_cv_,cmap = 'gray')
            plt.show()

            print "hi"

if __name__ == "__main__":
    rospy.init_node("ChallengeDetectTest")
    e = challengeDetect()
    rospy.spin()
