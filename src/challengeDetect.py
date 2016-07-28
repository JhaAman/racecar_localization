#!/usr/bin/env python
import numpy as np
import cv2
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge,  CvBridgeError
import threading


class challengeDetect:
    def __init__(self):
        self.node_name = "Challenge Tracker"
        self.thread_lock = threading.Lock()
        
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("~echo_image",\
                Image, queue_size=1)
        self.pub_notification = rospy.Publisher("/exploring_challenge", String, queue_size=10)
        self.notification = String()

        self.image_count = 0

        self.debugging = debugging

        self.bridge = CvBridge()

        self.image0 = cv2.imread('img/image00.jpg', 0) # sertac
        self.image1 = cv2.imread('img/image01.png', 0) # car
        self.image2 = cv2.imread('img/image02.jpg', 0) # ari
        self.image3 = cv2.imread('img/image03.jpg', 0) # cat
        
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()


    def processImage(self, image_msg):
            if not self.thread_lock.acquire(False):
                return
            image_cv = self.bridge.imgmsg_to_cv2(image_msg)
    
            self.detection(image_cv)
            
            if self.debugging:
                try:
                    self.pub_image.publish(\
                            self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))
                except CvBridgeError as e:
                    print(e)
            self.thread_lock.release()
    
    
    def detection(self,  image_cv):
        
    def pickBiggest(contours):
        result = contours[0]
        for x in contours:
            if cv2.contourArea(x)>cv2.contourArea(result):
                result = x
        return result
            
    rgb = cv2.imread('image03.jpg')
    hsv = cv2.cvtColor(rgb,cv2.cv.CV_BGR2HSV)
    
    binMask = cv2.inRange(hsv, np.array([14,20,200]), np.array([20,50,255])) #sertac hair (4,0,0) to (10,255,100), cat face (14,50,200) to (20,150,255), sertac sweater (14,20,200) to (20,50,255) 
    contours, _ = cv2.findContours(binMask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
    
    biggest = pickBiggest(contours) if len(contours) > 0 else None
    x,y,w,h = cv2.boundingRect(biggest)
    print w*h
    if w*h > 500:
        cv2.rectangle(rgb, (x,y),(x+w,y+h),(0,255,0),2)
    
    # Show keypoints
    cv2.imshow("final",rgb)
    cv2.waitKey(0)

