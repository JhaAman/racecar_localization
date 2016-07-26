#!/usr/bin/env python

#this program detects red/green/yellow blobs, identifies the center, and draws rectangles

#import statements
import threading
import rospy
from sensor_msgs.msg import Image
from BlobDetections.msg import ColorRGBA, Float64, Point
import cv2, cv_bridge
import numpy as np
import math


class blob_detector:
    #function that initializes nodes
    def __init__(self):
        #sets the HSV values for green, yellow, and red
        self.GREEN_BOUNDS = [np.array([40, 190, 0]), np.array([90, 255, 255])]
        self.YELLOW_BOUNDS = [np.array([23, 180, 160]), np.array([37, 255, 255])]
        self.RED_BOUNDS = [np.array([0, 190, 200]), np.array([15, 255, 255])]

        self.thread_lock = threading.Lock()
        #cv bridge is used to convert between ROS image messages and openCV images
        self.bridge = cv_bridge.CvBridge()
        #creates a window for the image
        cv2.namedWindow("window", 1)
        #creates a subscriber for the image
        self.image_sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.cbImage)
        self.pub_image = rospy.Publisher('image_output', Image)

    def cbImage(self, image_msg):
        thread = threading.Thread(target=self.image_callback, args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    #callback function for the image (passed to the subscriber)
    def image_callback(self, msg):
        if not self.thread_lock.acquire(False):
            return
        #converts rosmsg to 8bit bgr image
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #converts the bgr image to hsv
        imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #creates color masks using bounds
        redMask = cv2.inRange(imageHSV, self.RED_BOUNDS[0], self.RED_BOUNDS[1]).astype('uint8')
        yellowMask = cv2.inRange(imageHSV, self.YELLOW_BOUNDS[0], self.YELLOW_BOUNDS[1])
        greenMask = cv2.inRange(imageHSV, self.GREEN_BOUNDS[0], self.GREEN_BOUNDS[1])

        #final mask to be used
        finalMask = cv2.add(cv2.add(redMask, yellowMask), greenMask)
        #finalMask = yellowMask
        #creates hsv image with mask
        filteredHSV = cv2.bitwise_and(imageHSV, imageHSV, mask=finalMask)
        #converts filtered hsv image to bgr image
        imageBGR = cv2.cvtColor(imageHSV, cv2.COLOR_HSV2BGR)
        #converts bgr image to grayscale
        grayscale = cv2.cvtColor(filteredHSV, cv2.COLOR_BGR2GRAY)

        #finds contours
        contours, h = cv2.findContours(grayscale, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
        rospy.loginfo(len(contours))

        #creates box for every contour
        for j in range(0, len(contours)):
            hull = cv2.convexHull(contours[j])
            rect = cv2.minAreaRect(hull)

            box = cv2.cv.BoxPoints(rect)
            box = np.int0(box)

            #creates tuple for the center of the blob
            rectx = int(rect[0][0])
            recty = int(rect[0][1])
            center = (rectx, recty)

            #calculates the area of the blob
            area = rect[1][0] * rect[1][1]

            if  area > 1000:
                cv2.drawContours(imageBGR, [box], 0, (0, 0, 255), 2)
                cv2.circle(imageBGR, center, 10, (0, 0, 255), thickness=1)




        #cv2.imshow("window", imageBGR)

        self.pub_image.publish(self.bridge.cv2_to_imgmsg(imageBGR, "bgr8"))
        self.thread_lock.release()

#main method
rospy.init_node('blob_detection_node')
follower = blob_detector()
rospy.spin()
