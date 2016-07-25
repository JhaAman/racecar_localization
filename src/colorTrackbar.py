#!/usr/bin/env python
import threading
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import Float64, Header, ColorRGBA
from geometry_msgs.msg import Point
from racecar_vision.msg import blob_detection
import RacecarUtilities
import sys


class blob_detector:

    def __init__(self):
        # Trackbar creation
        def nothing(x):
            pass

        if len(sys.argv) == 2:
            self.image = np.zeros((720, 1280, 3), np.uint8)
            cv2.namedWindow('HSV')
            cv2.createTrackbar('HL', 'HSV', 0, 180, nothing)
            cv2.createTrackbar('SL', 'HSV', 0, 255, nothing)
            cv2.createTrackbar('VL', 'HSV', 0, 255, nothing)
            cv2.createTrackbar('HU', 'HSV', 0, 180, nothing)
            cv2.createTrackbar('SU', 'HSV', 0, 255, nothing)
            cv2.createTrackbar('VU', 'HSV', 0, 255, nothing)
            self.hl = 0
            self.sl = 0
            self.vl = 0
            self.hu = 0
            self.su = 0
            self.vu = 0
            self.is_tuning = sys.argv[1]
            self.window_thread = RacecarUtilities.StoppableThread(target=self.window_runner)
            self.window_thread.start()
        else:
            self.is_tuning = False
        # self.thread_lock = threading.Lock()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.image_callback)
        # self.pub_image = rospy.Publisher('image_output', Image, queue_size=1)
        self.info_pub = rospy.Publisher('target_info', blob_detection, queue_size=1)

    # def cbImage(self, image_msg):
    #   thread = threading.Thread(target=self.image_callback, args=(image_msg,))
   #    thread.setDaemon(True)
    #    thread.start()

    def image_callback(self, image_msg):
        # if not self.thread_lock.acquire(False):
         #   return
        self.image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        imageHSV = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        if self.is_tuning:
            TESTING_BOUNDS = [np.array([self.hl, self.sl, self.vl]), np.array([self.hu, self.su, self.vu])]
            finalMask = cv2.inRange(imageHSV, TESTING_BOUNDS[0], TESTING_BOUNDS[1])
        else:
            GREEN_BOUNDS = [np.array([40, 60, 40]), np.array([80, 255, 255])]
            # self.YELLOW_BOUNDS = [np.array([23, 180, 160]), np.array([37, 255, 255])]
            RED_BOUNDS = [np.array([0, 150, 90]), np.array([15, 255, 255])]
            redMask = cv2.inRange(imageHSV, RED_BOUNDS[0], RED_BOUNDS[1])
            # yellowMask = cv2.inRange(imageHSV, self.YELLOW_BOUNDS[0], self.YELLOW_BOUNDS[1])
            greenMask = cv2.inRange(imageHSV, GREEN_BOUNDS[0], GREEN_BOUNDS[1])
            finalMask = cv2.add(redMask, greenMask)
        # filteredHSV = cv2.bitwise_and(imageHSV, imageHSV, mask=finalMask)
        # imageBGR = cv2.cvtColor(filteredHSV, cv2.COLOR_HSV2BGR)
        # grayscale = cv2.cvtColor(imageBGR, cv2.COLOR_RGB2GRAY)
        contours, h = cv2.findContours(finalMask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
        # for drawing the contours themselves, uncomment the following line:
        # cv2.drawContours(rgb_new, contours, -1, (255,0,0), thickness=3)
        # rospy.loginfo(len(contours))
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            rect_area = w * h
            extent = float(area) / rect_area
            if extent > 0.78 and self.find_area(contour) > 100:
                rect = cv2.minAreaRect(contour)
                box = cv2.cv.BoxPoints(rect)
                box = np.int0(box)
                msg_header = Header()
                msg_header.stamp = rospy.Time.now()
                c = self.find_center(contour)
                pos = Point(x=c[0], y=c[1], z=0.0)
                blob_message = blob_detection(header=msg_header,
                                              color=ColorRGBA(r=self.image[c[1], c[0], 2], g=self.image[c[1], c[0], 1] , b=self.image[c[1], c[0], 0], a=1.0),
                                              size=Float64(data=self.find_area(contour)),
                                              location=pos)
                self.info_pub.publish(blob_message)
                if self.is_tuning:
                    cv2.drawContours(self.image, [box], 0, (255, 0, 0), 5)
                    cv2.circle(self.image, self.find_center(contour), 8, (255, 0, 0), thickness=5)
                break

        # try:
        #     # rospy.loginfo("Publishing")
        #     self.pub_image.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        # except CvBridgeError as e:
        #     print(e)
        # self.thread_lock.release()

    def find_area(self, contour):
        moments = cv2.moments(contour)

        return moments['m00']

    def find_center(self, contour):
        moments = cv2.moments(contour)
        x_val = int(moments['m10'] / moments['m00'])
        y_val = int(moments['m01'] / moments['m00'])

        return x_val, y_val

    def window_runner(self):
        # rospy.loginfo("Window Running")
        cv2.imshow('HSV', cv2.resize(self.image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            self.window_thread.stop()

        self.hl = cv2.getTrackbarPos('HL', 'HSV')
        self.sl = cv2.getTrackbarPos('SL', 'HSV')
        self.vl = cv2.getTrackbarPos('VL', 'HSV')
        self.hu = cv2.getTrackbarPos('HU', 'HSV')
        self.su = cv2.getTrackbarPos('SU', 'HSV')
        self.vu = cv2.getTrackbarPos('VU', 'HSV')


rospy.init_node('blob_detection_node')
node = blob_detector()
rospy.spin()
cv2.destroyAllWindows()
