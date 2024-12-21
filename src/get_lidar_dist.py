#!/usr/bin/env python
from math import floor
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import time
from tf.transformations import quaternion_from_euler
import tf2_ros
import geometry_msgs.msg
import csv

ANGLE_PER_PIXEL = 0.0703125
FILENAME = "./camera_data.csv"
class getDepth:
    def __init__(self):
        self.tfbuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfbuffer)
        file = open(FILENAME, 'w')
        self.writer = csv.writer(file)
        self.bridge = CvBridge()
        self.robot_name = rospy.get_param("~robot_number")
        rospy.Subscriber('/{}/camera/color/image_raw'.format(self.robot_name), Image, self.process_image)
        rospy.Subscriber('/{}/camera/aligned_depth_to_color/image_raw'.format(self.robot_name), Image, self.updateDepth)
        
        rospy.Subscriber('/scan', LaserScan, self.publish_laser)
        self.image = None
        self.scan_array = None
        self.angle_inc = None
        self.angle_min = None

    def detect_pink(self, img):
        hsvFrame = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        red_lower = np.array([114, 156, 80], np.uint8)
        red_upper = np.array([179, 255, 255], np.uint8)


        kernel = np.ones((5,5), "uint8")

        mask = cv2.inRange(hsvFrame, red_lower, red_upper)
        mask = cv2.dilate(mask, kernel)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]

        max_area = -1
        biggest_contour = None
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > max_area:
                biggest_contour = pic
                max_area = area

        try:
            x, y, w, h = cv2.boundingRect(contours[biggest_contour])
        except TypeError as e:
            rospy.logerr(e)
            return

        midx = x + (w//2)
        midy = y + (h//2)
        angle = (midx - 640) * ANGLE_PER_PIXEL
        rads = angle * (np.pi / 180)

        index = int((rads - self.angle_min) / self.angle_inc)
        distance_found = self.scan_array[index]
        print(distance_found)
        



    def updateDepth(self, data):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(data)
            self.image = cv_depth
            rospy.loginfo("self.image updated")
        except CvBridgeError as e:
            rospy.logerr("SELF.IMAGE FAILED TO UPDATE")
            rospy.logerr(e)
            rospy.logerr("got it?")
            return
        
    def process_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
            self.detect_pink(cv_image)
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        

    def publish_laser(self, data):
        self.scan_array = data.ranges
        self.angle_inc = data.angle_increment
        self.angle_min = data.angle_min
            
if __name__ == "__main__":
    rospy.init_node("image_processor")
    # getDepth()
    rospy.spin()