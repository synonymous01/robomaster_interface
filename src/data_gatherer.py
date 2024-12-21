#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CompressedImage
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
        self.image = None

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
        y_displacement = (self.image[midy, midx] * np.sin(rads)) / -1000
        x_displacement = (self.image[midy, midx] * np.cos(rads)) / 1000
        displacement = self.image[midy, midx] / 1000

        try:
            trans = self.tfbuffer.lookup_transform('{}_odom_combined'.format(self.robot_name), 'robot0_odom_combined', rospy.Time(), rospy.Duration(0.2))
            actual_x = trans.transform.translation.x
            actual_y = trans.transform.translation.y
            field = [x_displacement, y_displacement, displacement, angle, actual_x, actual_y]
            self.writer.writerow(field)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("cant find transform")
            pass



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
            
if __name__ == "__main__":
    rospy.init_node("image_processor")
    getDepth()
    rospy.spin()