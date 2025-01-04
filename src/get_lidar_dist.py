#!/usr/bin/env python
from math import floor, isnan
from turtle import distance
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import time
from tf.transformations import quaternion_from_euler
import tf2_ros
import geometry_msgs.msg

ANGLE_PER_PIXEL = 0.0703125
class getDepth:
    def __init__(self):
        self.tfbuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfbuffer)
        self.bridge = CvBridge()
        self.robot_name = rospy.get_param("~robot_number")
        rospy.Subscriber('/{}/camera/color/image_raw'.format(self.robot_name), Image, self.process_image)
        
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
        angle = (midx - 640) * -ANGLE_PER_PIXEL
        rads = angle * (np.pi / 180)

        index = int((rads - self.angle_min) / self.angle_inc)
        distance_found = self.scan_array[index]
        rads - -1 * rads
        
        if distance_found == 'inf':
            self.sendingTransform(0,0)
        else:
            y_displacement = distance_found * np.sin(rads)  * -1
            x_displacement = distance_found * np.cos(rads)

            self.sendingTransform(x_displacement, y_displacement)

        # rospy.loginfo(distance_found)
        

    def sendingTransform(self, x_disp, y_disp):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "{}_laser".format(self.robot_name)
        t.child_frame_id = "{}_estimated_enemy".format(self.robot_name)

        t.transform.translation.x = x_disp
        t.transform.translation.y = y_disp
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        br.sendTransform(t)


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
    getDepth()
    rospy.spin()