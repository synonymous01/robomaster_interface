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

#camera height = 0.2m
#camera x disp = 0.095m
#camera y disp = 0.05m
ANGLE_PER_PIXEL = 0.1359375
START = 0
END = 0
class getDepth:
    def __init__(self):
        self.bridge = CvBridge()
        self.robot_name = rospy.get_param('~robot_number')
        rospy.Subscriber('/{}/camera/color/image_raw'.format(self.robot_name), Image, self.process_image)
        rospy.Subscriber('/{}/camera/aligned_depth_to_color/image_raw'.format(self.robot_name), Image, self.updateDepth)
        self.image = None

    def prediction(self, theta, d_estimated):
        pows = [[0,0], [1,0], [0, 1], [2, 0], [1, 1], [0, 2], [3, 0], [2, 1], [1, 2], [0, 3], [4, 0], [3, 1], [2, 2], [1, 3], [0, 4], [5, 0], [4, 1], [3, 2], [2, 3], [1, 4], [0, 5]]

        l1 = []
        for p in pows:
            a = theta ** p[0]
            b = d_estimated ** p[1]
            c = a * b
            l1.append(c)
        c = [0, 2.81194092e-01, 3.27773491e-2, -2.37541723e-02, -2.01187981e-01, 1.47128551, 8.95738238e-04, 7.87159333e-03, 8.06450400e-02, -9.69487579e-01, -1.71280640e-05, -1.08111164e-04, -1.39420007e-03, -1.90562191e-02, 2.86003927e-01, 1.27006467e-07, 7.93858571e-07, 2.25215635e-06, 1.45581234e-04, 1.85001219e-03, -3.12306605e-02]
        intercept = 0.19853936
        d_pred = np.dot(c, l1) + intercept
        return d_pred
    
    def detect_pink(self, img):
        # START = time.time()
        hsvFrame = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        red_lower = np.array([120, 178, 0], np.uint8) 
        red_upper = np.array([179, 255, 255], np.uint8)


        kernel = np.ones((5,5), "uint8")

        mask = cv2.inRange(hsvFrame, red_lower, red_upper)
        # cv2.imshow('mask', mask)
        mask =  cv2.dilate(mask, kernel)
        

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
        # img = cv2.rectangle(img, (x,y), (x+w, y+h), (255, 0, 0), 2)
        midx = x + (w//2)
        midy = y + (h // 2)
        angle = (midx - 320) * ANGLE_PER_PIXEL
        rads = angle * (np.pi / 180)
        predicted_depth = self.prediction(angle, self.image[midy, midx])
        # x_displacement = self.image[midy, midx] * np.sin(rads)
        # y_displacement = self.image[midy, midx] * np.cos(rads)
        x_displacement = predicted_depth * np.sin(rads)
        y_displacement = predicted_depth * np.cos(rads)
        self.sendingTransform(x_displacement / 1000, y_displacement / 1000)

        
        return img

    def sendingTransform(self, x_disp, y_disp):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "{}_camera_aligned_depth_to_color_frame".format(self.robot_name)
        t.child_frame_id = "{}_estimated_enemy".format(self.robot_name)
        t.transform.translation.x = y_disp - 0.25
        t.transform.translation.y = -x_disp
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        br.sendTransform(t)

    def updateDepth(self, data):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(data)
            self.image = cv_depth
            rospy.logerr("self.image updated.")
        except CvBridgeError as e:
            rospy.logerr("SELF.IMAGE FAILED TO UPDATE")
            rospy.logerr(e)
            rospy.logerr("got it?")
            return



    def process_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
            # converted = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # rospy.logwarn_once(data.encoding)
            # END = time.time()
            # rospy.logwarn("depth at {} is {}, took: {}s".format((data.width / 2, data.height / 2), cv_image[data.width / 2, data.height / 2], END - START))
            disp = self.detect_pink(cv_image)
            # cv2.imshow("image", disp)
            # cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(e)
            return

    def process_image2(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
            disp = self.detect_pink(cv_image)
        except CvBridgeError as e:
            rospy
        
if __name__ == "__main__":
    rospy.init_node("image_processor")
    getDepth()
    rospy.spin()







# # vid = cv2.VideoCapture(0 + cv2.CV_CAP_INTELPERC)

# while True:
#     ret, frame = vid.read()
#     cv2.imshow('frame', frame)
#     output_frame = detect_pink(frame)
#     cv2.imshow('detected output', output_frame)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# vid.release()

# cv2.destroyAllWindows()

# while True:
#     getDepth()
#     if cv2.waitKey(1) & 0xFF==ord('q'):
#         break

# cv2.destroyAllWindows()