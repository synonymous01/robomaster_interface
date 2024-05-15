import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
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
        rospy.Subscriber('/camera/color/image_raw', Image, self.process_image)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.updateDepth)
        self.image = None
        self.robot_name = rospy.get_param('~robot_number')


    def detect_pink(self, img):
        # START = time.time()
        hsvFrame = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        red_lower = np.array([150, 120, 105], np.uint8) 
        red_upper = np.array([180, 255, 255], np.uint8)


        kernel = np.ones((5,5), "uint8")

        mask = cv2.inRange(hsvFrame, red_lower, red_upper)
        cv2.imshow('mask', mask)
        mask =  cv2.dilate(mask, kernel)
        

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]


        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > 400:
                x, y, w, h =cv2.boundingRect(contour)
                img = cv2.rectangle(img, (x,y), (x+w, y+h), (255, 0, 0), 2)
                midx = x + (w // 2)
                midy = y + (h // 2)
                # END = time.time()
                angle = (midx - 320) * ANGLE_PER_PIXEL
                rads = angle * (np.pi / 180)
                # rospy.logwarn("depth at {},{} and angle {} is: {} took {}s".format(midy, midx, angle, self.image[midy, midx], END - START))
                y_displacement = self.image[midy, midx] * np.sin(rads)
                x_displacement = self.image[midy, midx] * np.cos(rads)
                self.sendingTransform(x_displacement, y_displacement)
                # rospy.logwarn("x: {}, y: {}, h:{}".format(x_displacement, y_displacement, self.image[midy, midx]))
        # detected_output = cv2.bitwise_and(img, img, mask=mask)
        
        return img

    def sendingTransform(self, x_disp, y_disp):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "{}_camera_link".format(self.robot_name)
        t.child_frame_id = "{}_estimated_enemy".format(self.robot_name)
        t.transform.translation.x = x_disp
        t.transform.translation.y = y_disp
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0

        br.sendTransform(t)

    def updateDepth(self, data):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.image = cv_depth
        except CvBridgeError as e:
            rospy.logerr(e)
            return



    def process_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            # converted = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            rospy.logwarn_once(data.encoding)
            END = time.time()
            # rospy.logwarn("depth at {} is {}, took: {}s".format((data.width / 2, data.height / 2), cv_image[data.width / 2, data.height / 2], END - START))
            disp = self.detect_pink(cv_image)
            cv2.imshow("image", disp)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
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