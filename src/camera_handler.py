#!/usr/bin/env python3

import time
import numpy as np
from ultralytics import YOLO
import ros_numpy
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# from tf.transformations import quaternion_from_euler
# import tf2_ros
# import geometry_msgs.msg

ANGLE_PER_PIXEL = 0.0703125

class getDepth:
    def __init__(self):
        self.robot_name = rospy.get_param('~robot_number')
        self.robo_pub = rospy.Publisher('/{}/enemy_angle'.format(self.robot_name), Float32, queue_size=5)
        self.detection_model = YOLO("/home/jetson/catkin_ws/src/robomaster_interface/src/best90.pt")
        time.sleep(1)
        rospy.Subscriber('/{}/camera/color/image_raw'.format(self.robot_name), Image, self.detect_robot)
    def detect_robot(self, data):
        array = ros_numpy.numpify(data)
        if self.robo_pub.get_num_connections():
            det_result = self.detection_model(array)
            
            print(f"{len(det_result[0])} robots detected!")
            x, y, w, h = det_result[0].boxes.xywh[0]

            midx = x + (w//2)
            midy = y + (h//2)
            angle = (midx - 640) * -ANGLE_PER_PIXEL
            rads = angle * (np.pi / 180)
            self.robo_pub.publish(Float32(rads))
            

        
if __name__ == "__main__":    
    rospy.init_node("camera_detector")
    time.sleep(1)
    getDepth()
    rospy.spin()
