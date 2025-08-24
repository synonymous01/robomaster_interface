#!/usr/bin/env python3
from ultralytics import YOLO
from sensor_msgs.msg import Image
import ros_numpy
import rospy
import time

detection_model = YOLO("/home/jetson/catkin_ws/src/robomaster_interface/src/best90.pt")
rospy.init_node("ultralytics")
det_image_pub = rospy.Publisher("/ultralytics/detection/image", Image, queue_size=5)
time.sleep(1)

def callback(data):
    array = ros_numpy.numpify(data)
    print("image received")
    det_result =  detection_model(array)
    det_annotated = det_result[0].plot(show=False)
    to_pub = ros_numpy.msgify(Image, det_annotated, encoding="rgb8")
    to_pub.header.frame_id = 'map'
    det_image_pub.publish(to_pub)

rospy.Subscriber("/camera_out/image", Image, callback)

while True:
    rospy.spin()