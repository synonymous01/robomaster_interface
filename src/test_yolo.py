#!/usr/bin/env python
from ultralytics import YOLO
from sensor_msgs.msg import Image
import ros_numpy
import rospy
import time

detection_model = YOLO("best90.pt")
rospy.init_node("ultralytics")
time.sleep(1)

def callback(data):
    array = ros_numpy.numpify(data)
    det_result =  detection_model(array)
    det_annotated = det_result[0].plot(show=True)

rospy.Subscriber("/camera_out/image", Image, callback)

while True:
    rospy.spin()