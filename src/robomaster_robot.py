#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64

class robomaster_robot:
    def __init__(self, no):
        name = "robot{}".format(no)
        rospy.Subscriber("{}/robot_pose_ekf/odom_combined".format(name), PoseWithCovarianceStamped, self.update_pose)
        self.pub = rospy.Publisher("{}/cmd_vel".format(name), Twist, queue_size=10)
        self.position = Point()
        self.yaw = Float64()

    def update_pose(self, data):
        self.position = data.pose.pose.position
        self.yaw.data, _, __ =  euler_from_quaternion(data.pose.pose.orientation)

    def send_velocities(self, v, omega):
        sending = Twist()
        sending.linear.x = v
        sending.angular.z = omega
        self.pub.publish(sending)

    
