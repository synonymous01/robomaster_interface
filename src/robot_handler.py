#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TransformStamped
from std_msgs.msg import Int16
import tf2_ros
from math import floor

n_cols = 4
n_rows = 4

class handler:
    def __init__(self, no, init_x = 0, init_y = 0):
        self.name = "robot{}".format(no)
        rospy.Subscriber("/{}/robot_pose_ekf/odom_combined".format(self.name), PoseWithCovarianceStamped, self.update_pose)
        rospy.Subscriber("/{}/goal_sector".format(self.name), Int16, self.update_goal_sector)
        self.pub = rospy.Publisher("/{}/cmd_vel".format(self.name), Twist, queue_size=10)
        self.goal_sector = -1
        self.init_x = init_x
        self.init_y = init_y

    def update_goal_sector(self, data):
        self.goal_sector = data.data

    def update_pose(self, data):
        broadcaster = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "{}_odom_combined".format(self.name)
        t.transform.translation.x = data.pose.pose.position.x + self.init_x
        t.transform.translation.y = data.pose.pose.position.y + self.init_y
        t.transform.translation.z = 0.0
        t.transform.rotation = data.pose.pose.orientation
        broadcaster.sendTransform(t)

    def send_velocities(self, vx, vy, omega = 0):
        sending = Twist()
        sending.linear.x = vx
        sending.linear.y = vy
        sending.angular.z = omega
        self.pub.publish(sending)


    def send_to_sector(self):
        def get_goal_pose():
            x = ((self.goal_sector - 1) % n_cols) + 0.5
            y = (floor((self.goal_sector - 1) / n_cols)) + 0.5
            return x, y
        
        goal_x, goal_y = get_goal_pose()
        # self.go_to_goal(x, y)
        tfbuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfbuffer)

        rate = rospy.Rate(10)
        try:
            trans = tfbuffer.lookup_transform('world', '{}_odom_combined'.format(self.name), rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        K = [1.5, 1.5]
        u = [0.0, 0.0]
        goal_reached = False
        v_max = 0.3
        while not goal_reached:
            goal_x, goal_y = get_goal_pose()
            try:
                trans = tfbuffer.lookup_transform("world", "{}_odom_combined".format(self.name), rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y
            rospy.loginfo("currx: {}, curry: {}".format(curr_x, curr_y))
            err_x = goal_x - curr_x
            err_y = goal_y - curr_y
            u[0] = -K[0] * err_x
            u[1] = -K[1] * err_y
            vx = u[0]
            vy = u[1]

            if abs(vx) > v_max:
                vx = np.sign(vx) * v_max
            if abs(vy) > v_max:
                vy = np.sign(vy) * v_max

            self.send_velocities(vx, vy)
            goal_reached = (0.1 > np.linalg.norm(np.array([goal_x, goal_y]) - np.array([curr_x, curr_y])))


rospy.init_node('handler')
print('testing!')
robot_name = rospy.get_param('~robot_number')
init_x = rospy.get_param('~initial_x')
init_y = rospy.get_param('~initial_y')
robot_number = int(robot_name[-1])
robot_handler = handler(robot_number, float(init_x), float(init_y))

while not rospy.is_shutdown():
    if robot_handler.goal_sector != -1:
        robot_handler.send_to_sector()
        # pass