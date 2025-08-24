#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Float32

class robot_publisher:
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_number")
        self.window = int(rospy.get_param("~angle_window"))
        rospy.Subscriber('/{}/scan'.format(self.robot_name), LaserScan, self.publish_laser)
        rospy.Subscriber('/{}/enemy_angle'.format(self.robot_name), Float32, self.post_pose)
        self.scan_array = None
        self.angle_inc = None
        self.angle_min = None

    def publish_laser(self, data):
        self.scan_array = data.ranges
        self.angle_inc = data.angle_increment
        self.angle_min = data.angle_min

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
    
    def post_pose(self, angle_data):
        rospy.logdebug_once("angle received")
        rads = angle_data.data
        if self.angle_inc is not None and self.angle_min is not None:
            index = int((rads - self.angle_min) / self.angle_inc)
            rospy.logdebug("index becomes: {}".format(index))
            dist_window = self.scan_array[index - self.window:index + self.window]
            distance_found = min(dist_window)
            rads = -1 * rads

            if distance_found == 'inf':
                self.sendingTransform(0,0)
                rospy.logdebug("no distance found!")
            else:
                y_displacement = distance_found * np.sin(rads)
                x_displacement = distance_found * np.cos(rads)

                rospy.logdebug("{} y, {} x found.".format(y_displacement, x_displacement))
                self.sendingTransform(x_displacement, y_displacement)


if __name__ == "__main__":
    rospy.init_node("lidar_handler")
    robot_publisher()
    rospy.spin()




