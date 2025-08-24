#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TransformStamped, Vector3Stamped
from std_msgs.msg import Int16, Bool
from tf.transformations import quaternion_multiply, quaternion_from_euler, quaternion_inverse, euler_from_quaternion
import tf2_geometry_msgs
import tf2_ros
from math import floor
from prsbc import XRandSpan, create_si_pr_barrier_certificate_centralized

n_cols = 8
n_rows = 8
# meter_per_sector_length = 0.5

class handler:
    def __init__(self, no, init_x = 0, init_y = 0, confidence_level = 1, magnitude_limit = 0.1, safety_radius=0.5):
        # if no == 1:
        #     self.number = 0
        # elif no == 3:
        #     self.number = 1
        self.number = no
        self.name = "robot{}".format(no)
        rospy.Subscriber("/{}/robot_pose_ekf/odom_combined".format(self.name), PoseWithCovarianceStamped, self.update_pose)
        rospy.Subscriber("/{}/goal_sector".format(self.name), Int16, self.update_goal_sector)
        # rospy.Subscriber("/victory_signal", Bool, self.stop)
        self.pub = rospy.Publisher("/{}/cmd_vel".format(self.name), Twist, queue_size=10)
        self.goal_sector = -1
        self.init_x = init_x
        self.init_y = init_y
        self.meter_per_sector_length = rospy.get_param('~meter_per_sector_length')
        self.barrier_cert = create_si_pr_barrier_certificate_centralized(safety_radius=safety_radius, magnitude_limit=magnitude_limit, confidence_level=confidence_level)


    def update_goal_sector(self, data):
        self.goal_sector = data.data

    def update_pose(self, data):
        fixed_initial_rotation = quaternion_from_euler(0, 0, (np.pi / 2), 'rxyz')
        curr_orientation = data.pose.pose.orientation
        in_quaternions = [curr_orientation.x, curr_orientation.y, curr_orientation.z, curr_orientation.w]
        resultant = quaternion_multiply(fixed_initial_rotation, in_quaternions)
        broadcaster = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "{}_odom_combined".format(self.name)
        # t.transform.rotation = data.pose.pose.orientation
        t.transform.rotation.x = resultant[0]
        t.transform.rotation.y = resultant[1]
        t.transform.rotation.z = resultant[2]
        t.transform.rotation.w = resultant[3]

        _, __, yaw = euler_from_quaternion(resultant, 'rxyz')
        t.transform.translation.x = data.pose.pose.position.x * np.cos(yaw) - data.pose.pose.position.y * np.sin(yaw) + self.init_x
        t.transform.translation.y = data.pose.pose.position.x * np.sin(yaw) + data.pose.pose.position.y * np.cos(yaw) + self.init_y
        t.transform.translation.z = 0.0
        broadcaster.sendTransform(t)

    def send_velocities(self, vx, vy, omega = 0):
        sending = Twist()
        sending.linear.x = vx
        sending.linear.y = vy
        sending.angular.z = omega
        self.pub.publish(sending)

    def get_state_vector(self, poses=np.array([[0, 0, 0], [0, 0, 0]], dtype=np.float16)):
        tfbuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfbuffer)
        for i in range(1, 4):
            try:
                trans = tfbuffer.lookup_transform('world', 'robot{}_odom_combined'.format(i), rospy.Time(), rospy.Duration(0.2))
                poses[0, i-1] = trans.transform.translation.x # i - 1
                poses[1, i-1] = trans.transform.translation.y # i - 1 for experiments otherwise i
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("cant find transform")
                pass
        # try:
        #     trans = tfbuffer.lookup_transform('world', 'robot1_odom_combined', rospy.Time(), rospy.Duration(0.1))
        #     poses[0, 0] = trans.transform.translation.x
        #     poses[1, 0] = trans.transform.translation.y
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     pass

        # try:
        #     trans = tfbuffer.lookup_transform('world', 'robot3_odom_combined', rospy.Time(), rospy.Duration(0.1))
        #     poses[0, 1] = trans.transform.translation.x
        #     poses[1, 1] = trans.transform.translation.y 
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     pass
        return poses


    def send_to_sector(self):
        x_rand_span_x = 0 * np.random.randint(1, 2, (1, 3))
        x_rand_span_y = 0 * np.random.randint(1, 2, (1, 3))
        XRandSpan = np.concatenate((x_rand_span_x, x_rand_span_y))
        v_rand_span = 0.005 * np.ones((2, 3))

        def get_goal_pose():
            x = (((self.goal_sector - 1) % n_cols) + 0.5) * self.meter_per_sector_length
            y = ((floor((self.goal_sector - 1) / n_cols)) + 0.5) * self.meter_per_sector_length
            return x, y
        
        goal_x, goal_y = get_goal_pose()
        states = self.get_state_vector()
        # self.go_to_goal(x, y)
        tfbuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfbuffer)

        rate = rospy.Rate(10)
        try:
            trans = tfbuffer.lookup_transform('world', '{}_odom_combined'.format(self.name), rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        K = [1, 1]
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
            curr_rot = quaternion_inverse([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            _, __, yaw = euler_from_quaternion(curr_rot, 'rxyz')
            rospy.loginfo("for goal {}, currx: {}, curry: {}".format(self.goal_sector, curr_x, curr_y))
            err_x = goal_x - curr_x
            err_y = goal_y - curr_y
            u[0] = K[0] * err_x
            u[1] = K[1] * err_y


            # GENERALIZE THIS IF IT WORKS OK
            # vx = u[0] * np.cos(np.pi / -2) - u[1] * np.sin(np.pi / -2)
            # vy = u[1] * np.cos(np.pi / -2) + u[0] * np.sin(np.pi / -2)

            #generalized
            vx = u[0] * np.cos(yaw) - u[1] * np.sin(yaw)
            vy = u[1] * np.cos(yaw) + u[0] * np.sin(yaw)
            dx = [vx, vy]
            vels = np.zeros((2, 3))
            vels[:, self.number] = dx
            states = self.get_state_vector(states)
            rospy.logerr("states: {}".format(states))
            if states.all():
                dx_safe = self.barrier_cert(vels, states, XRandSpan, v_rand_span)
                rospy.loginfo("whoops! using safe velocities: {}".format(dx_safe))

                vx_safe = dx_safe[0, self.number] 
                vy_safe = dx_safe[1, self.number]
            else:
                vx_safe = vx
                vy_safe = vy
                rospy.loginfo("being notty: {}, {}".format(vx_safe, vy_safe))
            if abs(vx_safe) > v_max:
                vx_safe = np.sign(vx_safe) * v_max
            if abs(vy_safe) > v_max:
                vy_safe = np.sign(vy_safe) * v_max



            # self.send_velocities(vx, vy)
            self.send_velocities(vx_safe, vy_safe)
            goal_reached = (0.25 > np.linalg.norm(np.array([goal_x, goal_y]) - np.array([curr_x, curr_y])))

rospy.init_node('handler')
print('testing!')
robot_name = rospy.get_param('~robot_number')
init_x = rospy.get_param('~initial_x')
init_y = rospy.get_param('~initial_y')
#init_theta = rospy.get_param('~initial_theta')
conf_lvl = rospy.get_param('~confidence_level')
mag_lim = rospy.get_param('~magnitude_limit')
safety_rad = rospy.get_param('~safety_radius')
robot_number = int(robot_name[-1])
robot_handler = handler(robot_number, float(init_x), float(init_y), conf_lvl, mag_lim, safety_rad)

rospy.timer.sleep(10)

while not rospy.is_shutdown():
    if robot_handler.goal_sector != -1:
        robot_handler.send_to_sector()
    else:
        robot_handler.send_velocities(0, 0)