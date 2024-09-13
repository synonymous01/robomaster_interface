#!../venvs/robot_env/bin/python3.7
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TransformStamped, Vector3Stamped
from std_msgs.msg import Int16, Bool
from tf.transformations import quaternion_multiply, quaternion_from_euler, quaternion_inverse, euler_from_quaternion
import tf2_geometry_msgs
import tf2_ros
from math import floor

n_cols = 8
n_rows = 8
# meter_per_sector_length = 0.5

class handler:
    def __init__(self, no, init_x = 0, init_y = 0):
        self.name = "robot{}".format(no)
        rospy.Subscriber("/{}/robot_pose_ekf/odom_combined".format(self.name), PoseWithCovarianceStamped, self.update_pose)
        rospy.Subscriber("/{}/goal_sector".format(self.name), Int16, self.update_goal_sector)
        # rospy.Subscriber("/victory_signal", Bool, self.stop)
        self.pub = rospy.Publisher("/{}/cmd_vel".format(self.name), Twist, queue_size=10)
        self.goal_sector = -1
        self.init_x = init_x
        self.init_y = init_y
        self.meter_per_sector_length = rospy.get_param('~meter_per_sector_length')

    # def stop(self, data):
    #     if data.data:
    #         self.send_velocities(0, 0, 0)

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


    def send_to_sector(self):
        def get_goal_pose():
            x = (((self.goal_sector - 1) % n_cols) + 0.5) * self.meter_per_sector_length
            y = ((floor((self.goal_sector - 1) / n_cols)) + 0.5) * self.meter_per_sector_length
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
            # curr_rot = quaternion_inverse([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            rospy.loginfo("for goal {}, currx: {}, curry: {}".format(self.goal_sector, curr_x, curr_y))
            err_x = goal_x - curr_x
            err_y = goal_y - curr_y
            u[0] = K[0] * err_x
            u[1] = K[1] * err_y


            # GENERALIZE THIS IF IT WORKS OK
            vx = u[0] * np.cos(np.pi / -2) - u[1] * np.sin(np.pi / -2)
            vy = u[1] * np.cos(np.pi / -2) + u[0] * np.sin(np.pi / -2)

            # v = Vector3Stamped()
            # v.vector.x = vx
            # v.vector.y = vy
            # t = TransformStamped()
            # t.transform.rotation.x = curr_rot[0]
            # t.transform.rotation.y = curr_rot[1]
            # t.transform.rotation.z = curr_rot[2]
            # t.transform.rotation.w = curr_rot[3]
            # vt = tf2_geometry_msgs.do_transform_vector3(v, t)
            # vx = v.vector.x
            # vy = v.vector.y
            if abs(vx) > v_max:
                vx = np.sign(vx) * v_max
            if abs(vy) > v_max:
                vy = np.sign(vy) * v_max

            self.send_velocities(vx, vy)
            goal_reached = (0.25 > np.linalg.norm(np.array([goal_x, goal_y]) - np.array([curr_x, curr_y])))


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