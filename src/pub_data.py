#!/usr/bin/env python3
from robomaster import robot
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import numpy as np


def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


class rover:
    def __init__(self):
        self.imu_pub = rospy.Publisher('imu_data', Imu, queue_size=10)
        # self.pose_pub = rospy.Publisher('pose_data', Point, queue_size=10)
        # self.angle_pub = rospy.Publisher('angle_data', )
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        rospy.Subscriber('cmd_vel', Twist, self.send_velocities)
        rospy.init_node('robomaster', anonymous=True)
        self.robot_name = rospy.get_param('~robot_number')
        self.robo = robot.Robot()
        try:
                self.robo.initialize(conn_type="rndis")
                print("robot initialized")
        except:
                print("robot cant be initialized. check hardware connections")
        self.chass = self.robo.chassis
        self.chass.sub_imu(10, self.update_imu)
        self.chass.sub_position(0, 5, self.update_pose)
        self.chass.sub_attitude(5, self.update_angles)
        self.roll=0
        self.pitch=0
        self.yaw=0
        self.vx=0
        self.vy=0
        self.vz=0
        self.wx=0
        self.wy=0
        self.wz=0
        self.orientation = Quaternion()
        self.current_time=rospy.Time.now()
        self.previous_time=rospy.Time.now()


    def update_imu(self, imu_info):
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_info
        # acc_z= 9.8
        sending = Imu()
        sending.header.frame_id = '{}_imu'.format(self.robot_name)
        sending.header.stamp = rospy.Time.now()


        # converting left hand coords of robomaster to standard ROS right hand coords
        acc_x = acc_x
        acc_y = -1 * acc_y
        acc_z = acc_z

        gyro_x = 0.0
        gyro_y = 0.0
        gyro_z = gyro_z * -1* (np.pi/180)
        sending.linear_acceleration.x = acc_x
        sending.linear_acceleration.y = acc_y
        sending.linear_acceleration.z = acc_z
        sending.angular_velocity.x = gyro_x
        sending.angular_velocity.y = gyro_y
        sending.angular_velocity.z = gyro_z
        sending.orientation=self.orientation
        

        covarience=np.identity(3)
        covarience=covarience.flatten()

        sending.orientation_covariance=covarience
        sending.angular_velocity_covariance=covarience
        sending.linear_acceleration_covariance=covarience

        self.wx=gyro_x
        self.wy=gyro_y
        self.wz=gyro_z

        self.previous_time=self.current_time
        self.current_time=rospy.Time.now()

        dt=(self.current_time-self.previous_time).to_sec()

        self.vx=self.vx+acc_x*dt
        self.vy=self.vy+acc_y*dt
        self.vz=self.vz+acc_z*dt
  

        self.imu_pub.publish(sending)

    def update_pose(self, pose_info):
        x, y, z = pose_info
        pos = Point()
        pos.x = x
        pos.y = -1 * y
        pos.z = z


        angles = get_quaternion_from_euler(self.roll, self.pitch, self.yaw)
        self.orientation.x = angles[0]
        self.orientation.y = angles[1]
        self.orientation.z = angles[2]
        self.orientation.w = angles[3]

        cov= 0.1*np.identity(6)
        cov=cov.flatten()



        Robot_pose=Pose()
        Robot_pose.position=pos
        Robot_pose.orientation=self.orientation

        twist=Twist()
        twist.linear.x=self.vx
        twist.linear.y=self.vy
        twist.linear.z=0

        twist.angular.x=0
        twist.angular.y=0
        twist.angular.z=self.wz



        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "{}_odom".format(self.robot_name)

        odom.child_frame_id = "{}_base_link".format(self.robot_name)

        odom.pose.pose=Robot_pose
        odom.pose.covariance=cov


        odom.twist.twist=twist
        odom.twist.covariance=cov


        self.odom_pub.publish(odom)
        






    def update_angles(self, angle_info):
        y, p, r = angle_info
        self.yaw = y * -1 * (np.pi/ 180)
        self.pitch = 0.0 #p * (np.pi/180)
        self.roll = 0.0 #r * (np.pi/180)

    def send_velocities(self, data):
        v = data.linear.x
        omega = data.angular.z
        omega = omega * (180 / np.pi)
        self.chass.drive_speed(x=v, y=0, z=omega, timeout=5)

    def close_robot(self):
        print("Closing RoboMaster robot.")
        self.robo.close()
#    def press(self, key):
#            if key=='w':
#                    self.chass.drive_speed(x=0.3, y=0, z=0, timeout=5)
#            if key == 's':
#                    self.chass.drive_speed(x=-0.3, y=0, z=0, timeout=5)
#            if key == 'a':
#                    self.chass.drive_speed(x=0, y=0, z=-45, timeout=5)
#            if key == 'd':
#                    self.chass.drive_speed(x=0, y=0, z=45, timeout=5)
#            if key == 'b':
#                    self.chass.drive_speed(x=0, y=0, z=0, timeout=5)
#                    self.robo.close()
#                    
        
        
  #  def release(self,key):
 #       self.chass.drive_speed(x=0, y=0, z=0, timeout=5)



try:
    robo = rover()
    rospy.on_shutdown(robo.close_robot)
except rospy.ROSInterruptException:
    robo.robo.close()


    






    
