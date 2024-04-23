#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
from math import floor
import tf2_ros

n_cols = 8

def coords_to_sector(x_coord, y_coord, meter_per_square_length):
    x = floor(x_coord/meter_per_square_length)
    y = floor(y_coord/meter_per_square_length)
    return (x + y * n_cols) + 1

rospy.init_node('xf_publisher', anonymous=True)
meter_per_square_length = int(rospy.get_param('~meter_per_square_length'))
robot_name = rospy.get_param('~robot_number')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rate = rospy.Rate(10)
publisher = rospy.Publisher('sectors', Int32MultiArray, queue_size=1)

while not rospy.is_shutdown():
    sending = Int32MultiArray()

    sectors = [-1, 1, 3, 5]
    for i in range(4):
        if i == 0:
            try:
                trans = tfBuffer.lookup_transform('world', '{}_estimated_enemy', rospy.Time())
            except:
                rate.sleep()
                continue
        else:
            try:
                trans = tfBuffer.lookup_transform('world', 'robot{}_odom_combined'.format(i), rospy.Time())
            except:
                rate.sleep()
                continue

        sectors[i] = coords_to_sector(trans.transform.translation.x, trans.transform.translation.y, meter_per_square_length)
    rospy.loginfo("sectors sent: {}".format(sectors))
    sending.data = sectors
    publisher.publish(sending)