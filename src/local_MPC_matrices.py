#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
from math import floor # type: ignore
import tf2_ros

n_cols = 8

def coords_to_sector(x_coord, y_coord, meter_per_square_length=0.45):
    x = floor(x_coord/meter_per_square_length)
    y = floor(y_coord/meter_per_square_length)
    return (x + y * n_cols) + 1

rospy.init_node('xf_publisher')
# METER_PER_SQUARE_LENGTH = int(rospy.get_param('~meter_per_square_length'))
robot_name = rospy.get_param('~robot_number')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rate = rospy.Rate(10)
publisher = rospy.Publisher('/{}/sectors'.format(robot_name), Int32MultiArray, queue_size=1)

while not rospy.is_shutdown():
    sending = Int32MultiArray()

    sectors = [-1, -1, -1, 1, 4, 7]
    #finding sectors for defenders
    for i in range(1,4,1):
        try:
            trans = tfBuffer.lookup_transform('world', 'robot{}_odom_combined'.format(i), rospy.Time())
            rospy.loginfo("Trans data recieved: ({}, {})".format(trans.transform.translation.x, trans.transform.translation.y))
        except:
            rospy.logerr("Exception occurred trans")
            continue

        sectors[i + 2] = coords_to_sector(trans.transform.translation.x, trans.transform.translation.y, 0.45)
    
    # finding sector for attacker using information from all defenders
    for i in range(1,4,1):
        try:
            trans2 = tfBuffer.lookup_transform('world', 'robot{}_estimated_enemy'.format(i), rospy.Time())
            rospy.loginfo("trans2 data recieved: ({}, {})".format(trans2.transform.translation.x, trans2.transform.translation.y))
        except:
            rospy.logerr("Exception occured getting trans2")
            continue
        possible_sector = coords_to_sector(trans2.transform.translation.x, trans2.transform.translation.y, 0.45)

        if possible_sector == sectors[i + 2]:
            sectors[i - 1] = -1
        else:
            sectors[i - 1] = possible_sector


    try:
        trans3 = tfBuffer.lookup_transform('world', 'robot0_odom_combined', rospy.Time())
        rospy.loginfo("trans3 data recieved: ({}, {})".format(trans3.transform.translation.x, trans3.transform.translation.y))
    except:
        rospy.logerr("Exception occured getting trans3")
        continue

    actual_sector = coords_to_sector(trans3.transform.translation.x, trans3.transform.translation.y)

    if actual_sector == sectors[3] or actual_sector == sectors[4] or actual_sector == sectors[5]:
        rospy.signal_shutdown("Attacker captured!!!!!!!!!!!!!!!!!!!")
    
    
    rospy.loginfo("sectors sent: {}".format(sectors))
    sending.data = sectors
    publisher.publish(sending)
    rate.sleep()