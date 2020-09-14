import chessboard
import camera_record
import sys

# test
import rospy
from std_msgs.msg import String
from std_msgs.msg import *
from tf2_msgs.msg import TFMessage
import numpy as np
import csv
import tf
import geometry_msgs

def takler():
    pub  =rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) #10hz

    while not rospy.is_shutdown():
        hell_str = "hello world%s" % rospy.get_time()
        rospy.loginfo_once(hell_str)
        pub.publish(hell_str)
        rate.sleep()


def callback(msg):
    array= np.zeros((1, 3))
    x = msg.transforms[0].transform.translation.x
    y = msg.transforms[0].transform.translation.y
    z = msg.transforms[0].transform.translation.z
    # array[0] = x
    # array[1] = y
    # array[2] = z
    rospy.loginfo('x:{}, y:{}, z:{}'.format(x, y, z))


    # quaternion rotation
    rot_x = msg.transforms[0].transform.rotation.x
    rot_y = msg.transforms[0].transform.rotation.y
    rot_z = msg.transforms[0].transform.rotation.z
    rot_w = msg.transforms[0].transform.rotation.w
    #rospy.loginfo_once(('rot_x:{}, rot_y:{}, rot_z:{}, rot_w:{}'.format(rot_x, rot_y, rot_z, rot_w)))

    # quaternion to rotation


if __name__ == '__main__':
    # try:
    #     takler()
    #
    # except rospy.ROSInitException:
    #     pass

    rospy.init_node('position_optitrack')
   # pub = rospy.Subscriber('/tf', TFMessage, callback)
    data = rospy.wait_for_message('/tf', TFMessage, timeout=None)
    position = data.transforms[0].transform.translation
    print(position)





