#!/usr/bin/python3

import rospy
from move_cmd import move_pub
from dynamic_reconfigure.server import Server
from kobuki_exer2.cfg import Kobuki_Exer2Config

# Init of program
if __name__ == '__main__':

    rospy.init_node('move', anonymous=True)

    rospy.loginfo("RyCSV__2020-2")

    move_pub()

    rospy.spin()