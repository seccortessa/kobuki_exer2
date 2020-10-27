#!/usr/bin/python3

##### -------ESTOY EN LA RAMA DE CONTROL-------#####

import rospy
import numpy as np
import sys, select, termios, tty

from   std_msgs.msg                 import Float64
from   geometry_msgs.msg            import Twist
from   rospy.numpy_msg              import numpy_msg
from   kobuki_msgs.msg              import MotorPower
from   nav_msgs.msg                 import Odometry
from   tf.transformations           import euler_from_quaternion, quaternion_from_euler
from   dynamic_reconfigure.server   import Server
from   kobuki_exer2.cfg             import Kobuki_Exer2Config


class move_pub:

    def __init__(self):

        self.vel = Twist()
        self.mot = MotorPower()
        self.aa = 0.0
        self.param = 0.0
        key_timeout = rospy.get_param("~key_timeout", 0.0)
        if key_timeout == 0.0:
                key_timeout = None

        self.distance = None

        self.getParameters()

        if(self.distance is None):
            rospy.signal_shutdown("Parameters not declared")
        else:
            rospy.loginfo("Parameters found")

        def callback(data):
            self.aa = data.pose.pose.position.x
            err = self.param - self.aa
            
            self.vel.linear.x = 1 * err
            if self.vel.linear.x >= 0.3:
                self.vel.linear.x = 0.3 
            self.pubvel.publish(self.vel)
            """ self.vel.angular.z = control_signal
            self.pubvel.publish(self.vel)"""
            if not(err < 0.01):
                rospy.loginfo(self.vel.linear.x) 
            #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose.pose.orientation)

        
        self.nameTopic_mot = "/mobile_base/commands/motor_power"
        self.nameTopic_vel = "/mobile_base/commands/velocity"
        self.nameTopic_pose = "/odom"
        
        self.pubvel = rospy.Publisher(self.nameTopic_vel,numpy_msg(Twist),queue_size=10)
        self.pubmot = rospy.Publisher(self.nameTopic_mot,MotorPower,queue_size=10)

        self.subpose = rospy.Subscriber(self.nameTopic_pose, Odometry, callback)

        rate = rospy.Rate(10)

        self.mot.state = 1
        self.pubmot.publish(self.mot)
        rate.sleep()

        srv = Server(Kobuki_Exer2Config, self.DynConfCB)

    def getParameters(self):
        
            if rospy.has_param('~distance'):     self.distance = rospy.get_param('~distance')

            rospy.set_param('~distance', self.distance)

    def DynConfCB(self, config, level):
        self.param = config.distance
        return config





















'''         velocity = [0,1]

        
        self.vel.linear.x = velocity[0]
        self.vel.angular.z = velocity[1]
        print(self.aa)

        while True:
            self.pubvel.publish(self.vel)
            #print(self.vel.linear.x,self.vel.angular.z)
            print(self.aa)
            rate.sleep() '''
              
