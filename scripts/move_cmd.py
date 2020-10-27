#!/usr/bin/python3

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


        #ATRIBUTOS PARA PUBLICADORES
        self.vel = Twist()
        self.mot = MotorPower()

        #ATRIBUTOS PARA PARAMETROS DESDE EL SERVER
        self.angle = None
        self.positionx = None
        self.positiony = None

        #ATRIBUTOS PARA PROGRAMA EN PYTHON
        self.angle_meas = 0.0
        self.posx_meas = 0.0
        self.posy_meas = 0.0
        
        self.angle_to_py = 0.0
        self.posx_to_py = 0.0
        self.posy_to_py = 0.0

        self.error_angle = 1
        self.error_linearx = 1
        self.error_lineary = 1


        key_timeout = rospy.get_param("~key_timeout", 0.0)
        if key_timeout == 0.0:
                key_timeout = None

        

        self.getParameters()

        if((self.angle is None) or (self.positionx is None) or (self.positiony is None)):
            rospy.signal_shutdown("Parameters not declared")
        else:
            rospy.loginfo("Parameters found")

        
        

        #DEFINICION DE LOS PUBLICADORES
        self.nameTopic_mot = "/mobile_base/commands/motor_power"
        self.nameTopic_vel = "/mobile_base/commands/velocity"
        self.nameTopic_pose = "/odom"
        
        self.pubvel = rospy.Publisher(self.nameTopic_vel,numpy_msg(Twist),queue_size=10)
        self.pubmot = rospy.Publisher(self.nameTopic_mot,MotorPower,queue_size=10)


        #SE DEFINE EL SUSCRIPTOR PARA RECIBIR LA POSE (ODOMETRÍA)
        self.subpose = rospy.Subscriber(self.nameTopic_pose, Odometry, self.callback)


        #INICIAMOS SERVIDOR PARA PARAMETROS
        srv = Server(Kobuki_Exer2Config, self.DynConfCB)


        #INICIAMOS EL MOTOR
        self.turn_on_mot()


        #control para el angulo
        ''' while True:
            self.angle_control(self.angle_to_py*(np.pi/180))
            self.linear_controlx(self.posx_to_py)
            #self.linear_controly(self.posy_to_py) '''

        # for i in range(1):
        while True:
            if self.angle_to_py == 45:
                self.angle_control(np.pi/2)
                self.linear_controly(1)
                self.angle_control(0)
                self.linear_controlx(1)

        
        



    def getParameters(self):
        
            if rospy.has_param('~angle'):
                self.angle = rospy.get_param('~angle')

            if rospy.has_param('~positionx'):
                self.positionx = rospy.get_param('~positionx')

            if rospy.has_param('~positiony'):
                self.positiony = rospy.get_param('~positiony')

            rospy.set_param('~angle', self.angle)
            rospy.set_param('~positionx', self.positionx)
            rospy.set_param('~positiony', self.positiony)



    #CALLBACK PARA ACTUALIZACIÓN DE PARÁMETROS
    def DynConfCB(self, config, level):
        self.angle_to_py = config.angle
        self.posx_to_py = config.positionx
        self.posy_to_py = config.positiony
        print("se actualizó")
        return config

    def turn_on_mot(self):

        rate = rospy.Rate(30)
        self.mot.state = 1
        self.pubmot.publish(self.mot)
        rate.sleep()

    #---CALLBACK DISPARADO CUANDO SE RECIBE UN MENSAJE DEL TÓPICO SUSCRITO
    def callback(self,data):
    
            self.angle_meas = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2]
            self.posx_meas = data.pose.pose.position.x
            self.posy_meas = data.pose.pose.position.y

            #self.error_angle = self.angle_to_py - self.angle_meas
            #self.error_linearx = self.posx_to_py - self.posx_meas
            #self.error_lineary = self.posy_to_py - self.posy_meas
              
    def angle_control(self,ang):
        self.error_angle = (ang)-self.angle_meas
        while np.abs(self.error_angle) >= 0.01:
            #self.error_angle = (ang)-self.angle_meas
            control_signal = 4*(2*0.035/0.23)*self.error_angle

            if control_signal >= np.pi/6:
                control_signal = np.pi/6
            if control_signal <= (-1)*np.pi/6:
                control_signal = (-1)*np.pi/6
                
            self.vel.angular.z = control_signal
            self.pubvel.publish(self.vel)
            self.error_angle = (ang)-self.angle_meas

    def linear_controlx(self,xval):
        self.error_linearx = (xval) - (self.posx_meas)
        while np.abs(self.error_linearx) >= 0.01:
            #error = self.error_linearx
            control_signal = 1 * self.error_linearx
            
            if control_signal >= 0.3:
                control_signal = 0.3 
            if control_signal <= -0.3:
                control_signal = -0.3 

            self.vel.linear.x = control_signal
            self.pubvel.publish(self.vel)
            self.error_linearx = (xval) - (self.posx_meas)
            


    def linear_controly(self,yval):
        self.error_lineary = (yval) - (self.posy_meas)
        while np.abs(self.error_lineary) >= 0.01:
            #error = self.error_lineary
            control_signal = 1 * self.error_lineary
            
            if control_signal >= 0.3:
                control_signal = 0.3 
            if control_signal <= -0.3:
                control_signal = -0.3 

            self.vel.linear.x = control_signal
            self.pubvel.publish(self.vel)
            self.error_lineary = (yval) - (self.posy_meas)