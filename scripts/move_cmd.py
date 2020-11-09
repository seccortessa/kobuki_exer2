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

        #CAMBIO DE COORDENADA

        self.theta = -np.pi
        self.R = np.array([(np.cos(self.theta), -np.sin(self.theta),0,-0.2),(np.sin(self.theta),np.cos(self.theta),0,0),(0,0,1,0),(0,0,0,1)])
        self.R1 = np.linalg.inv(self.R)

        #ATRIBUTOS PARA PROGRAMA EN PYTHON
        self.angle_meas = 0.0
        self.posx_meas = 0.0
        self.posy_meas = 0.0

        self.angle_meas2 = 0.0
        self.posx_meas2 = 0.0
        self.posy_meas2 = 0.0
        
        self.angle_to_py = 45
        self.posx_to_py = 0.0
        self.posy_to_py = 0.0

        self.error_angle = 1
        self.error_linearx = 1
        self.error_lineary = 1
        self.error_hip = 1

        self.coord_act = False
        self.cont = 1

        self.target = np.array([(-3.5),(0),(0),(1)])  # coordenadas de meta en el marco de ref del mundo
        self.angle_init = -np.pi


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

        #################################PROGRAMA PRINCIPAL#################################################3------------------------------
        
        while True:
            if self.coord_act:
                if self.cont == 1:
                    
                    self.linear_control([-3.5,0])
                    self.angle_control(-np.pi/2)
                    self.linear_control([-3.5,3.5])
                    self.angle_control(-np.pi)
                    self.linear_control([1.5,3.5])
                    self.angle_control(np.pi/2)
                    self.linear_control([1.5,-1.5])
                    self.angle_control(np.pi)
                    self.linear_control([3.5,-1.5])
                    self.angle_control(np.pi/2)
                    self.linear_control([3.5,-8])
                    self.angle_control(0)
                    self.linear_control([-2.5,-8])
                    self.angle_control(-np.pi/2)
                    self.linear_control([-2.5,-5.5])
                    self.angle_control(-np.pi)
                    self.linear_control([1.5,-5.5])
                    self.angle_control(-np.pi/2)
                    self.linear_control([1.5,-3.5])
                    self.angle_control(0)
                    self.linear_control([-1,-3.5])
                    ''' self.ctrl_diag(self.target)
                    self.ctrl_diag(np.array([(-3.5),(3.5),(0),(1)]))
                    self.ctrl_diag(np.array([(1.5),(3.5),(0),(1)]))
                    self.ctrl_diag(np.array([(1.5),(-1.5),(0),(1)]))
                    self.ctrl_diag(np.array([(2),(1),(0),(1)]))
                    self.ctrl_diag(np.array([(2),(1),(0),(1)]))
                    self.ctrl_diag(np.array([(2),(1),(0),(1)])) '''
                    self.cont = 0
                    print(self.angle_meas2)

            
        
        



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
        #self.angle_to_py = config.angle
        self.posx_to_py = config.positionx
        self.posy_to_py = config.positiony
        return config

    #ENCENDER EL MOTOR
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
            self.tf()
            #print((180/np.pi)*self.angle_meas2)
            #self.R = np.array([(np.cos(self.angle_meas2), -np.sin(self.angle_meas2),0,-0.2),(np.sin(self.angle_meas2),np.cos(self.angle_meas2),0,0),(0,0,1,0),(0,0,0,1)])
            #self.R1 = np.linalg.inv(self.R)




            
 
    def angle_control(self,ang):
        
        self.error_angle = (ang)-self.angle_meas
        while np.abs(self.error_angle) >= 0.0001:
            
            control_signal = 2*self.error_angle

            if control_signal >= np.pi/6:
                control_signal = np.pi/6
            if control_signal <= (-1)*np.pi/6:
                control_signal = (-1)*np.pi/6
                
            self.vel.angular.z = control_signal
            self.pubvel.publish(self.vel)
            self.error_angle = (ang)-self.angle_meas
        self.vel.angular.z = 0.0
        self.pubvel.publish(self.vel)
            

    def linear_control(self,target):
        #self.error_linearx = (dist) - self.posx_meas
        dist = np.sqrt(np.power((self.posx_meas2-target[0]),2)+np.power((self.posy_meas2-target[1]),2))
        #la distancia ente el punto donde estoy y el punto destino es el error
        while dist >= 0.07:
            
            control_signal = 1 * dist
            
            if control_signal >= 0.3:
                control_signal = 0.3 
            if control_signal <= -0.3:
                control_signal = -0.3 

            self.vel.linear.x = control_signal
            self.pubvel.publish(self.vel)
            dist = np.sqrt(np.power((self.posx_meas2-target[0]),2)+np.power((self.posy_meas2-target[1]),2))
        self.vel.linear.x = 0.0
        self.pubvel.publish(self.vel)
            


    ''' def linear_controly(self,yval):
        self.error_lineary = (yval) - (np.abs(self.posy_meas))
        while np.abs(self.error_lineary) >= 0.01:
            #error = self.error_lineary
            control_signal = 1 * self.error_lineary
            
            if control_signal >= 0.3:
                control_signal = 0.3 
            if control_signal <= -0.3:
                control_signal = -0.3 

            self.vel.linear.x = control_signal
            self.pubvel.publish(self.vel)
            self.error_lineary = (yval) - (np.abs(self.posy_meas)) '''

    def tf(self):
        pose_R = np.array([(self.posx_meas),(self.posy_meas),(0),(1)])
        pose_L = np.matmul(self.R1,pose_R)
        self.posx_meas2 = pose_L[0]
        self.posy_meas2 = pose_L[1]
        a = self.angle_meas + self.theta
        if a > np.pi:
            a = a - 2*np.pi
        if a < -np.pi:
            a = a + 2*np.pi
        self.angle_meas2 = a

        self.coord_act = True



    def tf_to_robot(self,v,t):
        self.R = np.array([(np.cos(t), -np.sin(t),0,self.posx_meas2),(np.sin(t),np.cos(t),0,self.posy_meas2),(0,0,1,0),(0,0,0,1)])
        R1 = np.linalg.inv(self.R)
        b = np.matmul(R1,v)
        return b

    def tf_to_world(self,v):
        b = np.matmul(self.R1,v)
        return b    


    def ctrl_diag(self,target):
        
        print("anglemeas2 is: ",self.angle_meas2)
        print("pos x 2 is: ",self.posx_meas2)
        print("pos y 2 is: ",self.posy_meas2)
        target_robot = self.tf_to_robot(target,self.angle_meas2)
        print("target robot is",target_robot)
        angulo2 = np.arctan2(target_robot[1],target_robot[0])
        print("angulo entre punto y target: ",(180/np.pi)*angulo2)
        
        self.angle_control(angulo2)
        self.linear_control(target)
