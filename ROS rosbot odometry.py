#!/usr/bin/env python
import rospy

import numpy as np
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry



class PubSub():
    def __init__(self):

        #---------Subscribers-----------------#
        self.vel_subscriber = rospy.Subscriber(
            '/scan',
            LaserScan,
            self.scan_callback
        )

        self.odom_subscriber = rospy.Subscriber(
            '/odom',
            Odometry,
            self.odom_callback
        )
        #-----------Publishers---------------#
        self.vel_publisher = rospy.Publisher(
            '/cmd_vel',
            Twist,
            queue_size=1
        )
        self.cmd = Twist()
        #-----------variables----------------#

        self.laser_forward = 0

        self.x = 0
        self.y = 0
        self.z = 0
        self.quat = 0
        self.euler = 0
        self. yaw = 0

        #----------First State----------------#
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0

        #------------ettetianl----------------#
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        self.rate = rospy.Rate(10)

    def odom_callback(self, msg):

        # pa' ubicarse en un espacio tridimensional
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        self.z_quat = msg.pose.pose.orientation.z
        self.euler = tf.transformations.euler_from_quaternion((
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ))
        self.yaw = self.euler[2]

    def scan_callback(self, msg):
        self.laser_forward = msg.ranges[0]
    
    def control_rotation(self, sp_z_rad, P):
        sent = 0
        
        error_rad = sp_z_rad - self.yaw

        if sp_z_rad > self.yaw:
            if error_rad < np.pi:
                sent = 1
            else:
                sent = -1
        if sp_z_rad < self.yaw:
            if error_rad < np.pi:
                sent = -1
            else:
                sent = 1

        
        #publish
        self.cmd.angular.z = sent*abs(error_rad)*P
        self.error_r = error_rad
    def move_robot(self):
        rospy.sleep(1)
        tan = self.yaw
        #tan = np.pi/4
        flag = 0
        var = 0.1
        aux = 0
        while not self.ctrl_c:
            #avance lineal con aceleración constante
            if(flag == 0):
                rospy.sleep(0.1)
                self.control_rotation(tan,3)
                self.cmd.linear.x = var
                
                var += 0.05
                if(var >= 0.6):
                    flag = 1
                if(self.laser_forward <= 2):
                    self.cmd.linear.x = 0.1
                    flag = 2

            #velocidad lineal constante y sirve de trigger para entrar al siguiente estado  
            if(flag == 1):
                rospy.sleep(0.1)
                self.control_rotation(tan,3)
                self.cmd.linear.x = var
                if(self.laser_forward <= 5):
                    flag = 2

            #Desaceleración linal constante hasta 0.05 m/s
            if(flag == 2 and self.laser_forward < 5 and self.laser_forward > 0.5):
                rospy.sleep(0.1)
                self.control_rotation(tan,3)
                var -= 0.02
                if var <= 0.05:
                    var = 0.05
                self.cmd.linear.x = var
                if(self.laser_forward <= 0.56):
                    flag = 3

            # este flag espera un segundo para evitar el movimiento por inercia
            if(flag == 3 and self.laser_forward <= 0.56):
                print("entró a flag 3")
                rospy.sleep(1)   
                self.cmd.linear.x = 0
                aux = self.yaw
                flag = 4

            #giro de 180 grados
            if(flag == 4):
                
                print("está en flag 4")
                self.control_rotation(aux-np.pi,1)
                if (abs(self.error_r) < 0.05):
                    flag = 5

            # Se termina todo...
            if(flag == 5):
                rospy.sleep(0.1)#xd
                print("terminoooooo")
                self.cmd.linear.x = 0
                self.cmd.angular.z = 0

            print("----------state---------")
            print(f"-----------{flag}---------")
            print(f"yaw: {self.yaw}")
            print(f"Esta es la velocidad: {var}")
            print(f"Distancia: {self.laser_forward}")
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()

    def shutdownhook(self):
        self.ctrl_c = True


if __name__ == '__main__':
    rospy.init_node('proy', anonymous=True)
    rosbot_object = PubSub()

    try:
        rosbot_object.move_robot()
    except rospy.ROSInterruptException:
        pass
