#!/usr/bin/env python3

from this import d
import pygame
from djitellopy import tello
from re import L
import rospy
from Final_Project.msg import Flip 
from Final_Project.msg import State
from Final_Project.msg import Mode
from Final_Project.msg import Control
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
import sys
import time
import copy


class Movement:

    def __init__(self):

        rospy.init_node("movement", anonymous=True) #inits ros node

        rospy.Subscriber("tello/mode", Mode, self.mode_callback)
        rospy.Subscriber("tello/pose2D", Pose2D, self.pose2D_callback)

        self.flip_pub = rospy.Publisher("tello/flip", Flip, queue_size = 5)
        self.vel_pub = rospy.Publisher("tello/vel", Twist, queue_size = 5)
        self.land_pub = rospy.Publisher("tello/land", Empty, queue_size = 5)
        self.takeoff_pub = rospy.Publisher("tello/takeoff", Empty, queue_size = 5)
        self.control_pub = rospy.Publisher("tello/control", Control, queue_size = 5)

        self.control_msg = Control()
        self.flip_msg = Flip()
        self.vel_msg = Twist()
        self.mode_msg = Mode()
        self.pose2D_msg = Pose2D()
        self.camera_msg = Image()
        self.terminate = False
        self.velocity = 10
        self.prev_vel_msg = None
        self.mode = ""
        self.command = ""


    def set_defaults(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.z = 0
        self.flip_msg.direction = ""
        self.control_msg = ""

        self.target_detected = False
        self.delta_x = 0.5
        self.delta_y = 0.5
        self.delta_theta = 0.5


    def set_vel(self,x,y,z,theta):
        self.vel_msg.linear.x = x
        self.vel_msg.linear.y = y
        self.vel_msg.linear.z = z
        self.vel_msg.angular.z = theta
        if self.vel_msg != self.prev_vel_msg:
            self.vel_pub.publish(self.vel_msg)
            self.prev_vel_msg = copy.deepcopy(self.vel_msg)


    def mode_callback(self, mode):
        self.mode = mode.mode
        self.command = mode.command
    

    def pose2D_callback(self, pose):
        self.delta_x = pose.x
        self.delta_y = pose.y
        self.delta_theta = pose.theta
        self.target_detected = True


    def find_target(self):
        if self.target_detected == False:    
            self.set_vel(10,0,0,72)
            while(self.target_detected == False):
                pass
        else:
            pass


    def follow(self):
        set_distance = 100
        self.find_target()
        if self.target_detected == True:
            if self.delta_theta < 0.45:
                self.set_vel(0,0,0,self.velocity)
            elif self.delta_theta > 0.55:
                self.set_vel(0,0,0,-self.velocity)
            elif self.delta_y < 0.45:
                self.set_vel(0,0,self.velocity,0)
            elif self.delta_y > 0.55:
                self.set_vel(0,0,-self.velocity,0)
            else:
                pass
    

    def target(self):
        pass


    def tricks(self):
        if self.command == 'flip forward':
            self.flip_msg.direction = "f"
        elif self.command == 'flip backward':
            self.flip_msg.direction = "b"
        elif self.command == 'flip left':
            self.flip_msg.direction = "l"
        elif self.command == 'flip right':
            self.flip_msg.direction = "r"
        else:
            pass
        self.flip_pub.publish(self.flip_msg)
        self.set_defaults()
        
        if self.command == "bounce":
            self.set_vel(0,0,self.velocity,0)
            time.sleep(1)
            self.set_vel(0,0,-self.velocity,0)
            time.sleep(1)
            self.set_vel(0,0,self.velocity,0)
            time.sleep(1)
            self.set_vel(0,0,-self.velocity,0)
            time.sleep(1)

        if self.command == "freefall":
            self.set_vel(0,0,self.velocity,0)
            time.sleep(1)
            self.set_vel(0,0,0,0)
            self.control_msg = "emergency"
            self.control_pub.publish(self.control_msg)
            time.sleep(1)
            self.control_msg = ""
            self.control_pub.publish(self.control_msg)
            self.set_vel(0,0,self.velocity,0)
            time.sleep(1)
            self.set_defaults()

        if self.command == "wave":   #sinusoidal wave not hand wave
            self.set_vel(0,0,0,-90)
            time.sleep(1)
            self.set_vel(self.velocity,0,0,90)
            time.sleep(2)
            self.set_vel(self.velocity,0,0,-90)
            time.sleep(2)
            self.set_vel(self.velocity,0,0,90)
            time.sleep(2)
            self.set_vel(self.velocity,0,0,-90)
            time.sleep(2)
            self.set_vel(0,0,0,90)
            time.sleep(1)
            self.set_defaults()

        if self.command == "spiral":
            self.set_vel(30,0,10,90)
            time.sleep(5)
            self.set_vel(30,0,-10,90)
            time.sleep(5)
            self.set_defaults()

    def simple_movement(self):
        if self.command == 'takeoff':
            self.takeoff_pub.publish()
        elif self.command == 'land':
            self.land_pub.publish()
        elif self.command == 'forward':
            self.set_vel(self.velocity,0,0,0)
        elif self.command == 'backward':
            self.set_vel(-self.velocity,0,0,0)
        elif self.command == 'left':
            self.set_vel(0,-self.velocity,0,0)
        elif self.command == 'right':
            self.set_vel(0,self.velocity,0,0)
        elif self.command == 'up':
            self.set_vel(0,0,self.velocity,0)
        elif self.command == 'down':
            self.set_vel(0,0,-self.velocity,0)
        elif self.command == 'turn left':
            self.set_vel(0,0,0,-self.velocity)
        elif self.command == 'turn right':
            self.set_vel(0,0,0,self.velocity)
        elif self.command == 'turn 90 degrees left':
            self.set_vel(0,0,0,-45)
            time.sleep(2)
        elif self.command == 'turn 90 degrees right':
            self.set_vel(0,0,0,45)
            time.sleep(2)
        elif self.command == 'turn 180 degrees left':
            self.set_vel(0,0,0,-45)
            time.sleep(4)
        elif self.command == 'turn 180 degrees right':
            self.set_vel(0,0,0,45)
            time.sleep(4)
        elif self.command == 'turn 360 degrees left':
            self.set_vel(0,0,0,-45)
            time.sleep(8)
        elif self.command == 'turn 360 degrees right':
            self.set_vel(0,0,0,45)
            time.sleep(8)
        elif self.command == 'stop':
            self.set_vel(0,0,0,0)
        else:
            self.set_vel(0,0,0,0)


    def geometric_movement(self):
        if self.command == 'circle':
            self.set_vel(20,0,0,72)
            time.sleep(5)
            self.set_defaults()

        elif self.command == 'square':
            self.set_vel(-30,0,0,0)
            time.sleep(1)
            self.set_vel(0,30,0,0)
            time.sleep(1)
            self.set_vel(30,0,0,0)
            time.sleep(1)
            self.set_vel(0,-30,0,0)
            time.sleep(1)
            self.set_defaults()

        elif self.command == 'vertical square':
            self.set_vel(0,0,30,0)
            time.sleep(1)
            self.set_vel(-30,0,0,0)
            time.sleep(1)
            self.set_vel(0,0,-30,0)
            time.sleep(1)
            self.set_vel(30,0,0,0)
            time.sleep(1)
            self.set_defaults()

        elif self.command == 'triangle':
            self.set_vel(30,0,0,0)
            time.sleep(2)
            self.set_vel(0,0,0,60)
            time.sleep(2)
            self.set_vel(30,0,0,0)
            time.sleep(2)
            self.set_vel(0,0,0,60)
            time.sleep(2)
            self.set_vel(30,0,0,0)
            time.sleep(2)
            self.set_vel(0,0,0,60)
            time.sleep(2)
            self.set_defaults()
        else:
            pass
        
        
if __name__ == "__main__":
    movement = Movement()
    t0 = rospy.Time.now().to_sec()
    movement.set_defaults()
    while(not rospy.is_shutdown()):
        try:
            if movement.mode == "follow":
                movement.follow()
            elif movement.mode ==  "target":
                movement.target
            elif movement.mode ==  "find_target":
                movement.find_target()
            elif movement.mode ==  "tricks":
                movement.tricks()
            elif movement.mode ==  "simple_movement":
                movement.simple_movement()
            elif movement.mode ==  "geometric_movement":
                movement.geometric_movement()
            else:
                movement.set_vel(0,0,0,0)
            movement.set_defaults()
        except rospy.ROSInterruptException:
            pass
    pygame.quit()
