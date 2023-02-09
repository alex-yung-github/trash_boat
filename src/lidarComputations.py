#!/usr/bin/python3
from re import T
from turtle import pos

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty  
from rosgraph_msgs.msg import Clock
import numpy as np
from trash_boat.msg import state
from trash_boat.msg import move
from cv_bridge import CvBridge
import cv2
import time
import sys
import math
import csv
import coneDet



class driver:

    def __init__(self):

        rospy.init_node("driver", anonymous = True) 
        #anonymous prevents the error of two nodes same name
        self.cam_pub = rospy.Publisher('tello/cam_forward', Image, 
                                        queue_size=10)
        self.state_pub = rospy.Publisher('tello/state', state, 
                                            queue_size = 10)
        self.vel_sub = rospy.Subscriber("tello/cmd_vel", Twist, 
                                        self.velocity_callback)

        print("stuff initialized")

    def velocity_callback(self, data):
        linear_velocity = data.linear
        yaw_velocity = data.angular.z
        self.drone.send_rc_control(int(linear_velocity.x),
                              int(linear_velocity.y),
                              int(linear_velocity.z),
                              int(yaw_velocity))
    
    def takeoff_callback(self, data):
        self.drone.send_rc_control(0,0,0,0)
        print(self.drone.get_battery())
        self.drone.takeoff()
    
    def land_callback(self, data):
        self.drone.land()
        # self.drone.streamoff()
        self.drone.end()
        cv2.destroyAllWindows()
        sys.exit(0)


if __name__ == '__main__':
    try:
        bot = driver()
        while(not rospy.is_shutdown()):
            bot.publish_state()
            # bot.publish_video()
        rospy.on_shutdown(lambda: print("done"))
        bot = None
        sys.exit(0)
    except rospy.ROSInterruptException:
        pass