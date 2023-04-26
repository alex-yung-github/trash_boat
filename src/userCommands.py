#!/usr/bin/python3
import queue
from matplotlib.pyplot import yticks
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty  
from rosgraph_msgs.msg import Clock
import numpy as np
import cv2
import pygame
import sys
import time
import rospy
from trash_boat.msg import move



class User:

    def __init__(self):

        rospy.init_node("user", anonymous = True) 
        #anonymous prevents the error of two nodes same name
        self.start_pub = rospy.Publisher('/user/start', move, 
                                        queue_size=1)

        self.stop_pub = rospy.Publisher('/user/stop', move, 
                                        queue_size=1)

    def publish_run(self):
        run_msg = move()
        run_msg.xdirection = 1
        run_msg.ydirection = 0
        self.start_pub.publish(run_msg)

    def publish_stop(self):
        stop_msg = move()
        stop_msg.ydirection = 1
        stop_msg.xdirection = 0
        self.stop_pub.publish(stop_msg)
        

if __name__ == '__main__':
    try:
        teleop = User()
        pygame.init()
        pygame.display.set_mode((600, 600))

        while(True):
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:

                    #start/end
                    if(event.key == pygame.K_1):
                        print('1')
                        teleop.publish_run()
                    if(event.key == pygame.K_0):
                        print("0")
                        teleop.publish_stop() 


                if event.type == pygame.QUIT:
                    pygame.quit()
                    rospy.on_shutdown(lambda: print("done"))
                    sys.exit()


    except rospy.ROSInterruptException:
        pass
