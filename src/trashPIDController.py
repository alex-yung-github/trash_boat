#!/usr/bin/python3
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
import torch
import pandas
from pymavlink import mavutil

yolov5_path: str = "src/trash_boat/src/YOLOv5-Trash-Classifier"
model = torch.hub.load(yolov5_path, "custom", path=f"{yolov5_path}/weights/trash_classifier_v1.pt", source="local") 

class processor:
    def __init__(self):

        rospy.init_node("processor", anonymous = True) 
        #anonymous prevents the error of two nodes same name
        self.vel_pub = rospy.Publisher('/user/velocity', Image, 
                                        queue_size=10)
        self.start_sub = rospy.Subscriber('/user/start', move, self.mainProgram)
        self.start_sub = rospy.Subscriber('/user/stop', move, self.mainProgram)
        self.state_sub = rospy.Subscriber("boat/state", Twist, 
                                        self.state_callback)
        self.video_sub = rospy.Subscriber("/pylon_camera_node/image_raw", Image, self.runProgram)

        self.boatHullLocation = [320, 640]
        self.bridge = CvBridge()
        self.cameraOn = False
        self.runOn = False

        #Hyperparameters, change as needed:
        self.P = 0.006
        self.I = 0.000007
        self.D = 0.0004
        self.leftThruster = 5
        self.rightThruster = 3

        # self.the_connection = mavutil.mavlink_connection("/dev/ttyACM0") # if on jetson I think
        # self.the_connection.wait_heartbeat() # might be wait_heartbeat(the_connection)
        # print("Heartbeat from system (system %u component %u)" % (self.the_connection.target_system, self.the_connection.target_component))
        # print('finished initing')
        
        # context = zmq.Context()
        # self.socket = context.socket(zmq.REP)
        # self.socket.bind("tcp://*:5555")
    
    def testDrive(self):
        #now you can get and send messages I think
        pwmVal =1100 # something to send to propeller
        #wait_heartbeat(the_connection)
        msg1 = self.the_connection.mav.command_long_encode(
            1,0, # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,0, # command, confirmation
            5,pwmVal,0,0,0,0,0 # servo num, pwm val, zeros...
        )
        self.the_connection.mav.send(msg1)

        self.the_connection.armed   = True
        time.sleep(.2)

        msg1 = self.the_connection.mav.command_long_encode(
            1,0, # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,0,
            5,1500,0,0,0,0,0
        )
        self.the_connection.mav.send(msg1)
        print("sent value")

        
    def draw_bbox(self, image: np.ndarray, bbox_list: list) -> np.ndarray:
      for xmin, ymin, xmax, ymax, conf, _, label in bbox_list:
            r_xmin, r_ymin, r_xmax, r_ymax = [round(val) for val in (xmin, ymin, xmax, ymax)]
            bbox_color: tuple = tuple(map(int, np.random.choice(range(256), size=3))) # random color

            cv2.rectangle(image, (r_xmin, r_ymin), (r_xmax, r_ymax), bbox_color, 2)
            cv2.putText(image, f"{label}: {round(conf, 4) * 100}%", (r_xmin, r_ymin - 10), 2, 0.7, bbox_color, 1)
      return image

    def publish_velocity(self, xData, yData):
        temp = Twist()
        temp.linear.x = xData
        temp.linear.y = yData
        temp.linear.z = 0

        self.vel_pub.publish(temp)

    def getDistances(self, x1, x2):
        newDist= []
        v1 = x2[0] - x1[0]
        v2 = x2[1] - x1[1]
        newDist.append(v1)
        newDist.append(v2)
        return newDist

    def image_callback(self, data):
        self.cameraOn = True
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.df: pandas.DataFrame = model(cv_image).pandas().xyxy[0] # run the image through the model (make an inference)
        self.bbox_list: list = [tuple(self.df.loc[row_index]) for row_index in range(len(self.df.loc[:]))] # NOTE: [(xmin, ymin, xmax, ymax, conf, class_id, label),]
        mod_image = self.draw_bbox(cv_image, self.bbox_list)
        mod_image = cv2.resize(mod_image, (1200, 900))
        cv2.imshow("twitch.tv/xFishy_z", mod_image)
        cv2.waitKey(1)

    def state_callback(self, data):
        print(data)


    def runProgram(self, data):
        self.image_callback(data)
        if(self.runOn == True and self.cameraOn == True and len(self.bbox_list) > 0):
            print("in program")
            sortedItems = sorted(self.bbox_list,key=lambda t: t[2])
            closestItemLocation = list(sortedItems[0])
            print(closestItemLocation)
            print("boat hull", self.boatHullLocation)
            distances = self.getDistances(closestItemLocation, self.boatHullLocation)
            oldtime = time.time()
            newtime = time.time()
            xtotalError = 0
            ytotalError = 0
            if((abs(distances[0]) > 1 and abs(distances[1]) > 1) or len(self.bbox_list) > 0):
                sortedItems = sorted(self.bbox_list,key=lambda t: t[1])
                closestItemLocation = sortedItems[0]
                xError = distances[0]
                yError = distances[1]
                print("error", xError, yError)
                # goal = (360, 480) #set to center of camera in pixels
                if(xError < 0):
                    x_dir = -1
                else:
                    x_dir = 1
                if(yError > 0):
                    y_dir = 1
                else:
                    y_dir = -1

                xError = abs(xError)
                yError = abs(yError)
                newtime = time.time()
                deltaTime = newtime - oldtime
                xSpeed = (self.P * xError) + (self.I * xtotalError) + (self.D * deltaTime)
                ySpeed = (self.P * yError) + (self.I * ytotalError) + (self.D * deltaTime)
                xSpeed *= x_dir
                ySpeed *= y_dir
                pwmVal1 = 1500 + ySpeed# something to send to propeller
                pwmVal2 = 1500 + ySpeed
                if(x_dir == -1):
                    pwmVal1 = pwmVal1 + xSpeed
                else:
                    pwmVal2 = pwmVal2 + xSpeed
                #wait_heartbeat(the_connection)
                msg1 = self.the_connection.mav.command_long_encode(
                    1,0, # target system, target component
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,0, # command, confirmation
                    self.leftThruster,pwmVal1,0,0,0,0,0 # servo num, pwm val, zeros...
                )
                msg2 = self.the_connection.mav.command_long_encode(
                    1,0, # target system, target component
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,0, # command, confirmation
                    self.rightThruster,pwmVal2,0,0,0,0,0 # servo num, pwm val, zeros...
                )
                mavutil.send(msg1)
                mavutil.send(msg2)

                # self.publish_velocity(xSpeed, ySpeed)
                xtotalError += xError
                ytotalError += yError
                oldtime = newtime
                print("sped", xSpeed, ySpeed)     

    def mainProgram(self, data):
        if(data.xdirection == 1):
            print("gogogo")
            self.runOn = True
            # self.testDrive()
        elif(data.ydirection == 1):
            self.runOn = False
            sys.exit(0)


if __name__ == '__main__':
    try:
        tempProcessor = processor()
        rospy.spin() #make it so that ros will not shut down until user inputs Ctrl-C
    except rospy.ROSInterruptException:
        pass
