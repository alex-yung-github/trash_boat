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

yolov5_path: str = "src/trash_boat/src/YOLOv5-Trash-Classifier"
model = torch.hub.load(yolov5_path, "custom", path=f"{yolov5_path}/weights/trash_classifier_v1.pt", source="local") 

class processor:
    def __init__(self):

        rospy.init_node("processor", anonymous = True) 
        #anonymous prevents the error of two nodes same name
        self.vel_pub = rospy.Publisher('boat/velocity', Image, 
                                        queue_size=10)
        self.state_sub = rospy.Subscriber("boat/state", Twist, 
                                        self.state_callback)
        self.video_sub = rospy.Subscriber("/pylon_camera_node/image_raw", Image, self.image_callback)

        self.boatHullLocation = (320, 640)
        self.bridge = CvBridge()

        #Hyperparameters, change as needed:
        self.P = 0.06
        self.I = 0.0007
        self.D = 0.004
        print('finished initing')
        # context = zmq.Context()
        # self.socket = context.socket(zmq.REP)
        # self.socket.bind("tcp://*:5555")

    def getZMQ(self):
        while True:
            #  Wait for next request from client
            message = self.socket.recv()
            print("Received request: %s" % message)

            #  Do some 'work'
            time.sleep(1)

            #  Send reply back to client
            self.socket.send(b"World")
        
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

    def getDistances(x1, x2):
        newDist= []
        newDist[0] = x2[0] - x1[0]
        newDist[1] = x2[1] - x1[1]
        return newDist

    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        df: pandas.DataFrame = model(cv_image).pandas().xyxy[0] # run the image through the model (make an inference)
        bbox_list: list = [tuple(df.loc[row_index]) for row_index in range(len(df.loc[:]))] # NOTE: [(xmin, ymin, xmax, ymax, conf, class_id, label),]
        mod_image = self.draw_bbox(cv_image, bbox_list)
        cv2.imshow("twitch.tv/xFishy_z", mod_image)
        cv2.waitKey(1)

    def state_callback(self, data):
        print(data)

        

    # def runProgram(self):
    #     self.imageGatherer.streamOn()
    #     camera = self.imageGatherer.get_video_capture()
    #     items, frame = camera.read()
    #     if(len(items) > 0):
    #         sortedItems = sorted(items,key=lambda t: t[1])
    #         closestItemLocation = sortedItems[0]
    #         distances = self.getDistances(closestItemLocation, self.boatHullLocation)
    #         oldtime = time.time()
    #         newtime = time.time()
    #         xtotalError = 0
    #         ytotalError = 0
    #         while((abs(distances[0]) > 1 and abs(distances[1]) > 1) or len(items) > 0):
    #             items, frame = camera.read()
    #             sortedItems = sorted(items,key=lambda t: t[1])
    #             closestItemLocation = sortedItems[0]
    #             xError = distances[0]
    #             yError = distances[1]
    #             print("error", xError, yError)
    #             # goal = (360, 480) #set to center of camera in pixels
    #             if(xError < 0):
    #                 x_dir = -1
    #             else:
    #                 x_dir = 1
    #             if(yError > 0):
    #                 y_dir = 1
    #             else:
    #                 y_dir = -1

    #             xError = abs(xError)
    #             yError = abs(yError)
    #             newtime = time.time()
    #             deltaTime = newtime - oldtime
    #             xSpeed = (self.P * xError) + (self.I * xtotalError) + (self.D * deltaTime)
    #             ySpeed = (self.P * yError) + (self.I * ytotalError) + (self.D * deltaTime)
    #             xSpeed *= x_dir
    #             ySpeed *= y_dir
    #             self.publish_velocity(xSpeed, ySpeed)
    #             xtotalError += xError
    #             ytotalError += yError
    #             oldtime = newtime
    #             print("sped", xSpeed, ySpeed)      


if __name__ == '__main__':
    try:
        tempProcessor = processor()
        rospy.spin() #make it so that ros will not shut down until user inputs Ctrl-C
    except rospy.ROSInterruptException:
        pass
    



   