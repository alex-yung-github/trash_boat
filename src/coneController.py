from __future__ import division, print_function
import numpy as np
import matplotlib.pyplot as plt
import cv2
import coneDet
import time

P = 0.06
I = 0.0007
D = 0.004

class Boat:
    i = 1
    def f(self):
        print('hello world')

class BoatCam:
    image = "coneimage.jpg" 
    def getImage(self):
        return self.image

class Camera:
    image = ""
    distances = []
    drawnImage = ""

    def __init__(self, img):
        self.image = img
        self.analyzeSelf()

    def getFrame(self):
        return self.image

    def analyzeSelf(self):
        self.drawnImage, self.distances = coneDet.objectRun(self.image)

    def printImage(self):
        plt.imshow(self.drawnImage)
        plt.show()
        time.sleep(4)
    


def run(boat, boatCamera):
    xtotalError = 0
    ytotalError = 0
    tempCam = Camera(boatCamera.getImage())
    oldtime = time.time()
    newtime = time.time()
    while(abs(tempCam.distances[0]) > 1 and abs(tempCam.distances[1]) > 1):
        tempCam = Camera(boatCamera.getImage())
        xError = tempCam.distances[0]
        yError = tempCam.distances[1]
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
        xSpeed = (P * xError) + (I * xtotalError) + (D * deltaTime)
        ySpeed = (P * yError) + (I * ytotalError) + (D * deltaTime)
        xSpeed *= x_dir
        ySpeed *= y_dir
        xtotalError += xError
        ytotalError += yError
        oldtime = newtime
        print("sped", xSpeed, ySpeed)

        boat.f() #left turbine, right turbine

# file = "coneimage.jpg"
# cam = Camera(file)
# cam.printImage()

#[octthird2.bmp,octthird3.bmp,octthird8b.bmp]
tempBoat = Boat()
boatCam = BoatCam()
run(tempBoat, boatCam)