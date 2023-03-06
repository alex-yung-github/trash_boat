# trash_boat

#NOTE: CURRENTLY A WORK IN PROGRESS. What you see below is what our package looks like so far

# What this package does

The purpose of this project is to operate a boat with the NViDIA Jetson Computer with a Mid40 Livox Lidar, Basler Camera, and Pixhawk (+ turbines) to autonomously clean up trash from bodies of water. In the future, we will update the exact specifics of our boat and sensors.

So far, when you run our package, it should take in video and identify objects of trash which the boat will go towards to pick up. At the moment, we are working on integrating movement and getting real-world tests in.


# Installation

## Creating our Package

Create a folder (a workspace)
and cd that folder
```bash
mkdir catkin_ws
cd catkin_ws
```
Create a src folder and open that folder
```bash
mkdir src && cd src
```

Create a package (with any name, ours is trash_boat) in the src folder. You can use the following command:
```bash
catkin_create_pkg --rosdistro noetic trash_boat std_msgs rospy roscpp
```
For more information about packages, look here: https://wiki.ros.org/ROS/Tutorials/CreatingPackage 
Note: Must include std_msgs and rospy in the command creation

Open the trash_boat folder (or your folder name) and then clone this github repository: https://github.com/EatSumRice/trash_boat 
```bash
cd trash_boat
git clone https://github.com/EatSumRice/trash_boat.git
```

Extract all the items out of the trash_boat folder created within the trash_boat folder and then delete the old folder. (basically remove all the old items in the trash_boat folder and replace it with the new items cloned from the github repository)

# Getting other Packages as Dependencies

## Installing the Camera Package. Note: Might vary depending on your camera
Follow this guide to install the required packages for the pylon-ros-camera and install it in the same src folder: https://github.com/basler/pylon-ros-camera 
Please note that you may need to install pylon (the deb or tar package): https://www.baslerweb.com/en/products/basler-pylon-camera-software-suite/ 

## Installing the Lidar data retrieval package
Install the livox data retriever using this guide: https://github.com/Livox-SDK/livox_ros_driver 

## Quick Check:
Note that your file structure should look something like this image:
https://drive.google.com/file/d/1iwxbmhFR0OyVdmJw_4klo4V31bHIjNKT/view?usp=share_link 
Robo_catkin is the name of my catkin_ws (but any name works really)
Dragandbot, pylon-ros-camera, and ws_livox are both different packages that our package, trash_boat, uses
trash_boat is our package 

## Installing our YOLOv5 algorithm for image object detection
Within the YOLOv5 Classifier folder in the trash_boat package, clone this repository:
https://github.com/SophiaLee2023/YOLOv5-Trash-Classifier
```bash
git clone git@github.com:SophiaLee2023/YOLOv5-Trash-Classifier.git 
```

# Building the entire catkin workspace
Now, you should catkin_make (which builds the packages). Do these commands within the catkin_ws folder:
```bash
source /opt/ros/noetic/setup.bash
catkin_make
```
If it compiles with no errors, great! Otherwise, there may have been some errors in the installation steps.

# Installing some library requirements
## Open CV
Install OpenCV onto the home location using this (if it works another way, great!): https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html 

## Pytorch
Install torch libraries with your decided operating system using pip3
https://pytorch.org/get-started/locally/ and choose the options stable, linux (or your operating system), python, and cuda 11.6 (leftmost cuda)
Personally, I had difficulties using anaconda3 to install pytorch, so I wouldn't recommend using anaconda3 for this project.
Once you get torch, make sure that the ros file uses python 3 with python --version. If it doesn’t, you might have to change the environment variable PATH (within your .bashrc folder)

## MavROS
Install the rest of the dependences in the requirements.txt folder within the src folder in the trash_boat folder
Install Mavros
https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation 

# How to Run our Program
First terminal: 
```bash
roscore		
```
This is the base to run anything with ROS

Second terminal: 
```bash
roslaunch pylon_camera pylon_camera_node.launch		
```
This starts the camera node to retrieve image data.
For more information about the camera node, visit: https://github.com/basler/pylon-ros-camera 

Third terminal: 
```bash
roslaunch livox_ros_driver livox_lidar_rviz.launch bd_list:="0TFJBF001663J1"
```
This starts the connection to the lidar and starts getting data from it.

Fourth Term:  
```bash
roslaunch trash_boat program.launch	
```
This prints out camera images and will run the YOLO v5 algorithm on the images (and print bounding boxes on the images).


