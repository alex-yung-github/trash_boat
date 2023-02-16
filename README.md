# trash_boat

## Installation

###Creating a Package
Create a folder (a workspace)
cd that folder
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
git clone git@github.com:EatSumRice/trash_boat.git
```

Extract all the items out of the trash_boat folder created within the trash_boat folder and then delete the old folder. (basically remove all the old items in the trash_boat folder and replace it with the new items cloned from the github repository)
Follow this guide to install the required packages for the pylon-ros-camera and install it in the same src folder: https://github.com/basler/pylon-ros-camera 
Please note that you may need to install pylon (the deb or tar package): https://www.baslerweb.com/en/products/basler-pylon-camera-software-suite/ 
Install the livox data retriever using this guide: https://github.com/Livox-SDK/livox_ros_driver 
Note that your file structure should look something like this image:
https://drive.google.com/file/d/1iwxbmhFR0OyVdmJw_4klo4V31bHIjNKT/view?usp=share_link 
Robo_catkin is the name of catkin_ws (any name works really)
Dragandbot, pylon-ros-camera, and ws_livox are both different packages that our package, trash_boat, uses
trash_boat is our package 
Within the YOLOv5 Classifier folder, clone this repository:
https://github.com/SophiaLee2023/YOLOv5-Trash-Classifier
Command: 
git clone git@github.com:SophiaLee2023/YOLOv5-Trash-Classifier.git 
Now, you should catkin_make (which builds the packages). Do these commands within the catkin_ws folder:
source /opt/ros/noetic/setup.bash
catkin_make
If it compiles with no errors, great! Otherwise, please ask me for help.
Install OpenCV onto the home location using this (otherwise it doesn’t work for some reason): https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html 
Install torch libraries with your decided operating system using pip3
https://pytorch.org/get-started/locally/ and choose the options stable, linux (or your operating system), python, and cuda 11.6 (leftmost cuda)
Don’t use anaconda3, or if you do I won’t be able to assist in any errors. I got many errors while using anaconda3 for some reason and spent 2 weeks trying to fix it.
Once you get torch, make sure that the ros file uses python 3 with python --version. If it doesn’t, you might have to change the environment variable PATH


Install the rest of the dependences in the requirements.txt folder within the src folder in the trash_boat folder
Install Mavros
https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation 
How to use: https://docs.px4.io/main/en/ros/mavros_custom_messages.html 
Good Guide: https://masoudir.github.io/mavros_tutorial/ 



Saved Commands to Run: 
First terminal: roscore		
this is the base to run anything with ROS
Second terminal: roslaunch pylon_camera pylon_camera_node.launch		
this starts the camera node to retrieve image data
Github with information: https://github.com/basler/pylon-ros-camera 
Third terminal: rosrun trash_boat trashPIDController.py		
this prints out camera images and will run the YOLO v5 algorithm on the images (and print bounding boxes on the images)
Fourth Term:  roslaunch livox_ros_driver livox_lidar_rviz.launch bd_list:="0TFJBF001663J1"

Saved Useful Websites
General ROS ML: https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/ 
