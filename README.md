# Capstone 120 - User Guide

This is the Github repository of Capstone Team 120. You will find all necessary instructions in the README.


## Starting the Robot

First, cd into the project's directory.

If a package has been modified or if this is your first time testing the project, run:
```
catkin_make
```

Everytime you want to start the robot, turn it on and run the following commands:
```
source devel/setup.bash
roslaunch moving move_arm.launch real_arm:=true
```
*Note: if you want to run just the simulation without having the arm connected, change the real_arm parameter to false.*


## Calibrating the Camera position

First, place the Arm on marked spot of the sheet of paper with the AR Tag. Then, turn on the robot as explained in **Starting the Robot**, and use *free moving mode* to make sure the AR Tag is within view of the camera. Once the robot is in place, run the following command:
```
roslaunch ar_track_alvar cam_pose_calibration.launch
```
It should output the spatial transformation between the sgr532/link6 and usb_cam_link frames.

Next, edit the file **sgr532.urdf.xacro** in folder [capstone120_ws]/src/sagittarius_pcks/sagittarius_descriptions/urdf/ and go to **line 659**. Replace the xyz and rpy values with the ones displayed by the cam_pose_calibration.launch file.

