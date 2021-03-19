# WReadInput

This is the ROS input device library for Wisconsin Robotics.
The nodes in this package read from specific input devices (e.g. xbox controllers) using the Linux evdev interface and relay the data to ROS topics.
Controller initialization is handled by [WReady](https://github.com/WisconsinRobotics/wready), so some WReady server implementation must be present in the robot system where WReadInput is used.
This software is designed for ROS Noetic and Python 3.8.x.
