ROS-Video_handler
=================

A very basic ros node to publish video data to a ros topic. 
Motivation for this is to have a basic publisher where you can select from which file you're streaming (or camera device) and to which topic you send the information to. 

Installation
------------

The node depends on OpenCV and catkin which you should install first.

To install the node. clone the repo into a catkin source directory and build the program. As shown below:

```
cd src
git clone https://github.com/ldecamp/ros-video_handler.git
cd .. && catkin_make
```

Execution
---------

Two sample launch files are provided in the launch directory

you can launch the node from launch directory in the terminal using: 

```
roslaunch video_handler THE_SETTINGS.launch
````

Enjoy!