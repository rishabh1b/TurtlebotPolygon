# TurtlebotPolygon
Code in C++ that simulates the Turtlebot in the Gazebo Environment to move in a polygon of predefined sides and length of each side. This project is a a subpart of the coursework in Robotics and Perception at University of Maryland.

## Installation
```
mkdir -p ~/catkin_ws/src
cd catkin_ws/src
git clone https://github.com/rishabh1b/TurtlebotPolygon
catkin_init_workspace
cd ~/catkin_ws
catkin_make
```
An example for drawing hexagon-

Terminal 1 - roslaunch turtlebot_polygon project_3_world.launch

Terminal 2 - rosrun turtlebot_polygon proppolygon -n 4 -d 1

## Instructions for using it with Gazebo-7 and Indigo
Make sure you have a simulation packages for turtlebot installed from source. The Instructions to install this are as follows:
1. Install necessary dependencies
``` 
$ sudo apt-get update
$ sudo apt-get install ros-indigo-kobuki-testsuite pyqt4-dev-tools
```
2. Create a workspace with the turtlebot and kobuki packages
```
$ mkdir ~/turtle_ws
$ cd ~/turtle_ws
$ mkdir src
$ cd src
$ git clone https://github.com/turtlebot/turtlebot_simulator
$ git clone https://github.com/rohbotics/kobuki_desktop.git
```

3. Change branches for kobuki desktop - Make sure you switch to Kinetic before building
```
$ cd kobuki_desktop
$ git checkout kinetic
```
4. Compile the workspace
```
$ cd ~/turtle_ws
$ catkin_make
$ source devel/setup.bash
```
Now, to run the control code in this package, we will overlay the ```turtle_ws``` onto the ```catkin_ws```
In a new terminal:
```
cd ~/catkin_ws
source devel/setup.bash
cd ~/turtle_ws
source devel/setup.bash
roslaunch turtlebot_polygon project_3_world.launch
```
The last step should launch gazebo empty world with a TurtleBot.

In another terminal run the control code - 
```
cd ~/catkin_ws
source devel/setup.bash
rosrun turtlebot_polygon proppolygon -n 4 -d 1
```
