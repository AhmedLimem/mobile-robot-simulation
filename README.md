# Mobile Robot Simulation
This is a tutorial on robot localization with the tools of ROS and RViz. All the code which is required is included in the catkin_ws folder.  
This demo was tested on __Ubuntu 20.04 using ROS Noetic Ninjemys__.  
We will test the following examples:
  * Controlling the Robot in Simulation
  * Viewing Sensor Data
  * Creating a Map // World

## Getting Started
We will run the Rviz simulator of virtual robot urtlebot3 in version of [ROS Noetic Ninjemys](http://wiki.ros.org/noetic "ROS Noetic Ninjemys").  
We will test __SLAM (Simultaneous localization and mapping) and autonomous navigation__.
### Prerequisites
We assume you have ROS Noetic installed using Ububtu 20.04. Check which version of ROS you have by running the command:  
```
$ ls /opt/ros
```
### Installation
We need to install all of the dependent packages of TurtleBot3 simulator.  

We first make a ROS workspace that will contain all the code we will working on.  
In a new terminal, enter the following commands (one right after the other):
```
$ mkdir -p ~/catkin_ws/src
```
Clone the source code (You need to have git installed):
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
Then, we need to build the files:
```
$ cd ~/catkin_ws/
$ catkin_make
```
Next, source the workspace:
```
source ~/catkin_ws/devel/setup.bash
```
*Note:* TurtleBot3 has three models (Burger, Waffle, and Waffle Pi). We set the model __Burger__ before we launch TurtleBot3.  
This command open the `bashrc` file to add this setting:
```
gedit ~/.bashrc
```
And we write this line at the bottom of the file:
`export TURTLEBOT3_MODEL=burger`  
Save and close the file, then reload `.bashrc` to log back in:
```
source ~/.bashrc
```

## Simulating TurtleBot3 with RViz
[RViz](http://wiki.ros.org/rviz) is a physics engine in which we will run our simulation.  
The command `roslaunch` enables us to launch a program. ROS applications are organized in a system of __packages__, each with its own launch file. When we call `roslaunch`, we need to specify the desired package and launch file. We can build each ROS package alone. It is the smallest functional unit in the workspace. Note that we need to be inside a package for every ROS program we write.  

The ROS package should at least contain these elements:
  * src folder: Source code (C++, Python)
  * CMakeLists.txt: __CMake__ rules for compilation
  * package.xml: Package information and dependencies

We launch the simulation using the following command:
```
roslaunch turtlebot3_fake turtlebot3_fake.launch
```
If we want to move the robot around his environment, we need another launch file. We type this command in a new terminal:
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
In this terminal we click on the mentioned keys to control the movement of TurtleBot3:
