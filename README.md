# Mobile Robot Simulation
This is a tutorial on robot localization with the tools of ROS and RViz. All the code which is required is included in the catkin_ws folder.  
This demo was tested on __Ubuntu 20.04 using ROS Noetic Ninjemys__.  
We will test the following examples:
  * Controlling the Robot in Simulation
  * Viewing Sensor Data
  * Creating a Map // World

## Getting Started
We will run the Rviz simulator of virtual robot Turtlebot3 in version of [ROS Noetic Ninjemys](http://wiki.ros.org/noetic "ROS Noetic Ninjemys").  
We will test __SLAM (Simultaneous localization and mapping) and autonomous navigation__.
### Prerequisites
We assume you have ROS Noetic installed using Ubuntu 20.04. Check which version of ROS you have by running the command:  
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
Clone the source code (You need to have __git__ installed):
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

![image](https://user-images.githubusercontent.com/51058382/189886909-57f50ecb-83c6-4e6d-9d7d-004474bb96b6.png "turtlebot3 teleop node")

The Rviz simulator should open with the following screen:  

![image](https://user-images.githubusercontent.com/51058382/189885817-942df8b6-5639-466e-b767-f5cb8337cd01.png "Rviz simulator")  

## SLAM and Autonomous Navigation with TurtleBot3
[SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) concerns the problem of a robot building or updating a map of an unknown environment while simultaneously keeping track of its location in that environment.  

We choose an environment with obstacle avoidance for testing SLAM and navigation algorithms. The goal is to make TurtleBot3 autonomously navigate around a room and avoid colliding into objects.  
As a first step, we can access and inspect the sensor data. We can inspect the scan data from `rostopic` using a new terminal:
```
$ rostopic info /scan
```
We can check its publishing messages using:
```
$ rostopic echo /scan
```
The robot is being simulated in an open world, with no obstacles in sight, so we manually add an obstacle. We check the laser scanner topic again as we should notice that the output has changed.  
We can open RViz to visualize the LaserScan topic while TurtleBot3 is moving around in the world. In a new terminal, type:
```
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```
So we can see the __point cloud__:

![image](https://user-images.githubusercontent.com/51058382/189894510-87d2ef99-b0ec-4712-94a2-b392b9c5a3c0.png)

RViz is able to display other data. For instance, we can view the vision data by selecting the camera topic. The output of this would look like:  

![image](https://user-images.githubusercontent.com/51058382/189891527-93b9937b-ba15-4d57-9468-e63d6dbdbeea.png)

ROS comes with the SLAM module. Install it in a new terminal:
```
sudo apt install ros-noetic-slam-gmapping
```
We download the world we seek to map by calling this command:
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
Then, start SLAM in a new terminal:
```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
Finally, launch autonomous navigation in a last terminal tab:
```
roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
```
We can watch the robot create a map of the environment as it autonomously moves from place to place!

![image](https://user-images.githubusercontent.com/51058382/189900379-b9c84632-86f7-4225-8623-b43b778f32d8.png)
![image](https://user-images.githubusercontent.com/51058382/189900472-6e68301a-0a32-4779-9171-53e0572746c7.png)
![image](https://user-images.githubusercontent.com/51058382/189900560-af9843c3-048f-474e-98df-37d0ee47e7d8.png)

And that's it!  
__Keep building ;)__
