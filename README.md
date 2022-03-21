# Assignment 3 - [Research_Track_1](https://unige.it/en/off.f/2021/ins/51201.html?codcla=10635) , [Robotics Engineering](https://courses.unige.it/10635).
Robot simulator using ROS, Rviz and Gazebo.
================================

-----------------------

Introduction <img src= "https://media2.giphy.com/media/fLsd17IO7HTCR85bDY/giphy.gif?cid=ecf05e47y9qetlendf7s1str4q1hzuzdr4ykg086vprnnccc&rid=giphy.gif&ct=s" width=100 height=60>
------------

>This is a Gazebo and Rviz-based ROS Robotics Simulator.
The goal of this project was to build three mobility modes that the user could choose from to allow the robot to explore an unknown but constrained territory.
On a user-interface terminal window, the user can pick the robot's desired behavior; modalities include:

* __1 - Autonomous Drive__: The user can select a goal position for the robot, and it will reach there on its own.
* __2 - Free Drive__: The user can use the keyboard to drive the robot in the environment.
* __3 - Driver Assistant__: The operator can direct the robot's movement with its keyboard, but an obstacle avoidance algorithm will keep the robot from colliding with walls.

Installing and Running <img src="https://media4.giphy.com/media/R0cVWjg6rAojjvwygg/200w.webp?cid=ecf05e473pmwhtmgw65ncai8eble4n0jfjl4jpkizf17vimd&rid=200w.webp&ct=s" width="50"></h2>
--------

This simulator is built on the [__ROS__](http://wiki.ros.org) (__Robot-Operating-Systems__) platform, specifically the NOETIC version.

For the specific project the program requires the installation of the following packages and tools before it can begin:

* [Slam Gmapping package](https://github.com/CarmineD8/slam_gmapping)

which can be installed with the bash command:

```bash
	$ git clone https://github.com/CarmineD8/slam_gmapping.git
```

* xterm
	
Easy to install with:

```bash
	$ sudo apt-get install -y xterm
```
 
* Ros navigation stack
	
To Install it:

```bash
	$ sudo apt-get install ros-<ros_distro>-navigation
```

Before running the simulation, first you have to run ROS (using ```$ roscore &``` and ```$ catkin_make``` ), then the simulation begins when the user has all of the required packages and runs a .launch file called:

__Assignment3_RT1.launch__


```console
<?xml version="1.0"?>

<launch>
  <include file="$(find Assignment3_RT1)/launch/simulation_gmapping.launch"/>
  <include file="$(find Assignment3_RT1)/launch/move_base.launch"/>
 
 <!-- Run the UI node -->
  <node pkg="Assignment3_RT1" type="UI" respawn="false" name="UI" output="screen" launch-prefix="xterm -e" required="true"/>
 
</launch>
```

## Environment and mapping
The robot moves in the environment in the figure (Gazebo view):

<p align="center">
    
<img src="https://github.com/samuelepedrazzi/Assignment3_RT1/blob/noetic/images/Gazebo_map.png" width="600" height="350">
    
</p>


The robot does not have a map of the surrounding environment at first, but owing to the laser scanners it has and the "gmapping" package, it can create one.
The final map, visible on Rviz is as follows:

<p align="center">
	
Rviz map not explored      |  Rviz map explored 
:-------------------------:|:-------------------------:
![](https://github.com/samuelepedrazzi/Assignment3_RT1/blob/noetic/images/rviz_map.png)  width="400" height="200" |  ![](https://github.com/samuelepedrazzi/Assignment3_RT1/blob/noetic/images/rviz_map_explored.png) width="400" height="200"

</p>
	
## Project structure



<img src= "https://media1.giphy.com/media/2Mn5rVOQSGnlRquUkM/200w.webp?cid=ecf05e47wixskor4jhxrjrz9it6ww1p8gd7giv8tq64fke67&rid=200w.webp&ct=s" width=100 height=60>

 <img src= "https://cdn-icons-png.flaticon.com/128/854/854894.png" width=40>

<img src= "https://media1.giphy.com/media/HGn4DKP2K6HLMTtzf9/200w.webp?cid=ecf05e47d9q1lels5jeofny61n0cbjmyhpl0zas1si8bxxbo&rid=200w.webp&ct=s" width=100 height=60>


