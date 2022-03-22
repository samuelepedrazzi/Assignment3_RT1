# Assignment 3 - [Research_Track_1](https://unige.it/en/off.f/2021/ins/51201.html?codcla=10635) , [Robotics Engineering](https://courses.unige.it/10635).
Robot simulator using ROS, Rviz and Gazebo.
================================

-----------------------

Introduction <img src= "https://media2.giphy.com/media/fLsd17IO7HTCR85bDY/giphy.gif?cid=ecf05e47y9qetlendf7s1str4q1hzuzdr4ykg086vprnnccc&rid=giphy.gif&ct=s" width=100 height=60>
------------

>This is a Gazebo and Rviz-based ROS Robotics Simulator.
The goal of this project is to build three mobility modes that the user could choose from to allow the robot to explore an unknown but constrained territory inside the simulation.
On a user-interface terminal window, the user can pick the robot's desired behavior; the modalities include:

* __1 - Autonomous Drive__: The user can select a goal position for the robot, and it will reach there on its own.
* __2 - Free Drive__: The user can use the keyboard to drive the robot in the environment.
* __3 - Driver Assistant__: The operator can direct the robot's movement with the keyboard, but an collision avoidance algorithm will keep the robot from crashing with walls.

Installing and Running <img src="https://media0.giphy.com/media/XqYKfpjBL2bjUcWVQD/200w.webp?cid=ecf05e47f0avfktds1q4ksjx91r0uw1m2unss4u1btdrzy12&rid=200w.webp&ct=s" width="50"></h2>
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

Rviz (a 3D visualizer for the Robot Operating System (ROS) framework) and Gazebo (an open-source 3D Robotics simulator) appear on the screen as soon as the simulator starts:
Thanks to its sensors, the robot can see what's going on in the world around it.

ROS creates the environment described in the __world__ folder's file 'house.world.'

The robot moves in the ambience in the figure (Gazebo view):

<p align="center">
    
<img src="https://github.com/samuelepedrazzi/Assignment3_RT1/blob/noetic/images/Gazebo_map.png" width="600" height="350">
    
</p>

The robot knows only the portion of the surroundings that its sensors allow it to detect from its starting point because it does not know all of the map's boundaries.

The robot's map knowledge grows as it moves around the map. The robot's known surroundings is graphed and updated at each time instant on Rviz.

<p align="center">
	
Rviz map not explored      |  Rviz map explored 
:-------------------------:|:-------------------------:
<img src="https://github.com/samuelepedrazzi/Assignment3_RT1/blob/noetic/images/rviz_map.png" width="400" height="550"> |  <img src="https://github.com/samuelepedrazzi/Assignment3_RT1/blob/noetic/images/rviz_map_explored.png" width="400" height="550">

</p>

User-Interface <img src="https://media4.giphy.com/media/o8QCgJacJR5balxq8Y/200w.webp?cid=ecf05e47v2cy2r25cqior5ftits1w4lipka50hjfqkj4jhz4&rid=200w.webp&ct=s" width="40"></h2>
--------------

This is the primary node, and it was the first to appear.

This node shows the user a little image that explains how to choose the robot's movement modes. It also manages user input by modifying the ros parameters that allow the activation of the nodes defined for each mode if the command is correct.

The following are some of the commands that can be used:

<img src="https://github.com/samuelepedrazzi/Assignment3_RT1/blob/noetic/images/ui.png" width="450" height="450">

Depending on the user input it select the correct action to do, such as running one of the nodes (), close the program (exiting from the main function) or reset the simulation with ```console
ros::service::call("/gazebo/reset_simulation", reset);
```




<img src= "https://media1.giphy.com/media/2Mn5rVOQSGnlRquUkM/200w.webp?cid=ecf05e47wixskor4jhxrjrz9it6ww1p8gd7giv8tq64fke67&rid=200w.webp&ct=s" width=100 height=60>

<img src= "https://cdn-icons-png.flaticon.com/128/854/854894.png" width=40>

<img src= "https://media1.giphy.com/media/HGn4DKP2K6HLMTtzf9/200w.webp?cid=ecf05e47d9q1lels5jeofny61n0cbjmyhpl0zas1si8bxxbo&rid=200w.webp&ct=s" width=100 height=60>


