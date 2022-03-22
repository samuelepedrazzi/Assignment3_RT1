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

ROS creates the environment described in the __world__ folder's file `house.world.`

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

Depending on the user input it select the correct action to do, such as running one of the nodes,

```cpp
system("rosrun Assignment3_RT1 achieveGoalPosition")
```

close the program (exiting from the main function) or reset the simulation with 

```cpp
ros::service::call("/gazebo/reset_simulation", reset)`
```

For the three nodes I choose to use a non-blocking function to get the user input, which is good for speeding up program execution and improving consumer experience (you don't have to press enter key every time).

The repository I found on Github and changed a little bit for my purposes is from `kbNonBlock` at [teleop_twist_keyboard_repo](https://gist.github.com/whyrusleeping/3983293).

Achieve Goal Position node <img src= "https://cdn-icons-png.flaticon.com/128/854/854894.png" width=40>
--------------

The first needed feature is implemented by the achieveGoalPosition node. In fact, it gives the robot a new goal based on the user's preferences.
This node's goal is to drive the robot into the correct location in the environment once a position coordinates have been specified. At first the user is asked for the goal's x and y coordinates, after which the program generates and publishes a message of type `move_base_msgs/MoveBaseActionGoal` in the `/move_base/goal` topic. The node keeps track of each objective by assigning it an id that is generated at random within the node.

A `/move_base/status` message handler is used to determine whether the robot has reached the goal. It examines the messages that have been published on the previously indicated subject.

The initial status code is __1__, which indicates that the robot is on his way and the goal is active.
When the robot comes to a halt, the status code changes to __3__ if the robot has reached the goal position, and to __4__ if the robot is unable to reach the given location.
Other statuses that have been managed are for instance the goal lost with status identifier __5__ or the rejected status with the code __9__.

After having received the feedback of the status and the robot is stopped, so there isn't any active pending goal, the function `CancelGoal()`is called, a message of type `actionlib_msgs/GoalID` is generated and then published into the `/move_base/cancel` topic.

DriveWithKeyboard node
--------------


The user can control the robot movement with the keypad (remember to click on BLOC NUM, otherwise the correct ascii code will not be read):

<center>

|| Turn left | Do not turn | Turn right|							
|:--------:|:--------:|:----------:|:----------:|
|__Go forward__|`7`|`8`|`9`
|__Stop__|`4`|`5`|`6`
|__Go backward__|`1`|`2`|`3`

</center>

The user can change the robot velocity of 10% by pressing the following keys:

<center>

|| Linear and Angular | Linear only | Angular only|
|:--------:|:--------:|:----------:|:----------:|
|__Increment__|`*`|`+`|`a`
|__Reset__|`R or r`|`e`|`w`
|__Decrease__|`/`|`-`|`z`

</center>

The robot starts moving and the move_base node publishes the right velocity and orientation on the cmd_vel topic.

<img src= "https://media1.giphy.com/media/2Mn5rVOQSGnlRquUkM/200w.webp?cid=ecf05e47wixskor4jhxrjrz9it6ww1p8gd7giv8tq64fke67&rid=200w.webp&ct=s" width=100 height=60>

<img src= "https://media1.giphy.com/media/HGn4DKP2K6HLMTtzf9/200w.webp?cid=ecf05e47d9q1lels5jeofny61n0cbjmyhpl0zas1si8bxxbo&rid=200w.webp&ct=s" width=100 height=60>


