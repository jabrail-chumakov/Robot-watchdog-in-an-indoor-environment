# Robot watchdog in an indoor environment

A ROS-based exercise for the Experimental Robotics Laboratory course held at the University of Genoa.

Author: [Jabrail Chumakov](https://github.com/jabrail-chumakov)

## Documentation

The documentation can be found [here](https://jabrail-chumakov.github.io/Robot-watchdog-in-an-indoor-environment/)

## Introduction

This assignment involves the implementation of a watchdog robot in an interior setting where it must periodically visit each room while primarily remaining in the corridors. The robot also has a battery, which is gradually depleted with each walk across a room. The major objective of this study was to incorporate the finite state machine utilizing Smach, which enables you to adjust the environment-based states of the robot based on its location and battery level. The environment is generally separated into the following 7 rooms and 7 doors:
* **Room E:** The robot's starting location, which also houses the charging station.
* **Corridor 1:** Next to room E, accessible from another corridor through doors D7 or D5.
* **Corridor 2:** Located next to room E and accessible by door D6 or door D5 from another corridor. 
* **Room R1:** A room that can be entered from corridor 1 through door D1. 
* **Room R2:** A room that can be accessed from corridor 1 through door D2.
* **Room R3:** This room is reachable from corridor 2 via door D3.
* **Room R4:** This room is reachable from corridor 2 via door D4.

I've made the assumption that the entire environment is a **20 x 20** square with the origin in the top left corner for the sake of convenience. The figure below shows further assumptions that were made regarding the coordinates of the beginning point as well as the locations in each of the rooms.

<p align="center">
  <img width="400" height="400" src="https://user-images.githubusercontent.com/67557966/206316608-e73a44a9-af54-4e36-9ce8-8f25c863ee2c.jpg">
</p>

## Installation

Follow these steps to install the software.
 - Clone this repository inside your ROS workspace (which should be sourced in your `.bashrc`) and name it `robot_watchdog_in_an_indoor_environment`.
 - Run `chmod +x <file_name>` for each file inside the `scripts` folder.
 - Run `catkin_make` from the root of your ROS workspace.
 - You also need to install a few third-party libraries such as [colorama](https://pypi.org/project/colorama/) and [xterm](https://installati.one/ubuntu/21.04/xterm/) that I utilized for this project. 

Colorama is a Python library that allows you to print colored text on terminals that support ANSI escape sequences. This can be useful for creating colorful output in your Python programs, especially when working in the command line. XTerm is a terminal emulator for the X Window System. It is a program that allows users to interact with the Unix-like operating system using a command-line interface. XTerm supports a wide range of features, including color schemes, scrollback buffer, customizable fonts and key bindings, and support for international character sets. 

- After that you can write the following command to execute launch file:

```bash
$ roslaunch robot_watchdog_in_an_indoor_environment topological_map.launch
```

Inside `topological_map.launch` file you can edit other nodes to add an output screen to the terminal or xterm.

The software exploits [roslaunch](http://wiki.ros.org/roslaunch) and 
[rospy](http://wiki.ros.org/rospy) for using python with ROS. Rospy allows defining ROS nodes, 
services and related messages.

Also, the software uses [actionlib](http://wiki.ros.org/actionlib/DetailedDescription) to define
action servers. In particular, this software is based on 
[SimpleActionServer](http://docs.ros.org/en/jade/api/actionlib/html/classactionlib_1_1simple__action__server_1_1SimpleActionServer.html#a2013e3b4a6a3cb0b77bb31403e26f137).
Thus, you should use the [SimpleActionClient](https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html)
to solve the exercise.

The Finite States Machine that I used in this project based on [SMACH](http://wiki.ros.org/smach). You can check the 
[tutorials](http://wiki.ros.org/smach/Tutorials) related to SMACH, for an overview of its 
functionalities. In addition, you can exploit the [smach_viewer](http://wiki.ros.org/smach_viewer)
node to visualize and debug the implemented Finite States Machine.

## Software Architecture

For simplicity and showing purposes, we consider a scenario with the following assumptions.
 - The robot moves in a 2D environment without obstacles.
 - Given a current and target position, the robot plans a trajectory to follow, i.e., a list of via 
   points. Then, it follows the trajectory by reaching each via point. The distance between via
   points is small enough to disregard the positions within two via points (e.g., for 
   representing the current robot pose).
 - The user can only say simple (and predefined) commands to the robot through the speech-based 
   interface. The speech-based commands can be given at any time, and they might be wrongly
   detected.
 - A pointing gesture is valid only when the interaction occurs. If more gestures are provided,
   the robot should consider the more recent one.
 - The user can point to a 2D location at any time. The location might be out of the environment, 
   i.e, it can refer to a position that is unreachable by the robot.
 - The battery can become low at any time, and the robot should immediately react to this event.
 - The user does not move.

## Temporal Diagram

## State Diagram

## Package List

This repository contains a ROS package named `arch_skeleton` that includes the following resources.
 - [CMakeList.txt](CMakeList.txt): File to configure this package.
 - [package.xml](package.xml): File to configure this package.
 - [setup.py](setup.py): File to `import` python modules from the `utilities` folder into the 
   files in the `script` folder. 
 - [launcher/](launcher/): Contains the configuration to launch this package.
    - [manual_sense.launch](launcher/manual_sense.launch): It launches this package allowing 
       for keyboard-based interface.
    - [random_sense.launch](launcher/random_sense.launch): It launches this package with 
      random-based stimulus.
 - [msg/](msg/): It contains the message exchanged through ROS topics.
    - [Gesture.msg](msg/Gesture.msg): It is the message representing detected pointing gestures.
    - [Speech.msg](msg/Speech.msg): It is the message representing speech-based commands.
    - [Point.msg](msg/Point.msg): It is the message representing a 2D point.
 - [srv/](srv/): It Contains the definition of each server used by this software.
    - [GetPose.srv](srv/GetPose.srv): It defines the request and response to get the current 
      robot position.
    - [SetPose.srv](srv/SetPose.srv): It defines the request and response to set the current 
      robot position.
 - [action/](action/): It contains the definition of each action server used by this software.
    - [Plan.action](action/Plan.action): It defines the goal, feedback and results concerning 
      motion planning.
    - [Control.action](action/Control.action): It defines the goal, feedback and results 
      concerning motion controlling.
 - [scripts/](scripts/): It contains the implementation of each software components.
    - [speech.py](scripts/speech.py): It is a dummy implementation of the speech-based 
      commands detection algorithm.
    - [gesture.py](scripts/gesture.py): It is a dummy implementation of the gesture-based
      commands detection algorithm.
    - [robot_state.py](scripts/robot_state.py): It implements the robot state including:
      current position, and battery level.
    - [planner.py](scripts/planner.py): It is a dummy implementation of a motion planner.
    - [controller.py](scripts/controller.py): It is a dummy implementation of a motion 
      controller.
 - [utilities/arch_skeleton/](utilities/arch_skeleton/): It contains auxiliary python files, 
   which are exploited by the files in the `scripts` folder.
    - [architecture_name_mapper.py](scripts/architecture_name_mapper.py): It contains the name 
      of each *node*, *topic*, *server*, *actions* and *parameters* used in this architecture.
 - [diagrams/](diagrams/): It contains the diagrams shown below in this README file.

## Video demonstration

[<img src="https://user-images.githubusercontent.com/67557966/206757072-fc9b14c4-52ad-4594-9308-0a2355f47035.png" width="70%">](https://youtu.be/oVrAHOfNHNc)

---
