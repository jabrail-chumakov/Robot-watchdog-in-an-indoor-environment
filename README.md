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

## Software 

The specific **software architecture** of a robot deployed in an indoor environment for surveillance purposes would depend on the specific design and programming of the robot. However, in general, the robot's software architecture could be divided into a number of different components, each with a specific role and function. Some potential components of the robot's software architecture could include:

- Navigation and movement: This component would be responsible for controlling the robot's movements, including its speed, direction, and trajectory. It would use data from the robot's sensors and cameras to avoid obstacles and navigate the space, and it would be able to respond to commands or changes in the environment.
- Data collection and analysis: This component would be responsible for gathering data from the robot's sensors and cameras, as well as any other sources of information, such as external databases or other systems. It would then use algorithms and data structures to process and analyze the data, looking for potential threats or anomalies.
- Decision making and action: This component would be responsible for making decisions based on the data gathered and analyzed by the robot. It would use a set of rules or protocols to determine the appropriate course of action in response to different situations, and it would be able to initiate actions, such as sounding an alarm or sending a notification, as needed.
- Communication: This component would be responsible for managing the robot's communication with other systems, such as security personnel or other surveillance robots. It would handle the transmission and receipt of data, as well as any necessary encoding or encryption.

Generally, the robot's software architecture would likely involve a complex combination of components, each working together to enable the robot to fulfill its surveillance objectives.

<p align="center">
  <img width="657" height="201.5" src="https://user-images.githubusercontent.com/67557966/206814146-e68fbba2-7fab-4ae3-b3d8-afcdacc8c8e6.png">
</p>

The **temporal diagram** of a robot deployed in an indoor environment for surveillance purposes would likely depend on the specific details of its design and programming. However, in general, the robot's behavior could be described as follows:

- The robot begins by powering on and initializing its systems, including its sensors, cameras, and communication equipment.
- The robot then enters a "patrol" mode, in which it moves around the space, visiting different locations and staying in each location for a certain amount of time.
- While in patrol mode, the robot uses its sensors and cameras to gather data about the environment, looking for potential threats or other anomalies.
- If the robot detects a potential threat or anomaly, it may take a number of actions, such as sounding an alarm, sending a notification to security personnel, or taking a photo or video of the threat.
- Once the robot has completed its patrol of the space, it may enter a "sleep" mode to conserve power, or it may return to the beginning of its patrol route to start again.

Overall, the temporal diagram of the robot's behavior would likely involve a combination of movement, data gathering, and decision making, with the specific details and timing of these actions depending on the specific design and programming of the robot.

The **states diagrams** of a robot deployed in an indoor environment for surveillance purposes would likely depend on the specific design and programming of the robot. However, in general, the robot's states could be described as follows:

- Idle: In this state, the robot is powered on but not actively performing any tasks. It may be waiting for a command or for a specific event to occur before entering another state.
- Patrol: In this state, the robot is moving around the space, visiting different locations and staying in each location for a certain amount of time. It is using its sensors and cameras to gather data about the environment, and it is looking for potential threats or anomalies.
- Alert: In this state, the robot has detected a potential threat or anomaly, and it is taking action to respond to the situation. This could include things like sounding an alarm, sending a notification to security personnel, or taking a photo or video of the threat.
- Sleep: In this state, the robot is powered down or in a low-power mode to conserve energy. It may enter this state when it is not actively performing any tasks, or when it has completed its patrol of the space.

## Video demonstration

[<img src="https://user-images.githubusercontent.com/67557966/206757072-fc9b14c4-52ad-4594-9308-0a2355f47035.png" width="70%">](https://youtu.be/oVrAHOfNHNc)

## Working hypothesis

If a robot is deployed in an indoor environment for surveillance purposes, its main objective would be to move around the space and visit different locations within the space in order to gather information and monitor the environment. The robot may be equipped with sensors and cameras to allow it to navigate the space and collect data, and it may also have some means of communication, such as a loudspeaker, to alert occupants of the space or security personnel of any potential threats. In order to fulfill its objective of visiting different locations and staying there for a certain amount of time, the robot may be programmed with a set of rules and protocols that dictate its behavior and movements. This could include things like how long to stay in each location, how to prioritize certain areas over others, and how to avoid obstacles or other potential hazards in the environment.

The robot begins at position E and waits for data before beginning to construct the topological map. The robot shifts to a new position and waits there for a while before going somewhere else. This behavior keeps happening indefinitely. When the robot's battery is low, it travels to location E and waits there for a while before resuming the aforementioned behavior. The robot should navigate between locations according to the following rule while its battery is not low:
- It should primarily remain in corridors.
- It should visit a reachable room if it hasn't been seen in a while.

In this project, it was also expected that the surroundings would be `20 by 20` and that the robot's mobility would begin at `(x, y) | (7.0, 2.0)`. When the battery level drops below a certain point, it then moves in a random direction.
 
### System's features

If a robot is deployed in an indoor environment for surveillance purposes, its system may include a number of features designed to help it fulfill its objective of visiting different locations and staying there for a certain amount of time. Some potential system features could include:

- Sensors and cameras to allow the robot to navigate the space and gather data.
- A means of communication, such as a loudspeaker, to alert occupants of the space or security personnel of any potential threats.
- A set of rules and protocols governing the robot's behavior and movements, such as how long to stay in each location and how to prioritize certain areas over others.
- Obstacle avoidance capabilities to help the robot navigate around potential hazards in the environment.
- A power source, such as a battery or external power supply, to keep the robot running for extended periods of time.
- A network connection, such as Wi-Fi or cellular, to allow the robot to communicate with other systems or send data to a remote location for analysis.

In my instance, I added a battery that enables the robot to run up to its threshold. I should go back to the base room to rest after that. Additionally, the user can examine the robot's current location and direction straight from the `smach viewer`.

### System's limitations

While a robot deployed in an indoor environment for surveillance purposes may have a number of useful features, it is also likely to have some limitations. Some potential limitations of such a system could include:

- Limited mobility: The robot may not be able to navigate all areas of the space, particularly if there are tight corners, narrow passageways, or other obstacles that are difficult for the robot to navigate.
- Limited sensor range: The robot's sensors and cameras may not be able to detect threats or gather data from all areas of the space, particularly if there are large objects blocking the sensors' line of sight or if the sensors are not powerful enough to detect distant objects.
- Limited communication range: If the robot relies on a wireless connection to communicate with other systems or send data to a remote location, it may be limited by the range of the wireless signal, which can be disrupted by obstacles or other interference.
- Limited power: If the robot relies on a battery or other power source, it may need to be recharged or replaced frequently, which could limit its ability to operate continuously.
- Limited data processing and analysis: If the robot is collecting a large amount of data, it may have difficulty processing and analyzing all of the information in real-time, which could lead to potential threats being missed or overlooked.

### Possible technical Improvements

There are a number of potential technical improvements that could be made to a robot deployed in an indoor environment for surveillance purposes. Some potential improvements could include:

- Visual data: This project will be more challenging if it is implemented using Rviz and Gazebo, which could improve our understanding of robot movement and require the implementation of an obstacle avoidance algorithm.
- Improved mobility: The robot could be equipped with advanced navigation and obstacle avoidance capabilities, allowing it to move more easily and efficiently through the space. This could include things like better sensors, more powerful motors, and advanced algorithms for planning and executing movements.
- Improved sensor range: The robot's sensors and cameras could be made more powerful, allowing them to detect threats and gather data from a wider area. This could include things like higher resolution cameras, longer range sensors, and advanced image processing algorithms to extract more information from the data.
- Improved communication range: The robot could be equipped with a more powerful wireless communication system, allowing it to maintain a strong signal over longer distances and in the presence of obstacles or other interference. This could include things like advanced antennas, higher frequency bands, and better error correction algorithms.
- Improved power: The robot could be equipped with a more efficient power system, allowing it to operate for longer periods of time without needing to be recharged or replaced. This could include things like better batteries, more efficient motors, and advanced power management algorithms.
- Improved data processing and analysis: The robot could be equipped with more powerful processors and algorithms for analyzing the data it collects, allowing it to identify potential threats more quickly and accurately. This could include things like better machine learning algorithms, more efficient data structures, and advanced parallel processing techniques.
