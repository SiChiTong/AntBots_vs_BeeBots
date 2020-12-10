# Ant-Bots vs. Some Bee-Bots - EECS 106A Fall 2020 Final Project 

A simulation of two teams of robots competing in a CTF style game.

## Text-only Considerations

DO NOT MERGE!!!

Change MAX_RANGE in mouse_description/src/sensor to match the Gazebo sim.

	# to start
	roslaunch mouse_gazebo start.launch

	# to end (from a diff terminal)
	pkill -f python3 -9

## Setup Instructions

Developed on Ubuntu 20.04.1 LTS, but other distributions should work too.

### Install ROS 1

Follow instructions to install ROS Noetic on [http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu). They are recreated below.

	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	sudo apt update
	sudo apt install ros-noetic-desktop-full
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
	source ~/.bashrc

For some reason rosdep is not installed by default.

	sudo apt install python3-rosdep
	sudo rosdep init
	rosdep update

### Create Workspace and Install Packages

You can also clone this repo into your own workspace!

	mkdir -p catkin_ws/src
	cd catkin_ws/src
	catkin_init_workspace
	git clone https://github.com/dragonlock2/AntBots_vs_BeeBots.git abvbb
	cd ..
	catkin_make

## How to Run

	catkin_make
	source devel/setup.sh

	# Start everything
	roslaunch mouse_gazebo start.launch

	# Tip to kill stuff super quick
	rosnode kill -a
	pkill -f python3 -9

	# Manual Control
	# edit mouse_gazebo/launch/start.launch to enable manual control for one team
	rosrun mouse_control manualcontrol

## Stuff of Interest

	# Internal components to mice
	ants/ant*/sensor_data # nothing rn
	ants/ant*/cmd_vel # commands to motors (runs thru a control loop in Gazebo)
	ants/ant*/laser # raw laser sensor data

	# Wireless comms w/ mothership
	/ants/ant*/command # forward, turn left, or turn right
	/ants/ant*/telemetry # discretized position/orientation + if reached

	# Wireless comms w/ god node
	/ants/godTX
	/ants/godRX
	/omniscience # level 1

	# "Threads" running per mouse
	/ants/ant*/sensor # aggregated laser sensor data
	/ants/ant*/brain # control loop to reach discretized position

	# The mothership that oversees all
	/ants/mothership

	# Equivalent topics/nodes found under /bees

	# Each mothership has access to the following for algorithm dev
	NUM # number of mice under its control
	miceData # location, orientation, and known cells from each mouse
	score # current game score
	reconMap # Level 1 Omniscient: contains positions and orientation of everything

	# Each mouse brain has access to the following
	xd, yd, angd # its own location and orientation
	sense # laser sensor identifying distance and type (if in range) + if has flag


## Build Your Own World

Check out the .txt files.

	  // air
	# // wall
	A // ant
	B // bee
	F // flag

Numbering for ants and bees is from 0 up, left to right, bottom to top, from the perspective of that team's side. Ants should be at the bottom. All robots start facing the opposite side of the map. The flag is associated with the team whose half of the map it is. Map should be evenly dividable into top and bottom halves.

## Algorithms

Change the bottom of mouse_control/src/mothership to pick an algorithm for each team.

	# Level 0: Test
	random.py
	template.py
	template_multistrat.py

	# Level 1: Omniscient
	attacker.py
	blind_astar.py
	naive_astar.py
	defender.py
	defender_tom.py
	attacker_tom.py

	# Level 2: Hivemind
	matstar.py

	# Level 3: Minion
	m3tstar.py
