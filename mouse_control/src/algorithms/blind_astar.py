#!/usr/bin/env python3
import rospy
import numpy as np

from mouse_description.msg import MouseCommand
from . import path_finding
import copy
from . import utils
from .robot_state import RobotState
import random

# constants
WORLD_HEIGHT = rospy.get_param('/WORLD_HEIGHT')
WORLD_WIDTH = rospy.get_param('/WORLD_WIDTH')

# code to run before node even starts
print('Hello! I\'m the blind A* algorithm!')

# private variables
myFlag = None
enemyFlag = None
reconMap = [[' ' for j in range(WORLD_HEIGHT)] for i in range(WORLD_WIDTH)]

# Level 1: Total Omniscience

# helpers
def computeFlags(rmap):
	global myFlag, enemyFlag
	for x in range(WORLD_WIDTH):
		for y in range(WORLD_HEIGHT // 2):
			if rmap[x][y] == 'F':
				if ISANT:
					myFlag = (x,y)
				else:
					enemyFlag = (x,y)
				reconMap[x][y] = 'F'
	for x in range(WORLD_WIDTH):
		for y in range(WORLD_HEIGHT // 2, WORLD_HEIGHT):
			if rmap[x][y] == 'F':
				if ISANT:
					enemyFlag = (x,y)
				else:
					myFlag = (x,y)
				reconMap[x][y] = 'F'


# standard interface functions
def initAlg(isant, numMice):
	global ISANT, NUM, ENEMYCHAR, lastxs, lastys, currentxs, currentys
	ISANT = isant
	NUM = numMice
	if ISANT: 
		ENEMYCHAR = 'B'
	else: 
		ENEMYCHAR = 'A'

	#listxs = [-1 for a in range(NUM)]
	#listys = [-1 for b in range(NUM)]
	#listxs = np.zeros(NUM)
	#listys = np.zeros(NUM)
	lastxs = []
	lastxs = [-1 for _ in range(NUM)]
	lastys = []
	lastys = [-1 for _ in range(NUM)] 

	currentxs = []
	currentxs = [-1 for _ in range(NUM)]
	currentys = []
	currentys = [-1 for _ in range(NUM)]


def computeMoves(miceMoves, score, miceData, omniMap):
	# miceMoves - modify this with the moves u wanna do
	# score - current score, see mouse_control/msg/Score.msg
	# miceData - telemetry data from each mouse, see mouse_description/msg/MouseData.msg
	# TODO add sensor data from each mouse to do level 2 knowledge

	# Level 2: Hivemind - available data
	# reconMap - xy-indexed, see mouse_control/msg/Omniscience.msg, only has already seen nodes from all bots

	# First call should not have any flags captured, so can grab flag locations from there
	# keep in mind these are base locations, need to re-search if flag is stolen
	if not (myFlag and enemyFlag):
		computeFlags(omniMap)

	global reconMap
	# get all the xs, ys, types for every mouse
	hive_mind = dict()
	for i in range(NUM):
		current_mouse = miceData[i]
		types = current_mouse.types
		xs = current_mouse.xs
		ys = current_mouse.ys
		for j in range(len(types)):
			x, y = xs[j], ys[j]
			hive_mind[(x, y)] = types[j]

	# clear the map besides obstacles
	for x in range(WORLD_WIDTH):
		for y in range(WORLD_HEIGHT):
			if reconMap[x][y] != '#':
				reconMap[x][y] = ' '

	# make a new map with the xs, ys, and types of 'A' and 'B' and 'F' added in
	for p, t in hive_mind.items():
		x, y = p
		reconMap[x][y] = t  
	
	# add back the flags
	reconMap[enemyFlag[0]][enemyFlag[1]] = 'F'
	reconMap[myFlag[0]][myFlag[1]] = 'F'
	
	orien_symbols = {'0': ">", '1': "^", '2': "<", '3': "v"}
	for y in reversed(range(WORLD_HEIGHT)):
		for x in range(WORLD_WIDTH):
			r = reconMap[x][y]
			if 'B' in r or 'A' in r:
				angle = r[0]
				a = orien_symbols[angle] + r[1:]
			else: 
				a = r
			print(f'{a:^5}', end="")
		print()

	# run dijstrka
	for i in range(NUM):
		cm = miceData[i]
		start_state = RobotState(cm.x, cm.y, cm.ang)
		flag_state = RobotState(enemyFlag[0], enemyFlag[1], 0)
		traj = path_finding.djistrka(start_state, flag_state, reconMap, WORLD_HEIGHT, WORLD_WIDTH, path=True, ignore_theta=True, debug=False)
		if len(traj) != 0:
			miceMoves[i].type = traj[0][0]
		else: 
			miceMoves[i].type = random.randint(0, 2)
	