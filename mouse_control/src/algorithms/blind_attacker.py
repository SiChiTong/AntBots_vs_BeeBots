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
print('Hello! I\'m the blind attacker* algorithm!')

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
	utils.updateHiveReconMap(reconMap, NUM, WORLD_WIDTH, WORLD_HEIGHT, miceData, enemyFlag, myFlag)

	# run dijstrka
	for i in range(NUM):
		cm = miceData[i]
		start_state = RobotState(cm.x, cm.y, cm.ang)
		if 'F' in reconMap[cm.x][cm.y]:
			nx, ny = myFlag
		else: 
			nx, ny = enemyFlag
		flag_state = RobotState(nx, ny, 0)
		traj = path_finding.djistrka(start_state, flag_state, reconMap, WORLD_HEIGHT, WORLD_WIDTH, path=True, ignore_theta=True, debug=False)
		if len(traj) != 0:
			miceMoves[i].type = traj[0][0]
		else: 
			miceMoves[i].type = random.randint(0, 2)
	