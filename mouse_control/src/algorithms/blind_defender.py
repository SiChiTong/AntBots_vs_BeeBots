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
print('Hello! I\'m the blind defender algorithm!')

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
	global ISANT, NUM, ENEMYCHAR, GOALCAMPDX, GOALCAMPDY
	ISANT = isant
	NUM = numMice
	if ISANT: 
		ENEMYCHAR = 'B'
	else: 
		ENEMYCHAR = 'A'
	GOALCAMPDX, GOALCAMPDY = [], []
	L = 2
	for i in range(NUM):
		fx = 0
		while True: 
			fx = random.randint(-L, L)
			if fx != 0: break 
		fy = 0
		while True: 
			fy = random.randint(-L, L)
			if fy != 0: break 
		GOALCAMPDX.append(fx)
		GOALCAMPDY.append(fy)

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
		computeMouseMove(i, miceMoves, score, miceData, omniMap, reconMap)




def computeMouseMove(idx, miceMoves, score, miceData, omniMap, reconMap):
	current_mouse = miceData[idx]
	mx, my = current_mouse.x, current_mouse.y
	mang = current_mouse.ang
	start_state = RobotState(mx, my, mang)
	enemy_state, _ = path_finding.find_closest_enemy(start_state, ENEMYCHAR, reconMap, WORLD_HEIGHT, WORLD_WIDTH)
	
	if enemy_state is None:
		ex = GOALCAMPDX[idx]
		ey = GOALCAMPDY[idx]
		nx = min(max(myFlag[0] + ex, 0), WORLD_WIDTH - 1)
		ny = min(max(myFlag[1] + ey, 0), WORLD_HEIGHT - 1)
		flag_state = RobotState(nx, ny, 0)
		print(f"flag_state: {flag_state}")
		traj = path_finding.djistrka(start_state, flag_state, reconMap, WORLD_HEIGHT, WORLD_WIDTH, path=True, ignore_theta=True)
		if len(traj) ==  0: 
			miceMoves[idx].type = MouseCommand.STOP
		else: 
			miceMoves[idx].type = traj[0][0]
		if not utils.in_my_half(ISANT, WORLD_HEIGHT, traj[0][1]):
			miceMoves[idx].type = MouseCommand.STOP
	else: 
		traj = path_finding.djistrka(start_state, enemy_state, reconMap, WORLD_HEIGHT, WORLD_WIDTH, path=True, ignore_theta=True)
		# for (a, s) in traj: print(f"A: {a} s: {s}")
		miceMoves[idx].type = traj[0][0]
		if not utils.in_my_half(ISANT, WORLD_HEIGHT, traj[0][1]):
			miceMoves[idx].type = MouseCommand.STOP
	