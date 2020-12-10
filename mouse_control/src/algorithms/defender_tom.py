#!/usr/bin/env python3
import rospy
import math 
import random

from mouse_description.msg import MouseCommand
from . import path_finding
from .robot_state import RobotState
from . import utils

# constants
WORLD_HEIGHT = rospy.get_param('/WORLD_HEIGHT')
WORLD_WIDTH = rospy.get_param('/WORLD_WIDTH')
reconMap = [[' ' for j in range(WORLD_HEIGHT)] for i in range(WORLD_WIDTH)]

# private variables
myFlag = None
enemyFlag = None

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
	for x in range(WORLD_WIDTH):
		for y in range(WORLD_HEIGHT // 2, WORLD_HEIGHT):
			if rmap[x][y] == 'F':
				if ISANT:
					enemyFlag = (x,y)
				else:
					myFlag = (x,y)

# standard interface functions
def initAlg(isant, numMice, random_blocker=True, fixed_blocker=-3):
	# code to run before node even starts
	print('Hello! I\'m the tagger algorithm!')
	global ISANT, NUM, ENEMYCHAR, RANDOM_BLOCKER
	ISANT = isant
	NUM = numMice
	RANDOM_BLOCKER = random_blocker
	FIXED_BLOCKER = fixed_blocker 
	if isant: 
		ENEMYCHAR = 'B'
	else: 
		ENEMYCHAR = 'A'
	
def computeMoves(miceMoves, score, miceData, omniMap):
	# miceMoves - modify this with the moves u wanna do
	# score - current score, see mouse_control/msg/Score.msg
	# miceData - telemetry data from each mouse, see mouse_description/msg/MouseData.msg
	# Ants go to goal and bees tag.

	if not (myFlag and enemyFlag):
		computeFlags(omniMap)

	# Level 1: Omnisas pfcient - available data
	for i in range(NUM):
		computeMouseMove(i, miceMoves, score, miceData, omniMap, reconMap)

def computeMouseMove(idx, miceMoves, score, miceData, omniMap, reconMap, myFlag, enemyFlag):
	current_mouse = miceData[idx]
	mx, my = current_mouse.x, current_mouse.y
	mang = current_mouse.ang
	start_state = RobotState(mx, my, mang)
	enemy_state, _ = path_finding.find_closest_enemy(start_state, ENEMYCHAR, omniMap, WORLD_HEIGHT, WORLD_WIDTH)
	my_flag_state = RobotState(myFlag[0], myFlag[1], 0)
	enemy_traj = path_finding.djistrka(enemy_state, my_flag_state, omniMap, WORLD_HEIGHT, WORLD_WIDTH, path=True, ignore_theta=True)
	if RANDOM_BLOCKER: 
		index = random.randint(0, len(enemy_traj) - 1)
		enemy_future_state = enemy_traj[index][1]
		traj = path_finding.djistrka(start_state, enemy_future_state, omniMap, WORLD_HEIGHT, WORLD_WIDTH, path=True, ignore_theta=True)
	elif -FIXED_BLOCKER <= len(enemy_traj): 
		enemy_future_state = enemy_traj[FIXED_BLOCKER][1]
		traj = path_finding.djistrka(start_state, enemy_future_state, omniMap, WORLD_HEIGHT, WORLD_WIDTH, path=True, ignore_theta=True)
	else: 
		traj = path_finding.djistrka(start_state, enemy_state, omniMap, WORLD_HEIGHT, WORLD_WIDTH, path=True, ignore_theta=True)

	# for (a, s) in traj: print(f"A: {a} s: {s}")
	miceMoves[idx].type = traj[0][0]
	print(f"traj: {traj[0][1]}")

	if not utils.in_my_half(ISANT, WORLD_HEIGHT, traj[0][1]):
		miceMoves[idx].type = MouseCommand.STOP
	# print("Moving Bee: ", miceMoves[i].type)