#!/usr/bin/env python3
import rospy
import math 
import random 

from mouse_description.msg import MouseCommand
from . import path_finding
from .robot_state import RobotState


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
def initAlg(isant, numMice, eps=0.25, safety_score=2):
	# code to run before node even starts
	print('Hello! I\'m the attacker tom algorithm!')
	global ISANT, NUM, ENEMYCHAR, RANDOM_EPS, SAFETY_SCORE
	ISANT = isant
	NUM = numMice
	if isant: 
		ENEMYCHAR = 'B'
	else: 
		ENEMYCHAR = 'A'
	RANDOM_EPS = eps
	SAFETY_SCORE = safety_score
	
def computeMoves(miceMoves, score, miceData, omniMap):
	# miceMoves - modify this with the moves u wanna do
	# score - current score, see mouse_control/msg/Score.msg
	# miceData - telemetry data from each mouse, see mouse_description/msg/MouseData.msg
	# Ants go to goal and bees tag.

	# First call should not have any flags captured, so can grab flag locations from there
	# keep in mind these are base locations, need to re-search if flag is stolen
	if not (myFlag and enemyFlag):
		computeFlags(omniMap)

	# Level 1: Omnisas pfcient - available data
	for i in range(NUM):
		computeMouseMove(i, miceMoves, score, miceData, omniMap, reconMap)

def computeMouseMove(idx, miceMoves, score, miceData, omniMap, reconMap, myFlag, enemyFlag):
	if random.random() < RANDOM_EPS:
		miceMoves[idx].type = random.randint(0, 3)
	else: 
		current_mouse = miceData[idx]
		mx, my = current_mouse.x, current_mouse.y
		mang = current_mouse.ang
		if 'F' in omniMap[mx][my]:
			fx, fy = myFlag
		else: 
			fx, fy = enemyFlag
		start_state = RobotState(mx, my, mang)
		flag_state = RobotState(fx, fy, 0)
		traj = path_finding.djistrka(start_state, flag_state, omniMap, WORLD_HEIGHT, WORLD_WIDTH, path=True, ignore_theta=True)
		enemy_state, distance = path_finding.find_closest_enemy(start_state, ENEMYCHAR, omniMap, WORLD_HEIGHT, WORLD_WIDTH)
		if distance <= SAFETY_SCORE: 
			miceMoves[idx].type, _ = path-finding.move_away_from_enemy(start_state, enemy_state, RMAP, WORLD_HEIGHT, WORLD_WIDTH)
		else: 
			miceMoves[idx].type = traj[0][0]
	# print("Moving Ant: ", miceMoves[i].type)