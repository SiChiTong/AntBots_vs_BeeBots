#!/usr/bin/env python3
import rospy
import math 

from mouse_description.msg import MouseCommand
from . import path_finding


# constants
WORLD_HEIGHT = rospy.get_param('/WORLD_HEIGHT')
WORLD_WIDTH = rospy.get_param('/WORLD_WIDTH')

# code to run before node even starts
print('Hello! I\'m the tagger algorithm!')

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
def initAlg(isant, numMice):
	global ISANT, NUM
	ISANT = isant
	NUM = numMice

def getClosestEnemyInRmap(sx, sy, rmap):	
	closestEnemyX, closestEnemyY, closestEnemyManhattanDist = -1,  -1, math.inf
	for x in range(WORLD_WIDTH):
		for y in range(WORLD_HEIGHT):
			if ISANT: 
				enemyChar = 'B'
			else: 
				enemyChar = 'A'
			manhattanDist = path_finding.manhattan(sx, sy, x, y)
			if enemyChar in rmap[x][y] and manhattanDist < closestEnemyManhattanDist:
				closestEnemyManhattanDist = manhattanDist
				closestEnemyX = x
				closestEnemyY = y
	return closestEnemyX, closestEnemyY, closestEnemyManhattanDist


def computeMoves(miceMoves, score, miceData, reconMap):
	# miceMoves - modify this with the moves u wanna do
	# score - current score, see mouse_control/msg/Score.msg
	# miceData - telemetry data from each mouse, see mouse_description/msg/MouseData.msg
	# Ants go to goal and bees tag.

	# Level 1: Omnisas pfcient - available data
	for i in range(NUM):
		current_mouse = miceData[i]
		mx, my = current_mouse.x, current_mouse.y
		mang = current_mouse.ang
		enemy_x, enemy_y, enemy_dist = getClosestEnemyInRmap(mx, my, reconMap)
		assert enemy_x != -1 and enemy_y != -1 
		astar_path = path_finding.astar(mx, my, enemy_x, enemy_y, reconMap, WORLD_HEIGHT, WORLD_WIDTH)
		miceMoves[i].type = path_finding.get_move(astar_path, mx, my, mang)