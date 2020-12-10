#!/usr/bin/env python3
import rospy

from mouse_description.msg import MouseCommand

# Import other algorithms
import algorithms.template as template
import algorithms.tagger as tagger
import algorithms.attacker_tom as attacker_tom

# constants
WORLD_HEIGHT = rospy.get_param('/WORLD_HEIGHT')
WORLD_WIDTH = rospy.get_param('/WORLD_WIDTH')

# code to run before node even starts
print('Hello! I\'m the 1v1 test algorithm!')

# private variables
myFlag = None
enemyFlag = None
reconMap = [[' ' for j in range(WORLD_HEIGHT)] for i in range(WORLD_WIDTH)]

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

	# initialize other algs
	attacker_tom.initAlg(isant, numMice, eps=0.25)

def computeMoves(miceMoves, score, miceData, omniMap):
	# miceMoves - modify this with the moves u wanna do
	# score - current score, see mouse_control/msg/Score.msg

	# Level 1: Omniscient - available data
	# omniMap - xy-indexed, see mouse_control/msg/Omniscience.msg (also can print out in god node leakMap)

	# Level 2: Hivemind - available data
	# omniMap - just use your half and ignore the rest
	# miceData - telemetry data from each mouse, see mouse_description/msg/MouseData.msg

	# First call should not have any flags captured, so can grab flag locations from there
	# keep in mind these are base locations, need to re-search if flag is stolen
	if not (myFlag and enemyFlag):
		computeFlags(omniMap)

	# Compute some moves
	# keep in mind ants go first, then bees, but tag and point logic doesn't apply until bees are done
	roles = [attacker_tom]
	for i in range(NUM):
		roles[i%len(roles)].computeMouseMove(i, miceMoves, score, miceData, omniMap, reconMap, myFlag, enemyFlag)