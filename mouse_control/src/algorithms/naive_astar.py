#!/usr/bin/env python3
import rospy

from mouse_description.msg import MouseCommand
from . import path_finding

# constants
WORLD_HEIGHT = rospy.get_param('/WORLD_HEIGHT')
WORLD_WIDTH = rospy.get_param('/WORLD_WIDTH')

# code to run before node even starts
print('Hello! I\'m the naive A* algorithm!')

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

def computeMoves(miceMoves, score, miceData, reconMap):
	# miceMoves - modify this with the moves u wanna do
	# score - current score, see mouse_control/msg/Score.msg
	# miceData - telemetry data from each mouse, see mouse_description/msg/MouseData.msg
	# TODO add sensor data from each mouse to do level 2 knowledge

	# Level 1: Omnisas pfcient - available data
	# reconMap - xy-indexed, see mouse_control/msg/Omniscience.msg (also can print out in god node leakMap)

	# First call should not have any flags captured, so can grab flag locations from there
	# keep in mind these are base locations, need to re-search if flag is stolen
    if not (myFlag and enemyFlag):
        computeFlags(reconMap)

<<<<<<< HEAD
    # Compute some moves
    if ISANT:
        for i in range(NUM):
            current_mouse = miceData[i]
            mx, my = current_mouse.x, current_mouse.y
            mang = current_mouse.ang
            
            close_list = path_finding.astar(mx, my, enemyFlag[0], enemyFlag[1], reconMap, WORLD_HEIGHT, WORLD_WIDTH)
=======
	# Compute some moves
	if ISANT:
		for i in range(NUM):
			current_mouse = miceData[i]
			mx, my = current_mouse.x, current_mouse.y
			mang = current_mouse.ang
			
			close_list = path_finding.astar(mx, my, enemyFlag[0], enemyFlag[1], reconMap, WORLD_HEIGHT, WORLD_WIDTH)
>>>>>>> 15ee43b5ddf10fd463e1a7beffc1952bf484aebb
						
            nextx, nexty = 0, 0

            for close_node in close_list:
                if close_node[4] == mx and close_node[5] == my:
                    nextx = close_node[0] - mx
                    nexty = close_node[1] - my

            if (nextx == 1):
                # go EASTboulders_large
                if mang == 0:
                	miceMoves[i].type = MouseCommand.FORWARD
                else:
                    miceMoves[i].type = MouseCommand.LEFT
            elif (nextx == -1):
                # go WEST
                if mang == 2:
                    miceMoves[i].type = MouseCommand.FORWARD
                else:
                    miceMoves[i].type = MouseCommand.LEFT
            elif (nexty == 1):
                #go NORTH
                if mang == 1:
                    miceMoves[i].type = MouseCommand.FORWARD
                else:
                    miceMoves[i].type = MouseCommand.LEFT
            elif (nexty == -1):
                #go SOUTH
                if mang == 3:
                    miceMoves[i].type = MouseCommand.FORWARD
                else:
                    miceMoves[i].type = MouseCommand.LEFT
    else:
        for i in range(NUM):
            current_mouse = miceData[i]
            mx, my = current_mouse.x, current_mouse.y
            mang = current_mouse.ang
            
            close_list = path_finding.astar(mx, my, enemyFlag[0], enemyFlag[1], reconMap, WORLD_HEIGHT, WORLD_WIDTH)
						
            nextx, nexty = 0, 0

            for close_node in close_list:
                if close_node[4] == mx and close_node[5] == my:
                    nextx = close_node[0] - mx
                    nexty = close_node[1] - my

            if (nextx == 1):
                # go EASTboulders_large
                if mang == 0:
                	miceMoves[i].type = MouseCommand.FORWARD
                else:
                    miceMoves[i].type = MouseCommand.LEFT
            elif (nextx == -1):
                # go WEST
                if mang == 2:
                    miceMoves[i].type = MouseCommand.FORWARD
                else:
                    miceMoves[i].type = MouseCommand.LEFT
            elif (nexty == 1):
                #go NORTH
                if mang == 1:
                    miceMoves[i].type = MouseCommand.FORWARD
                else:
                    miceMoves[i].type = MouseCommand.LEFT
            elif (nexty == -1):
                #go SOUTH
                if mang == 3:
                    miceMoves[i].type = MouseCommand.FORWARD
                else:
                    miceMoves[i].type = MouseCommand.LEFT