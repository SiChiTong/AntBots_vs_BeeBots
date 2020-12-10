#!/usr/bin/env python3
import rospy
import numpy as np

from mouse_description.msg import MouseCommand
from . import path_finding
import copy

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
	global ISANT, NUM, lastxs, lastys, currentxs, currentys
	ISANT = isant
	NUM = numMice

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

	# Compute some moves
	if ISANT:
		for i in range(NUM):
			current_mouse = miceData[i]
			mx, my = current_mouse.x, current_mouse.y
			mang = current_mouse.ang
			
			types = current_mouse.types
			xs = current_mouse.xs
			ys = current_mouse.ys
	
			# init current x and y
			currentxs[i] = mx
			currentys[i] = my

			# created an updated recon map with radius information here:
			updatedReconMap = copy.deepcopy(reconMap)

			# update map with all vision
			for j in range(len(types)):
				newx, newy = xs[j], ys[j]	
				updatedReconMap[newx][newy] = types[j]

			# delete old one
			#print(lastxs[i])
			#print(currentxs[i])
			#print(lastys[i])
			#print(currentys[i])
			if lastxs[i] != -1 and lastys[i] != -1:
				if currentxs[i] != lastxs[i] or currentys[i] != lastys[i]:
					print("HIT")
					updatedReconMap[lastxs[i]][lastys[i]] = ' '

			lastxs[i] = currentxs[i]
			lastys[i] = currentys[i]

			reconMap = updatedReconMap

			print(updatedReconMap)

			if 'AF' in reconMap[mx][my]:
				# HAS FLAG
				print("ANT GOING HOME!")
				close_list = path_finding.astar(mx, my, myFlag[0], myFlag[1], reconMap, WORLD_HEIGHT, WORLD_WIDTH)
			else:
				# DOES NOT HAVE FLAG
				print("ANT GOING IN!")
				close_list = path_finding.astar(mx, my, enemyFlag[0], enemyFlag[1], reconMap, WORLD_HEIGHT, WORLD_WIDTH)
						
			nextx, nexty = 0, 0

			for close_node in close_list:
				if close_node[4] == mx and close_node[5] == my:
					nextx = close_node[0] - mx
					nexty = close_node[1] - my

			if (nextx == 1):
				#go EAST
				if mang == 0:
					miceMoves[i].type = MouseCommand.FORWARD
				else:
					miceMoves[i].type = MouseCommand.LEFT
			elif (nextx == -1):
				#go WEST
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

			types = current_mouse.types
			xs = current_mouse.xs
			ys = current_mouse.ys
			
			# init current x and y
			currentxs[i] = mx
			currentys[i] = my

			# created an updated recon map with radius information here:
			updatedReconMap = copy.deepcopy(reconMap)

			# update map with all vision
			for j in range(len(types)):
				newx, newy = xs[j], ys[j]	
				updatedReconMap[newx][newy] = types[j]

			# delete old one
			#print(lastxs[i])
			#print(currentxs[i])
			#print(lastys[i])
			#print(currentys[i])
			if lastxs[i] != -1 and lastys[i] != -1:
				if currentxs[i] != lastxs[i] or currentys[i] != lastys[i]:
					updatedReconMap[lastxs[i]][lastys[i]] = ' '

			lastxs[i] = currentxs[i]
			lastys[i] = currentys[i]

			reconMap = updatedReconMap

			print(updatedReconMap)

			if 'BF' in reconMap[mx][my]:
				# HAS FLAG
				print("BEE GOING HOME!")
				close_list = path_finding.astar(mx, my, myFlag[0], myFlag[1], reconMap, WORLD_HEIGHT, WORLD_WIDTH)
			else:
				# DOES NOT HAVE FLAG
				print("BEE GOING IN!")
				close_list = path_finding.astar(mx, my, enemyFlag[0], enemyFlag[1], reconMap, WORLD_HEIGHT, WORLD_WIDTH)
							
			nextx, nexty = 0, 0

			for close_node in close_list:
				if close_node[4] == mx and close_node[5] == my:
					nextx = close_node[0] - mx
					nexty = close_node[1] - my

			if (nextx == 1):
				#go EAST
				if mang == 0:
					miceMoves[i].type = MouseCommand.FORWARD
				else:
					miceMoves[i].type = MouseCommand.LEFT
			elif (nextx == -1):
				#go WEST
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

	#reconMap = updatedReconMap