#!/usr/bin/env python3
import rospy

import heapq
import random
import copy

from mouse_description.msg import MouseCommand

# matstar but Level 3

# constants
WORLD_HEIGHT = rospy.get_param('/WORLD_HEIGHT')
WORLD_WIDTH = rospy.get_param('/WORLD_WIDTH')

# code to run before node even starts
print('Hello! I\'m the m3tstar algorithm!')

# private variables
myFlag = None
enemyFlag = None
dxys = [(1,0),(0,1),(-1,0),(0,-1)] # direction 0-3
moves = [MouseCommand.FORWARD, MouseCommand.LEFT, MouseCommand.RIGHT, MouseCommand.STOP]

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
	global ISANT, NUM, reconMaps
	ISANT = isant
	NUM = numMice
	reconMaps = [[[' ' for j in range(WORLD_HEIGHT)] for i in range(WORLD_WIDTH)] for k in range(NUM)]

def computeMoves(miceMoves, score, miceData, omniMap):
	# First call should not have any flags captured, so can grab flag locations from there
	# keep in mind these are base locations, need to re-search if flag is stolen
	if not (myFlag and enemyFlag):
		computeFlags(omniMap)

	# still has potential for deadlock
	for i in range(NUM):
		reconMap = reconMaps[i]
		# Level 3: reset
		for x in range(WORLD_WIDTH):
			for y in range(WORLD_HEIGHT):
				if reconMap[x][y] != '#':
					reconMap[x][y] = ' '
		# Level 3: only use my sensor data
		data = miceData[i]
		for t, x, y in zip(data.types, data.xs, data.ys):
			reconMap[x][y] = t

		# take into account tag radius
		# assumes map surrounded by walls
		filterMap = [[' ' for j in range(WORLD_HEIGHT)] for i in range(WORLD_WIDTH)]
		if ISANT:
			for x in range(WORLD_WIDTH):
				for y in range(WORLD_HEIGHT):
					if reconMap[x][y] == '#' or 'A' in reconMap[x][y]:
						filterMap[x][y] = '#'
					elif 'B' in reconMap[x][y]:
						if y < WORLD_HEIGHT//2: # taggable
							continue
						else:
							filterMap[x][y] = '#'
							a = int(reconMap[x][y][0])
							for t in (a,(a+1)%4,(a-1)%4):
								dx, dy = dxys[t]
								nx, ny = x+dx, y+dy
								if ny >= WORLD_HEIGHT//2:
									filterMap[nx][ny] = '#'
		else:
			for x in range(WORLD_WIDTH):
				for y in range(WORLD_HEIGHT):
					if reconMap[x][y] == '#' or 'B' in reconMap[x][y]:
						filterMap[x][y] = '#'
					elif 'A' in reconMap[x][y]:
						if y >= WORLD_HEIGHT//2: # taggable
							continue
						else:
							filterMap[x][y] = '#'
							a = int(reconMap[x][y][0])
							for t in (a,(a+1)%4,(a-1)%4):
								dx, dy = dxys[t]
								nx, ny = x+dx, y+dy
								if ny < WORLD_HEIGHT//2:
									filterMap[nx][ny] = '#'

		if 'F' not in reconMap[x][y] and random.random() < 0.2:
			miceMoves[i].type = random.choice(moves)
			continue

		x, y, ang = miceData[i].x, miceData[i].y, miceData[i].ang
		if 'F' in omniMap[x][y]:
			miceMoves[i].type = astar((x, y, ang), (myFlag[0], myFlag[1], -1), filterMap)
		else:
			miceMoves[i].type = astar((x, y, ang), (enemyFlag[0], enemyFlag[1], -1), filterMap)

def astar(start, end, rmap):
	# format is (x,y,ang)
	# put -1 if don't care about that one matching

	dist = {}
	prev = {}

	dist[start] = 0
	prev[start] = start

	pq = []
	heapq.heappush(pq, (0, start))

	fend = None

	while pq:
		_, cell = heapq.heappop(pq)
		cost = dist[cell]

		# goal check
		goal = True
		for c1, c2 in zip(cell, end):
			if not (c2 == -1 or c1 == c2):
				goal = False
		if goal:
			fend = cell # found end
			break

		# compute neighbors
		neighs = []
		neighs.append((cell[0], cell[1], (cell[2]+1)%4)) # left
		neighs.append((cell[0], cell[1], (cell[2]-1)%4)) # right
		dx, dy = dxys[cell[2]]
		nx, ny = cell[0]+dx, cell[1]+dy
		if nx >= 0 and nx < WORLD_WIDTH and ny >= 0 and ny < WORLD_HEIGHT:
			if rmap[nx][ny] == ' ' or rmap[nx][ny] == 'F': # check wall
				neighs.append((nx, ny, cell[2])) # forward

		# add neighbors
		for ncell in neighs:
			ncost = dist[ncell] if ncell in dist else float('inf')
			if cost + 1 < ncost:
				heapq.heappush(pq, (cost+1+heuristic(ncell,end,rmap), ncell))
				dist[ncell] = cost+1
				prev[ncell] = cell
			elif cost+1 == ncost and random.random() < 0.5:
				prev[ncell] = cell

	# path reconstruction
	if not fend:
		return MouseCommand.STOP
	while prev[fend] != start:
		fend = prev[fend]
	if fend[0] != start[0] or fend[1] != start[1]:
		return MouseCommand.FORWARD
	elif (start[2]+1)%4 == fend[2]:
		return MouseCommand.LEFT
	else:
		return MouseCommand.RIGHT

def heuristic(start, end, rmap):
	h = 0
	for c1, c2 in zip(start, end):
		if c2 != -1:
			h += abs(c1-c2)
	return h