#!/usr/bin/env python3
import copy 
from collections import defaultdict
import heapq

from mouse_description.msg import MouseCommand

# determines path to take based on A* search algorithm
def astar(sx, sy, ex, ey, reconMap, height, width):
    found_path = False

    # F = G + H
    g_array = [[None for j in range(height)] for i in range(width)]
    h_array = [[None for j in range(height)] for i in range(width)]
    f_array = [[None for j in range(height)] for i in range(width)]
    
    open_list = []
    close_list = []
    parent_array = [[None for j in range(height)] for i in range(width)]
    
    f_array[sx][sy] = 0
    g_array[sx][sy] = 0
    h_array[sx][sy] = 0
    
    open_list.append((sx, sy, 0, 0, -1, -1))
    
    while open_list:
        # temp variables for minimum F and minimum tuple
        min_f = float('inf')
        # MIN_TUPLE = (x, y, F, G, parentx, parenty)
        min_tuple = (-1, -1, -1, -1, -1, -1)

        for open_node in open_list:
            openx, openy, openf = open_node[0], open_node[1], open_node[2]

            if min_f > openf:
                min_f = openf
                min_tuple = open_node
        
        # pop min f tuple from open list
        open_list.remove(min_tuple)
        
        # add min f tuple to close list
        close_list.append(min_tuple)
        
        if min_tuple[0] == ex and min_tuple[1] == ey:
            return close_list
            #found goal, backtrack for path

        mx, my = min_tuple[0], min_tuple[1]
        # generate the 4 successor tuples
        succ1 = (mx - 1, my, -1, -1, mx, my)
        succ2 = (mx + 1, my, -1, -1, mx, my)
        succ3 = (mx, my - 1, -1, -1, mx, my)
        succ4 = (mx, my + 1, -1, -1, mx, my)

        succ_list = [succ1, succ2, succ3, succ4]

        for succ in succ_list:
            succx, succy = succ[0], succ[1]

            if not isValid(succ, reconMap):
                continue
          
            for closed_child in close_list:
                if succ[0] == closed_child[0] and succ[1] == closed_child[1]:
                    continue
 
           
            g_value = min_tuple[3] + 1
            h_value = manhattan(succx, succy, ex, ey)
            f_value = g_value + h_value
            
            child = (succx, succy, f_value, g_value, mx, my)

            for open_check in open_list:
                if child[0] == open_check[0] and child[1] == open_check[1] and child[3] > open_check[3]:
                    continue

            open_list.append(child)

    if (found_path == False):
        print("ERROR: NO PATH FOUND IN A*!")
    
    return parent_array

# calculates and returns the manhattan distance between
# a start x, y and end x, y. 
def manhattan(sx, sy, ex, ey):
    return abs(sx - ex) + abs(sy - ey)

# calculates and returns the cantor pairing number of 
# a x, y coordinate
def cantor_pairing(x, y):
    return 0.5 * (x + y) * (x + y + 1) + y

# determines whether (x, y) tuple is legal: not OOB and not a wall
def isValid(tuple, reconMap):
    x, y = tuple[0], tuple[1]
    if reconMap[x][y] == '#':
        return False
    if reconMap[x][y] == 'A':
        return False
    if reconMap[x][y] == 'B':
        return False
    return True


def get_move(astar_path, mx, my, mang):
    nextx, nexty = 0, 0
    for node in astar_path:
        if node[4] == mx and node[5] == my:
            nextx = node[0] - mx
            nexty = node[1] - my

    if (nextx == 1):
        # go EAST
        if mang == 0:
            return MouseCommand.FORWARD
        else:
            return MouseCommand.LEFT
    elif (nextx == -1):
        # go WEST
        if mang == 2:
            return MouseCommand.FORWARD
        else:
            return MouseCommand.LEFT
    elif (nexty == 1):
        #go NORTH
        if mang == 1:
            return MouseCommand.FORWARD
        else:
            return MouseCommand.LEFT
    elif (nexty == -1):
        #go SOUTH
        if mang == 3:
            return MouseCommand.FORWARD
        else:
            return MouseCommand.LEFT

class RobotState(object):
    def __init__(x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __eq__(self, other):
        return other and self.x == other.x and self.y == other.y and self.theta == other.theta

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(): 
        return hash((self.x, self.y, self.theta))

def djistrka(sx, sy, st, ex, ey, rMap, height, width):
    start_state = RobotState(sx, sy, st)
    q, visited, cost_map, parent = [(0, start_state)], set(), {}, {}
    while q:
        (cost, state) = heapq.heappop(q)
        if state in visited: continue
        visited.add(state)
        for (action, neighbor) in get_valid_neighbors(state, rMap, height, width):
            if neighbor in visited: continue 
            if neighbor.x == ex and neighbor.y == ey:
                return get_dijstrka_trajectory(start=start, end=neighbor, parent=parent)
            prev_cost = cost_map[neighbor]
            new_cost = cost + 1
            if prev_cost is None or new_cost < prev_cost:
                cost_map[neighbor] = new_cost
                parent[neighbor] = (action, state)
                heappush(q, (new_cost, neighbor))
    return None 

def get_dijstrka_trajectory(start, end, parent):
    action, state, trajectory = None, end, []
    while state != start:
        action, parent_state = parent[state]
        trajectory.append((action, state))
        state = parent_state 
    return list(reversed(trajectory))


def get_valid_neighbors(state, rMap, height, width):
    def is_valid_state(state):
        sc = rMap[state.x][state.y]
        return 0 <= state.x < width and 0 <= state.y < height \
            and (' ' == sc or 'F' == sc)

    actions = [MouseCommand.LEFT, MouseCommand.RIGHT, MouseCommand.FORWARD, MouseCommand.STOP]    
    neighbors = []
    for a in actions: 
        neighbor = state.copy()
        if a == MouseCommand.LEFT:
            neighbor.theta = (state.theta + 3) % 4
        elif a == MouseCommand.RIGHT:
            neighbor.theta = (state.theta + 1) % 4
        elif a == MouseCommand.STOP:
            pass 
        else: 
            if state.theta == 0: #E
                neighbor.x += 1
            elif state.theta == 1: #N
                neighbor.y += 1
            elif state.theta == 2: #W
                neighbor.x -= 1
            else: #S
                neighbor.y -= 1   
        if is_valid_state(neighbor):
            neighbors.append((a, neighbor))     
    return neighbors

def print_astar_path(astar_path, sx, sy, ex, ey, rmap):	
    aMap = copy.deepcopy(rMap) 
    for nodes in astar_path:
        nodes[0], nodes[1],


    for y in reversed(range(WORLD_HEIGHT)):
		for x in range(WORLD_WIDTH):
			print(f'{aMap[x][y]:^5}', end="")
		print()