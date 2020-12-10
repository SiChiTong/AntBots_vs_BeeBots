#!/usr/bin/env python3
import copy 
from collections import defaultdict
import heapq
import random

from mouse_description.msg import MouseCommand
from .robot_state import RobotState

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
    
	# OPEN_NODE = (x, y, F, G, parentx, parenty)
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
    if 'F' in reconMap[x][y]:
        return True
    if 'A' in reconMap[x][y]:
        return False
    if 'B' in reconMap[x][y]:
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

def djistrka(start_state, end_state, rMap, height, width, path=True, ignore_theta=True, debug=False):
    q, visited, cost_map, parent = [(0, start_state)], set(), defaultdict(lambda: float('inf')), {}
    special_valid_cond = get_valid_goal_lambda(end_state)
    while q:
        (cost, state) = heapq.heappop(q)
        if debug:
            print(f"Cost: {cost} State: {state}, start: {start_state}, end: {end_state}, height: {height}, width: {width}")
        if state in visited: continue
        if state.x == end_state.x and state.y == end_state.y and (ignore_theta or state.theta == end_state.theta):
            if path: 
                return get_dijstrka_trajectory(start=start_state, end=state, parent=parent)
            else: 
                return cost
        visited.add(state)
        for (action, neighbor) in get_valid_neighbors(state, rMap, height, width, special_valid_cond=special_valid_cond):
            if neighbor in visited: continue 
            prev_cost = cost_map[neighbor]
            new_cost = cost + 1
            if prev_cost is None or new_cost < prev_cost:
                cost_map[neighbor] = new_cost
                parent[neighbor] = (action, state)
                heapq.heappush(q, (new_cost, neighbor))
    return None 

def find_closest_enemy(start_state, enemyChar, rMap, height, width):
    closest_enemy, nearest_distance = None, float('inf')
    for x in range(width):
        for y in range(height):
            if enemyChar in rMap[x][y]:
                et = int(rMap[x][y][0])
                enemy_state = RobotState(x, y, et)
                cost = djistrka(enemy_state, start_state, rMap, height, width, path=False, ignore_theta=True, debug=False)
                if cost is not None and cost < nearest_distance:
                    nearest_distance = cost 
                    closest_enemy = enemy_state
    return closest_enemy, nearest_distance

def manhattan_dist(s1, s2):
    return abs(s1.x - s2.x) + abs(s1.y - s2.y)

def move_away_from_enemy(start_state, enemy_state, rMap, height, width, old_action):
    best_action, best_dcost, best_mcost = 3, -float('inf'), float('inf')
    for (action, state) in get_valid_neighbors(start_state, rMap, height, width):
        if action == 3: continue
        new_dcost = djistrka(enemy_state, state, rMap, height, width, path=False, ignore_theta=False)
        new_mcost = manhattan_dist(enemy_state, state)
        print(f"new dcost: {new_dcost}, new_mcost: {new_mcost}, action: {action}, state: {state}, enemy_state: {enemy_state}, start_state: {start_state}")
        if best_dcost < new_dcost or (new_dcost == best_dcost and best_mcost < new_mcost): 
            best_dcost = new_dcost
            best_mcost = new_mcost
            best_action = action 
        if best_dcost == new_dcost and new_dcost == best_dcost and action == old_action:
            best_action = action

    print(f"move away action: {best_action}")
    return best_action, best_dcost, best_mcost

def get_dijstrka_trajectory(start, end, parent):
    action, state, trajectory = None, end, []
    while state != start:
        action, parent_state = parent[state]
        trajectory.append((action, state))
        state = parent_state 
    #for (a, s) in trajectory: print(f"Original A: {a} s: {s}")
    return list(reversed(trajectory))

def get_valid_goal_lambda(g):
    return lambda s: s.x == g.x and s.y == g.y

def get_valid_goal_state_lambda(g):
    return lambda s: g == s

LAMBDA_FALSE = lambda x: False

def get_valid_neighbors(state, rMap, height, width, special_valid_cond=LAMBDA_FALSE):
    def is_valid_forward_state(state):
        if state.x < 0 or state.x >= width or state.y < 0 or state.y >= height:
            return False
        sc = rMap[state.x][state.y]
        return ' ' == sc or 'F' == sc or special_valid_cond(state)

    actions = [MouseCommand.LEFT, MouseCommand.RIGHT, MouseCommand.FORWARD, MouseCommand.STOP]    
    neighbors = []
    for a in actions: 
        neighbor = copy.copy(state)
        if a == MouseCommand.LEFT:
            neighbor.theta = (state.theta + 1) % 4
            neighbors.append((a, neighbor))
        elif a == MouseCommand.RIGHT:
            neighbor.theta = (state.theta + 3) % 4
            neighbors.append((a, neighbor))
        elif a == MouseCommand.STOP:
            neighbors.append((a, neighbor))
        else: 
            if state.theta == 0: #E
                neighbor.x += 1
            elif state.theta == 1: #N
                neighbor.y += 1
            elif state.theta == 2: #W
                neighbor.x -= 1
            else: #S
                neighbor.y -= 1   
            if is_valid_forward_state(neighbor):
                neighbors.append((a, neighbor))     
    return neighbors
