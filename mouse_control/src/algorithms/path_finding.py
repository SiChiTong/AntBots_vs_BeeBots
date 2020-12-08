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
        # MIN_TUPLE = (x, y, F, G, parentx, parent y)
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