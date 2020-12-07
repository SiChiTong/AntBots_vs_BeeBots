# determines path to take based on A* search algorithm
def astar(sx, sy, ex, ey, reconMap):
    found_path = False

    # F = G + H
    g_list = [None for i in range(1000)]
    h_list = [None for i in range(1000)]
    f_list = [None for i in range(1000)]

    open_list = []
    close_list = []
    parent_list = [None for i in range(1000)]
    
    start_id = int(cantor_pairing(sx, sy))
    f_list[start_id] = 0
    g_list[start_id] = 0
    h_list[start_id] = 0
    
    
    open_list.append((sx, sy))
    
    while open_list:
        # temp variables for minimum F and minimum tuple
        min_f = float('inf')
        min_tuple = (-1, -1)

        for o in open_list:
            if (o == None):
                continue
            ox, oy = o[0], o[1]
            o_id = int(cantor_pairing(ox, oy))

            if min_f > f_list[o_id]:
                min_f = f_list[o_id]
                min_tuple = o
        
        m = int(cantor_pairing(min_tuple[0], min_tuple[1]))
        # pop min f tuple from open list
        open_list.remove(min_tuple)
        
        # add min f tuple to close list
        close_list.append(min_tuple)
        
        mx, my = min_tuple[0], min_tuple[1]
        # generate the 4 successor tuples
        succ1 = (mx - 1, my)
        succ2 = (mx + 1, my)
        succ3 = (mx, my - 1)
        succ4 = (mx, my + 1)

        succ_list = [succ1, succ2, succ3, succ4]

        for succ in succ_list:
            succx, succy = succ[0], succ[1]
            s_id = int(cantor_pairing(succx, succy))
            minx, miny = min_tuple[0], min_tuple[1]
            min_id = int(cantor_pairing(minx, miny))

            if not isValid(succ, reconMap):
                continue
          
            if succx == ex and succy == ey:
                found_path = True
                print("DONE!")
                return parent_list

            elif (succx, succy) not in close_list:

                g_value = g_list[min_id] + 1
                h_value = manhattan(succx, succy, ex, ey)
                f_value = g_value + h_value

                if (succ not in open_list or f_value < f_list[s_id]):
                    g_list[s_id] = g_value
                    h_list[s_id] = h_value
                    f_list[s_id] = f_value
                    
                    open_list.append(succ)
                    parent_list[s_id] = o

    if (found_path == False):
        print("error: no path found")
    
    print("done!")
    return parent_list

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
    if reconMap[x][y] == "#":
        return False
    return True