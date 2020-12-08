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
<<<<<<< HEAD:mouse_control/src/path_finding.py
    if reconMap[x][y] == 'A':
        return False
    if reconMap[x][y] == 'B':
        return False
    return True
=======
    return True

# SAVED CODE FROM MOTHERSHIP

"""
                    flagx, flagy = -1, -1
                    for x in range(WORLD_WIDTH):
                        for y in range(WORLD_HEIGHT):
                            if reconMap[x][y] == 'F':
                                
                                flagx, flagy = x, y
                                print("FLAG HERE:")
                                print(flagx, flagy)
                    
                    # if flagx == -1 and flagy == -1:
                    #   print("error: no flag detected")
                    
    
                    for i in range(NUM):
                        current_mouse = miceData[i]
                        mx, my = current_mouse.x, current_mouse.y
                        mang = current_mouse.ang

                        path_array = path_finding.astar(mx, my, flagx, flagy, reconMap, WORLD_HEIGHT, WORLD_WIDTH)
                        print(path_array)

                        nextx, nexty = 0, 0
                        
                        if (path_array[mx + 1][my] == (mx, my)):
                            nextx = 1
                        elif (path_array[mx - 1][my] == (mx, my)):
                            nextx = -1
                        elif (path_array[mx][my + 1] == (mx, my)):
                            nexty = 1
                        elif (path_array[mx][my - 1] == (mx, my)):
                            nexty = -1
                        else:
                            print("error: bad path")
                        
                        miceMoves[i] = MouseCommand()

                        print("NEXT")
                        print(nextx)
                        print(nexty)
                        ## ONLY TURNING RIGHT FOR TESTING, NOT CORRECTLY TURNING / MOVING YET
                        if (nextx == 1):
                            # go right
                            print("HIT RIGHT")
                            #if (mang[num])
                            miceMoves[i].type = MouseCommand.LEFT
                            print("MANG")
                            print(mang)
                        elif (nextx == -1):
                            # go left
                            print("HIT LEFT")
                            miceMoves[i].type = MouseCommand.LEFT
                        elif (nexty == 1):
                            #go up
                            miceMoves[i].type = MouseCommand.LEFT
                        else:
                            #go backwards
                            miceMoves[i].type = MouseCommand.LEFT
"""
>>>>>>> 75dcd3af12bbb159039ec58bfb2f1fc0cb42e646:mouse_control/src/algorithms/path_finding.py
