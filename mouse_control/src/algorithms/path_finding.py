# determines path to take based on A* search algorithm
def astar(sx, sy, ex, ey, reconMap, height, width):
    found_path = False

    # F = G + H
    #g_list = [None for i in range(1000)]
    #h_list = [None for i in range(1000)]
    #f_list = [None for i in range(1000)]
    g_array = [[None for j in range(height)] for i in range(width)]
    h_array = [[None for j in range(height)] for i in range(width)]
    f_array = [[None for j in range(height)] for i in range(width)]
    
    open_list = []
    close_list = []
    #parent_list = [None for i in range(1000)]
    parent_array = [[None for j in range(height)] for i in range(width)]
    
    f_array[sx][sy] = 0
    g_array[sx][sy] = 0
    h_array[sx][sy] = 0
    
    open_list.append((sx, sy))
    
    while open_list:
        # temp variables for minimum F and minimum tuple
        min_f = float('inf')
        min_tuple = (-1, -1)

        for o in open_list:
            if (o == None):
                continue
            ox, oy = o[0], o[1]

            if min_f > f_array[ox][oy]:
                min_f = f_array[ox][oy]
                min_tuple = o
        
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
                return parent_array

            elif (succx, succy) not in close_list:
                g_value = g_array[minx][miny] + 1
                h_value = manhattan(succx, succy, ex, ey)
                f_value = g_value + h_value

                if (succ not in open_list or f_value < f_array[succx][succy]):
                    g_array[succx][succy] = g_value
                    h_array[succx][succy] = h_value
                    f_array[succx][succy] = f_value
                    
                    open_list.append(succ)
                    parent_array[succx][succy] = o

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
    if reconMap[x][y] == '#':
        return False
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