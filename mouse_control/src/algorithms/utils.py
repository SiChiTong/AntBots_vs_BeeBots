


def in_my_half(is_ant, height, state):
    return (is_ant and state.y < (height / 2)) or (not is_ant and state.y >= (height / 2))

def update_level_two_map(reconMap, types, xs, ys, WORLD_WIDTH, WORLD_HEIGHT):
    # clear the memory of any B or A
    for x in range(WORLD_WIDTH):
        for y in range(WORLD_HEIGHT):
            if 'B' in reconMap[x][y] or 'A' in reconMap[x][y]:
                reconMap[x][y] = ' '

    # add obstacle  and new sensing data
    for j in range(len(types)):
        newx, newy = xs[j], ys[j]
        if types[j] == '#' or 'B' in types[j] or 'A' in types[j]: 
            reconMap[newx][newy] = types[j]
    
def updateHiveReconMap(reconMap, NUM, WORLD_WIDTH, WORLD_HEIGHT, miceData, enemyFlag, myFlag):
	hive_mind = dict()
	for i in range(NUM):
		current_mouse = miceData[i]
		types = current_mouse.types
		xs = current_mouse.xs
		ys = current_mouse.ys
		for j in range(len(types)):
			x, y = xs[j], ys[j]
			hive_mind[(x, y)] = types[j]

	# clear the map besides obstacles
	for x in range(WORLD_WIDTH):
		for y in range(WORLD_HEIGHT):
			if reconMap[x][y] != '#':
				reconMap[x][y] = ' '

	# make a new map with the xs, ys, and types of 'A' and 'B' and 'F' added in
	for p, t in hive_mind.items():
		x, y = p
		reconMap[x][y] = t  
	
	# add back the flags
	reconMap[enemyFlag[0]][enemyFlag[1]] = 'F'
	reconMap[myFlag[0]][myFlag[1]] = 'F'