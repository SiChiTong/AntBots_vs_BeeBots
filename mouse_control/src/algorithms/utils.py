


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
    