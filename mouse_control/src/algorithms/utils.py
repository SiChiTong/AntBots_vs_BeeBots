


def in_my_half(is_ant, height, state):
    return (is_ant and state.y < (height / 2)) or (not is_ant and state.y >= (height / 2))