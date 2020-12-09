# Mouse Data

# Constants
uint8 EAST=0
uint8 NORTH=1 # positive Y-axis
uint8 WEST=2
uint8 SOUTH=3

# Position
bool aligned
int32 x
int32 y
uint8 ang

# Processed Sensor Data
string[] types # see mouse_control/msg/Omniscience.msg for more info (will have empty cells too)
int32[] xs
int32[] ys

# TODO add sensor data as needed