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
uint32[] types # see SensorData.msg for more info
bool[] flags
int32[] xs
int32[] ys

bool hasFlag

# TODO add sensor data as needed