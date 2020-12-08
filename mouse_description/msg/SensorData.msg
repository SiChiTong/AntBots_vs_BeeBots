# SensorData

uint8 AIR=0
uint8 WALL=1
uint8 ANT=2
uint8 BEE=3

uint8 leftType # what am I seeing?
bool leftFlag # is there also a flag?
int32 leftDist # distance in number of squares (0 if too far/invalid)

uint8 rightType
bool rightFlag
int32 rightDist

uint8 centerType
bool centerFlag
int32 centerDist

bool hasFlag

# TODO add more as needed