class RobotState(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __eq__(self, other):
        return other and self.x == other.x and self.y == other.y and self.theta == other.theta

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self): 
        return hash((self.x, self.y, self.theta))

    def __copy__(self):
        return RobotState(self.x, self.y, self.theta)

    def __lt__(self, other):
        return other.x < self.x or other.y < self.y

    def __repr__(self):
        return f"(x: {self.x}, y: {self.y}, t: {self.theta})"