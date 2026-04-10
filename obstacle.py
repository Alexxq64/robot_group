import numpy as np

class Obstacle:
    def __init__(self, x, y, radius):
        self.pos = np.array([x, y], dtype=float)
        self.radius = radius