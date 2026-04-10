import numpy as np

class Robot:
    def __init__(self, x, y, is_leader=False):
        self.pos = np.array([x, y], dtype=float)
        self.vel = np.array([0.0, 0.0])  # добавить
        self.is_leader = is_leader
        self.trajectory = [self.pos.copy()]
    
    def update(self, force, dt, damping=0.8):
        self.vel = self.vel + force * dt
        self.vel = self.vel * damping
        self.pos = self.pos + self.vel * dt
        self.trajectory.append(self.pos.copy())