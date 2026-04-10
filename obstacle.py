import numpy as np
import random
from config import NUM_OBSTACLES_RANDOM, OBSTACLE_RADIUS_RANDOM

class Obstacle:
    def __init__(self, x, y, radius):
        self.pos = np.array([x, y], dtype=float)
        self.radius = radius


def generate_random_obstacles(num_robots, starts, goal, field_size=20):
    obstacles = []
    min_dist_to_robot = 1.5
    min_dist_to_goal = 1.5
    min_dist_between = 1.5
    
    num_obs = random.randint(*NUM_OBSTACLES_RANDOM)
    
    for _ in range(num_obs):
        radius = random.uniform(*OBSTACLE_RADIUS_RANDOM)
        for attempt in range(100):
            x = random.uniform(2, field_size - 2)
            y = random.uniform(2, field_size - 2)
            ok = True
            
            for s in starts[:num_robots]:
                if np.hypot(x - s[0], y - s[1]) < min_dist_to_robot + radius:
                    ok = False
                    break
            
            if np.hypot(x - goal[0], y - goal[1]) < min_dist_to_goal + radius:
                ok = False
            
            for o in obstacles:
                if np.hypot(x - o.pos[0], y - o.pos[1]) < min_dist_between + radius + o.radius:
                    ok = False
                    break
            
            if ok:
                obstacles.append(Obstacle(x, y, radius))
                break
    
    return obstacles