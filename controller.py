import numpy as np
from config import DEAD_ZONE, K_GOAL, K_FORM, K_AVOID, K_OBS, SAFE_DIST, DANGER_DIST

def to_goal(robot, goal, k=K_GOAL):
    dir_vec = goal - robot.pos
    dist = np.linalg.norm(dir_vec)
    if dist < 0.1:
        return np.array([0.0, 0.0])
    return dir_vec / dist * k

def to_formation(robot, target_pos, k=K_FORM):
    diff = target_pos - robot.pos
    dist = np.linalg.norm(diff)
    if dist < DEAD_ZONE:
        return np.array([0.0, 0.0])
    return diff * k

def avoid_neighbors(robot, robots, k=K_AVOID, safe_dist=SAFE_DIST):
    force = np.array([0.0, 0.0])
    for other in robots:
        if other is robot:
            continue
        diff = robot.pos - other.pos
        dist = np.linalg.norm(diff)
        if dist < safe_dist and dist > 0.01:
            force += diff / (dist * dist) * k
    return force

def avoid_obstacle(robot, obstacle, k=K_OBS, danger_dist=DANGER_DIST):
    if obstacle is None:
        return np.array([0.0, 0.0])
    diff = robot.pos - obstacle.pos
    dist = np.linalg.norm(diff)
    if dist < danger_dist and dist > 0.01:
        return diff / (dist * dist) * k
    return np.array([0.0, 0.0])

def resolve_collision(robot, obstacle):
    if obstacle is None:
        return
    dist = np.linalg.norm(robot.pos - obstacle.pos)
    if dist < obstacle.radius:
        direction = robot.pos - obstacle.pos
        if np.linalg.norm(direction) > 0:
            direction = direction / np.linalg.norm(direction)
        else:
            direction = np.array([1.0, 0.0])
        robot.pos = obstacle.pos + direction * (obstacle.radius + 0.1)