import numpy as np
from config import DT, GOAL, K_GOAL, K_FORM, K_AVOID, K_OBS, SAFE_DIST, DANGER_DIST, STEPS
from robot import Robot
from controller import to_goal, to_formation, avoid_neighbors, avoid_obstacle, resolve_collision
from formation import get_formation_targets

def simulate_step(robots, goal, obstacle, formation_offsets, k_form_current):
    leader = robots[0]
    
    if len(leader.trajectory) > 1:
        leader_vel = leader.pos - leader.trajectory[-2]
    else:
        leader_vel = np.array([0.0, 0.0])
    
    target_positions = get_formation_targets(leader.pos, leader_vel, formation_offsets, goal)
    
    forces = []
    for i, robot in enumerate(robots):
        force = np.array([0.0, 0.0])
        
        # Проверка близости к препятствию
        near_obstacle = False
        if obstacle:
            dist_to_obs = np.linalg.norm(robot.pos - obstacle.pos)
            if dist_to_obs < DANGER_DIST * 1.5:
                near_obstacle = True
        
        # Выбираем вес формации: если у препятствия или мягкий старт
        if near_obstacle:
            k_form_local = k_form_current * 0.5
        else:
            k_form_local = k_form_current
        
        if robot.is_leader:
            force += to_goal(robot, goal)
        else:
            if i < len(target_positions):
                force += to_formation(robot, target_positions[i], k_form_local)
        
        force += avoid_neighbors(robot, robots, K_AVOID) * 0.5
        force += avoid_obstacle(robot, obstacle, K_OBS)
        
        forces.append(force)
    
    for robot, force in zip(robots, forces):
        robot.update(force, DT)
    
    # Принудительное разрешение коллизий с препятствием
    for robot in robots:
        resolve_collision(robot, obstacle)

def run_simulation(robots, goal, obstacle, formation_offsets, steps=STEPS):
    error_history = []
    min_dist_history = []
    
    SOFT_START_STEPS = 200
    K_FORM_FULL = K_FORM
    K_FORM_SOFT = K_FORM * 0.5
    
    for step in range(steps):
        if step < SOFT_START_STEPS:
            k_form_current = K_FORM_SOFT
        else:
            k_form_current = K_FORM_FULL
        
        simulate_step(robots, goal, obstacle, formation_offsets, k_form_current)
        
        # Ошибка формации
        leader = robots[0]
        target_positions = get_formation_targets(leader.pos, np.array([0.0, 0.0]), formation_offsets, goal)        

        errors = []
        for i, robot in enumerate(robots):
            if not robot.is_leader and i < len(target_positions):
                err = np.linalg.norm(robot.pos - target_positions[i])
                errors.append(err)
        error_history.append(np.mean(errors) if errors else 0)
        
        # Минимальное расстояние между роботами
        min_dist = float('inf')
        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                dist = np.linalg.norm(robots[i].pos - robots[j].pos)
                if dist < min_dist:
                    min_dist = dist
        min_dist_history.append(min_dist)
        
        # Остановка при достижении цели
        if np.linalg.norm(leader.pos - goal) < 0.5:
            break
    
    return error_history, min_dist_history