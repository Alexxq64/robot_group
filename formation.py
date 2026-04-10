import numpy as np

# Сглаженный угол для формации
_smoothed_angle = 0.0
_angle_alpha = 0.1

def rotate_formation(offsets, angle):
    """Поворачивает формацию на угол (радианы)"""
    rotated = []
    for offset in offsets:
        x = offset[0] * np.cos(angle) - offset[1] * np.sin(angle)
        y = offset[0] * np.sin(angle) + offset[1] * np.cos(angle)
        rotated.append(np.array([x, y]))
    return rotated

def get_formation_offsets(num_robots, formation_type):
    """Генерирует формацию: 'line', 'rhombus', 'circle'"""
    offsets = []
    
    if formation_type == 'line':
        offsets.append(np.array([2.0, 0.0]))
        for i in range(1, num_robots):
            if i % 2 == 1:
                side = -1.5 * ((i + 1) // 2)
            else:
                side = 1.5 * (i // 2)
            offsets.append(np.array([0.0, side]))
    
    elif formation_type == 'rhombus':
        offsets = [np.array([0.0, 0.0])]  # лидер
        d = 2.0  # полуось ромба
        vertices = [
            np.array([d, 0]),   # право
            np.array([0, d]),   # верх
            np.array([-d, 0]),  # лево
            np.array([0, -d])   # низ
        ]
        # добавляем вершины
        for v in vertices:
            if len(offsets) < num_robots:
                offsets.append(v)
        # добавляем на рёбра, если есть ещё роботы
        if len(offsets) < num_robots:
            remaining = num_robots - len(offsets)
            edges = [(vertices[i], vertices[(i+1)%4]) for i in range(4)]
            per_edge = remaining // 4
            extra = remaining % 4
            for i, (a, b) in enumerate(edges):
                n = per_edge + (1 if i < extra else 0)
                for k in range(1, n+1):
                    t = k / (n+1)
                    offsets.append(a * (1-t) + b * t)
    
    elif formation_type == 'circle':
        offsets.append(np.array([0.0, 0.0]))
        if num_robots > 1:
            radius = 2.0
            num_followers = num_robots - 1
            for i in range(num_followers):
                angle = 2 * np.pi * i / num_followers
                offsets.append(np.array([radius * np.cos(angle), radius * np.sin(angle)]))
    
    return offsets[:num_robots]

def get_formation_targets(leader_pos, leader_vel, offsets, goal):
    """Вычисляет целевые позиции с учётом сглаженного направления на цель"""
    global _smoothed_angle
    
    goal_dir = goal - leader_pos
    if np.linalg.norm(goal_dir) > 0.01:
        target_angle = np.arctan2(goal_dir[1], goal_dir[0])
        _smoothed_angle = _smoothed_angle * (1 - _angle_alpha) + target_angle * _angle_alpha
    
    rotated_offsets = rotate_formation(offsets, _smoothed_angle)
    return [leader_pos + offset for offset in rotated_offsets]