import numpy as np

# Параметры симуляции
DT = 0.05
STEPS = 3000  # максимальное количество шагов
FIELD_SIZE = 20
GOAL = np.array([18.0, 18.0])

# Веса
K_GOAL = 1.0
K_FORM = 1.2
K_AVOID = 2.0
K_OBS = 1.5

# Пороги
SAFE_DIST = 1.0
DANGER_DIST = 1.5
DEAD_ZONE = 0.15

# Параметры случайных препятствий
RANDOM_OBSTACLES = True
NUM_OBSTACLES_RANDOM = (1, 5)      # от 1 до 5
OBSTACLE_RADIUS_RANDOM = (0.5, 1.5) # от 0.5 до 1.2

# Глобальные цвета для роботов (10)
ROBOT_COLORS = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'cyan', 'magenta', 'lime']