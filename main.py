import matplotlib
matplotlib.use('TkAgg')

import numpy as np
from config import GOAL, SAFE_DIST, RANDOM_OBSTACLES
from robot import Robot
from obstacle import Obstacle, generate_random_obstacles
from formation import get_formation_offsets
from simulation_realtime import RealtimeSimulation
from visualization import plot_errors

def run_scenario(scenario_name, use_obstacle, formation_name, num_robots):
    print(f"\n{scenario_name}")
    
    starts = [
        [2.0, 2.0], [3.0, 4.0], [1.0, 5.0], [4.0, 3.0], [2.0, 6.0],
        [5.0, 1.0], [1.0, 2.0], [6.0, 4.0], [3.0, 7.0], [4.0, 5.0]
    ]
    
    robots = []
    for i in range(num_robots):
        is_leader = (i == 0)
        robots.append(Robot(starts[i][0], starts[i][1], is_leader))
    
    # Создание препятствий
    obstacles = []
    if use_obstacle:
        if RANDOM_OBSTACLES:
            obstacles = generate_random_obstacles(num_robots, starts, GOAL)
            print(f"Сгенерировано {len(obstacles)} случайных препятствий")
        else:
            obstacles = [Obstacle(8.0, 8.0, 1.2)]
    
    sim = RealtimeSimulation(robots, GOAL, obstacles, formation_name, num_robots)
    error_history, min_dist_history = sim.run()
    
    plot_errors(error_history, min_dist_history, f"{scenario_name} - графики", SAFE_DIST)

if __name__ == "__main__":
    print("=" * 50)
    print("УПРАВЛЕНИЕ ГРУППОЙ МОБИЛЬНЫХ РОБОТОВ")
    print("=" * 50)
    
    while True:
        try:
            NUM_ROBOTS = int(input("\nВведите количество роботов (от 5 до 10): "))
            if 5 <= NUM_ROBOTS <= 10:
                break
            print("Ошибка: введите число от 5 до 10")
        except ValueError:
            print("Ошибка: введите целое число")
    
    print("\nВыберите формацию:")
    print("1 - Шеренга")
    print("2 - Ромб")
    print("3 - Круг")
    formation_choice = input("Введите 1, 2 или 3: ")
    
    if formation_choice == "1":
        formation_name = "line"
        formation_display = "Шеренга"
    elif formation_choice == "2":
        formation_name = "rhombus"
        formation_display = "Ромб"
    else:
        formation_name = "circle"
        formation_display = "Круг"
    
    print("\nВыберите сценарий:")
    print("1 - Без препятствий")
    print("2 - С препятствием")
    scenario_choice = input("Введите 1 или 2: ")
    
    if scenario_choice == "1":
        run_scenario(f"{NUM_ROBOTS} роботов, формация: {formation_display}, без препятствий", 
                     False, formation_name, NUM_ROBOTS)
    else:
        run_scenario(f"{NUM_ROBOTS} роботов, формация: {formation_display}, с препятствием", 
                     True, formation_name, NUM_ROBOTS)
    
    print("\n" + "=" * 50)
    print("Программа завершена.")
    print("=" * 50)