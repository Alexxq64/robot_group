import numpy as np
from config import GOAL, DT, SAFE_DIST, K_GOAL, K_FORM, K_AVOID, K_OBS, DANGER_DIST
from robot import Robot
from obstacle import Obstacle
from formation import get_formation_offsets
from simulation import run_simulation
from visualization import visualize_trajectories, plot_errors, animate_simulation

def run_scenario(scenario_name, use_obstacle, formation_offsets, num_robots):
    print(f"\nЗапуск сценария: {scenario_name}")
    
    # Стартовые позиции
    starts = [
        [2.0, 2.0], [3.0, 4.0], [1.0, 5.0], [4.0, 3.0], [2.0, 6.0],
        [5.0, 1.0], [1.0, 2.0], [6.0, 4.0], [3.0, 7.0], [4.0, 5.0]
    ]
    
    robots = []
    for i in range(num_robots):
        is_leader = (i == 0)
        robots.append(Robot(starts[i][0], starts[i][1], is_leader))
    
    obstacle = None
    if use_obstacle:
        obstacle = Obstacle(8.0, 8.0, 1.2)
    
    error_history, min_dist_history = run_simulation(robots, GOAL, obstacle, formation_offsets)
    
    print(f"Роботы достигли цели. Шагов: {len(error_history)}")
    
    if use_obstacle:
        visualize_trajectories(robots, GOAL, obstacle, f"{scenario_name} - траектории")
    else:
        visualize_trajectories(robots, GOAL, None, f"{scenario_name} - траектории")
    
    plot_errors(error_history, min_dist_history, f"{scenario_name} - графики", SAFE_DIST)
    animate_simulation(robots, GOAL, obstacle, f"{scenario_name} - анимация", formation_offsets, DT, K_GOAL, K_FORM, K_AVOID, K_OBS, DANGER_DIST)

if __name__ == "__main__":
    print("=" * 50)
    print("УПРАВЛЕНИЕ ГРУППОЙ МОБИЛЬНЫХ РОБОТОВ")
    print("=" * 50)
    
    while True:
        try:
            NUM_ROBOTS = int(input("\nВведите количество роботов (от 5 до 10): "))
            if 5 <= NUM_ROBOTS <= 10:
                break
            else:
                print("Ошибка: введите число от 5 до 10")
        except ValueError:
            print("Ошибка: введите целое число")
    
    print("\nВыберите формацию:")
    print("1 - Шеренга (лидер впереди, остальные в линию)")
    print("2 - Ромб (лидер в центре)")
    print("3 - Круг (лидер в центре, ведомые на окружности)")
    formation_choice = input("Введите 1, 2 или 3: ")
    
    if formation_choice == "1":
        formation_offsets = get_formation_offsets(NUM_ROBOTS, 'line')
        formation_name = "Шеренга"
    elif formation_choice == "2":
        formation_offsets = get_formation_offsets(NUM_ROBOTS, 'rhombus')
        formation_name = "Ромб"
    else:
        formation_offsets = get_formation_offsets(NUM_ROBOTS, 'circle')
        formation_name = "Круг"
    
    print("\nВыберите сценарий:")
    print("1 - Без препятствий")
    print("2 - С препятствием (круг, центр 8,8)")
    scenario_choice = input("Введите 1 или 2: ")
    
    if scenario_choice == "1":
        run_scenario(f"{NUM_ROBOTS} роботов, формация: {formation_name}, без препятствий", 
                     False, formation_offsets, NUM_ROBOTS)
    else:
        run_scenario(f"{NUM_ROBOTS} роботов, формация: {formation_name}, с препятствием", 
                     True, formation_offsets, NUM_ROBOTS)
    
    print("\n" + "=" * 50)
    print("Программа завершена.")
    print("=" * 50)