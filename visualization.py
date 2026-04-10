import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
from config import FIELD_SIZE, ROBOT_COLORS, GOAL
from robot import Robot

def visualize_trajectories(robots, goal, obstacle, title):
    plt.figure(figsize=(10, 8))
    for i, robot in enumerate(robots):
        traj = np.array(robot.trajectory)
        plt.plot(traj[:, 0], traj[:, 1], color=ROBOT_COLORS[i], linewidth=1.5, label=f'Robot {i+1}')
        plt.scatter(traj[0, 0], traj[0, 1], color=ROBOT_COLORS[i], marker='o', s=80, zorder=5)
        plt.scatter(traj[-1, 0], traj[-1, 1], color=ROBOT_COLORS[i], marker='s', s=80, zorder=5)
    
    plt.scatter(goal[0], goal[1], color='red', marker='*', s=200, label='Goal', zorder=5)
    
    if obstacle:
        circle = Circle(obstacle.pos, obstacle.radius, color='gray', alpha=0.5, label='Obstacle')
        plt.gca().add_patch(circle)
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(title)
    plt.legend()
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.xlim(-2, FIELD_SIZE+2)
    plt.ylim(-2, FIELD_SIZE+2)
    plt.show()

def plot_errors(error_history, min_dist_history, title, safe_dist):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    ax1.plot(error_history)
    ax1.set_xlabel('Time step')
    ax1.set_ylabel('Formation error')
    ax1.set_title('Formation error over time')
    ax1.grid(True, alpha=0.3)
    
    ax2.plot(min_dist_history)
    ax2.set_xlabel('Time step')
    ax2.set_ylabel('Min distance between robots')
    ax2.set_title('Minimum inter-robot distance')
    ax2.axhline(y=safe_dist, color='r', linestyle='--', label='Safe threshold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.suptitle(title)
    plt.tight_layout()
    plt.show()

def animate_simulation(robots, goal, obstacle, title, formation_offsets, dt, k_goal, k_form, k_avoid, k_obs, danger_dist):
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(-2, FIELD_SIZE+2)
    ax.set_ylim(-2, FIELD_SIZE+2)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title(title)
    
    points = []
    for i in range(len(robots)):
        point, = ax.plot([], [], 'o', color=ROBOT_COLORS[i], markersize=10, label=f'Robot {i+1}')
        points.append(point)
    
    ax.scatter(goal[0], goal[1], color='red', marker='*', s=200, label='Goal', zorder=5)
    
    if obstacle:
        circle = Circle(obstacle.pos, obstacle.radius, color='gray', alpha=0.5, label='Obstacle')
        ax.add_patch(circle)
    
    ax.legend()
    
    trajectories = [[] for _ in range(len(robots))]
    traj_lines = []
    for i in range(len(robots)):
        line, = ax.plot([], [], color=ROBOT_COLORS[i], linewidth=1, alpha=0.5)
        traj_lines.append(line)
    
    # Копии роботов для анимации
    anim_robots = []
    for r in robots:
        new_r = Robot(r.trajectory[0][0], r.trajectory[0][1], r.is_leader)
        anim_robots.append(new_r)
    
    max_steps = len(robots[0].trajectory)
    step_counter = 0
    
    def animate(frame):
        nonlocal step_counter
        if step_counter >= max_steps:
            return points + traj_lines
        
        for i, robot in enumerate(anim_robots):
            if step_counter < len(robots[i].trajectory):
                robot.pos = robots[i].trajectory[step_counter].copy()
        
        for i, robot in enumerate(anim_robots):
            points[i].set_data([robot.pos[0]], [robot.pos[1]])
            trajectories[i].append(robot.pos.copy())
            if len(trajectories[i]) > 1:
                traj = np.array(trajectories[i])
                traj_lines[i].set_data(traj[:, 0], traj[:, 1])
        
        step_counter += 5
        return points + traj_lines
    
    ani = animation.FuncAnimation(fig, animate, frames=min(400, max_steps//2), interval=50, blit=True)
    plt.show()