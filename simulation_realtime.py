import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from matplotlib.animation import FuncAnimation
import matplotlib.image as mpimg
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import imageio

from config import GOAL, DT, K_FORM, K_AVOID, K_OBS, DANGER_DIST, ROBOT_COLORS
from controller import to_goal, to_formation, avoid_neighbors, avoid_obstacle, resolve_collision
from formation import get_formation_targets, get_formation_offsets


class RealtimeSimulation:
    def __init__(self, robots, goal, obstacles, initial_formation, num_robots):
        self.robots = robots
        self.goal = goal

        self.obstacles = (
            [] if obstacles is None
            else (obstacles if isinstance(obstacles, list) else [obstacles])
        )

        self.num_robots = num_robots
        self.current_formation = initial_formation

        # физика фиксирована
        self.dt = DT

        # UI скорость
        self.speed = 1.0

        # формация
        self.SOFT_START_STEPS = 200
        self.K_FORM_FULL = K_FORM
        self.K_FORM_SOFT = K_FORM * 0.5

        self.step = 0
        self.max_steps = 3000

        # метрики
        self.error_history = []
        self.min_dist_history = []

        # запись GIF
        self.frames = []
        self.MAX_FRAMES = 300
        self.FRAME_SKIP = 10

        # графика
        self._setup_graphics()
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        self.animation = None

    # ---------------- GRAPHICS ----------------
    def _setup_graphics(self):
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.canvas.manager.window.resizable(False, False)
        self.ax.set_xlim(-2, 22)
        self.ax.set_ylim(-2, 22)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title(f"Формация: {self.current_formation} | 1/2/3 | +/- скорость")

        # Флаг
        img = mpimg.imread("Finish.jpeg")
        imagebox = OffsetImage(img, zoom=0.036)
        ab = AnnotationBbox(imagebox, (self.goal[0], self.goal[1]), frameon=False, zorder=0)
        self.ax.add_artist(ab)
        self.ax.text(self.goal[0] + 0.6, self.goal[1] + 0.3, "Финиш", fontsize=10, color="red", zorder=1)

        # Препятствия
        for obs in self.obstacles:
            self.ax.add_patch(Circle(obs.pos, obs.radius, color='gray', alpha=0.5, zorder=2))

        # Траектории
        self.trajectories = [[] for _ in range(self.num_robots)]
        self.traj_lines = [
            self.ax.plot([], [], color=ROBOT_COLORS[i], linewidth=1, zorder=3)[0]
            for i in range(self.num_robots)
        ]

        # Роботы
        self.robot_shapes = []
        self.robot_width = 0.6
        self.robot_height = 0.4

        for i in range(self.num_robots):
            if i == 0:
                # лидер — звезда
                star, = self.ax.plot([], [], '*', color=ROBOT_COLORS[i], markersize=12, zorder=5)
                self.robot_shapes.append(star)
            else:
                # ведомые — прямоугольники
                rect = Rectangle(
                    (0, 0), self.robot_width, self.robot_height,
                    angle=0, color=ROBOT_COLORS[i], alpha=0.8, zorder=5
                )
                self.ax.add_patch(rect)
                self.robot_shapes.append(rect)

    # ---------------- CONTROL ----------------
    def _update_interval(self):
        if self.animation:
            self.animation.event_source.interval = max(5, int(20 / self.speed))

    def on_key(self, event):
        if event.key == '1':
            self.current_formation = 'line'
        elif event.key == '2':
            self.current_formation = 'rhombus'
        elif event.key == '3':
            self.current_formation = 'circle'

        elif event.key in ['+', 'plus', 'equal', 'kp_add']:
            self.speed = min(self.speed + 0.2, 3.0)
            self._update_interval()
            print(f"Скорость: {self.speed:.1f}x")

        elif event.key in ['-', 'minus', 'kp_subtract']:
            self.speed = max(self.speed - 0.2, 0.2)
            self._update_interval()
            print(f"Скорость: {self.speed:.1f}x")

    # ---------------- SIMULATION ----------------
    def _get_k_form(self):
        return self.K_FORM_SOFT if self.step < self.SOFT_START_STEPS else self.K_FORM_FULL

    def _compute_forces(self, targets, k_form):
        forces = []
        for i, robot in enumerate(self.robots):
            force = np.zeros(2)

            if robot.is_leader:
                force += to_goal(robot, self.goal)
            else:
                force += to_formation(robot, targets[i], k_form)

            force += avoid_neighbors(robot, self.robots, K_AVOID)

            for obs in self.obstacles:
                force += avoid_obstacle(robot, obs, K_OBS, DANGER_DIST)

            forces.append(force)

        return forces

    def _apply_forces(self, forces):
        for robot, force in zip(self.robots, forces):
            robot.update(force, self.dt * self.speed)

    def _resolve_collisions(self):
        for robot in self.robots:
            for obs in self.obstacles:
                resolve_collision(robot, obs)

    def _update_display(self):
        # Обновление роботов
        for i, robot in enumerate(self.robots):
            if i == 0:
                # лидер — звезда
                self.robot_shapes[i].set_data([robot.pos[0]], [robot.pos[1]])
            else:
                # прямоугольники с ориентацией
                if len(robot.trajectory) > 1:
                    vx = robot.pos[0] - robot.trajectory[-2][0]
                    vy = robot.pos[1] - robot.trajectory[-2][1]
                    if abs(vx) > 0.001 or abs(vy) > 0.001:
                        angle = np.degrees(np.arctan2(vy, vx))
                    else:
                        angle = 0.0
                else:
                    angle = 0.0

                w = self.robot_width
                h = self.robot_height
                cx = robot.pos[0]
                cy = robot.pos[1]
                angle_rad = np.radians(angle)

                x = cx - (w/2) * np.cos(angle_rad) + (h/2) * np.sin(angle_rad)
                y = cy - (w/2) * np.sin(angle_rad) - (h/2) * np.cos(angle_rad)

                self.robot_shapes[i].set_xy((x, y))
                self.robot_shapes[i].angle = angle

        # Траектории
        for i, robot in enumerate(self.robots):
            self.trajectories[i].append(robot.pos.copy())
            traj = np.array(self.trajectories[i])
            self.traj_lines[i].set_data(traj[:, 0], traj[:, 1])

    def _update_metrics(self, targets):
        errors = [
            np.linalg.norm(robot.pos - targets[i])
            for i, robot in enumerate(self.robots)
            if not robot.is_leader and i < len(targets)
        ]
        self.error_history.append(np.mean(errors) if errors else 0.0)

        dists = []
        for i in range(len(self.robots)):
            for j in range(i + 1, len(self.robots)):
                dists.append(np.linalg.norm(self.robots[i].pos - self.robots[j].pos))

        self.min_dist_history.append(min(dists) if dists else 0.0)

    def _check_goal(self, leader):
        if np.linalg.norm(leader.pos - self.goal) < 0.5:
            print(f"Цель достигнута на шаге {self.step}")
            if self.animation:
                self.animation.event_source.stop()
            return True
        return False

    # ---------------- FRAME CAPTURE ----------------
    def _capture_frame(self):
        if self.step % self.FRAME_SKIP != 0:
            return
        if len(self.frames) >= self.MAX_FRAMES:
            return

        self.fig.canvas.draw()
        buf = np.frombuffer(self.fig.canvas.buffer_rgba(), dtype=np.uint8).copy()
        frame = buf.reshape(self.fig.canvas.get_width_height()[::-1] + (4,))
        self.frames.append(frame[:, :, :3])

    # ---------------- UPDATE ----------------
    def update(self, frame):
        if self.step >= self.max_steps:
            if self.animation:
                self.animation.event_source.stop()
            return self.traj_lines

        k_form = self._get_k_form()
        offsets = get_formation_offsets(self.num_robots, self.current_formation)

        leader = self.robots[0]
        leader_vel = leader.pos - leader.trajectory[-2] if len(leader.trajectory) > 1 else np.zeros(2)

        targets = get_formation_targets(leader.pos, leader_vel, offsets, self.goal)

        forces = self._compute_forces(targets, k_form)
        self._apply_forces(forces)
        self._resolve_collisions()
        self._update_display()
        self._update_metrics(targets)

        self._capture_frame()

        self.step += 1
        self._check_goal(leader)

        return self.traj_lines

    # ---------------- RUN ----------------
    def run(self):
        self.animation = FuncAnimation(
            self.fig,
            self.update,
            interval=20,
            blit=False
        )

        plt.show()

        if self.frames:
            save = input("\nСохранить GIF? (y/n): ")
            if save.lower() == 'y':
                name = input("Имя файла: ") or "animation"
                imageio.mimsave(f"{name}.gif", self.frames, fps=30)
                print(f"Сохранено: {name}.gif")

        return self.error_history, self.min_dist_history