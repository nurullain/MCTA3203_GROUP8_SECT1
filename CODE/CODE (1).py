import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math
import random
import tkinter as tk
from tkinter import ttk
import threading

# ==========================================
# 1. CONFIGURATION
# ==========================================
# Genetic Algorithm Settings
POPULATION_SIZE = 25  # Larger population for better diversity
GENERATIONS = 20  # More generations to allow "learning"
MUTATION_RATE = 0.1  # 10% chance to mutate a gene
ELITISM_COUNT = 2  # Keep the top 2 best robots unchanged (Do not lose best solution)

# Standard Manual Parameters (for "Standard Mode")
MANUAL_GENOME = {
    'safe_dist': 15.0,
    'goal_gain': 0.8,
    'avoid_gain': 3.0
}


# ==========================================
# 2. ENVIRONMENT
# ==========================================
class Environment:
    def __init__(self, map_type='complex'):
        self.width = 100
        self.height = 100
        self.start = (10, 10)
        self.goal = (90, 90)
        self.map_type = map_type

        # Combined Map Logic
        if map_type == 'simple':
            self.obstacles = [(50, 50, 15), (20, 70, 5)]
        else:
            self.obstacles = [
                (30, 20, 9), (70, 20, 9),
                (50, 50, 10),
                (20, 80, 9), (80, 80, 9),
                (50, 85, 4), (10, 45, 6),
                (90, 40, 4)
            ]


# ==========================================
# 3. ROBOT MODEL
# ==========================================
class Robot:
    def __init__(self, start_pos):
        self.x, self.y = start_pos
        self.theta = 0.0
        self.radius = 2.0
        self.path_x = [self.x]
        self.path_y = [self.y]
        self.crashed = False

        # Stuck Detection Memory
        self.history_x = []
        self.history_y = []

    def get_inputs(self, obstacles):
        """Sensors: Returns Distance & Angle to nearest obstacle"""
        min_dist = float('inf')
        nearest_angle = 0.0
        for (ox, oy, r) in obstacles:
            dist = np.sqrt((self.x - ox) ** 2 + (self.y - oy) ** 2) - r
            if dist < min_dist:
                min_dist = dist
                nearest_angle = math.atan2(oy - self.y, ox - self.x)
        return min_dist, nearest_angle

    def move(self, v, w, dt=0.5):
        """Discrete Steps Movement"""
        self.theta += w * dt
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt

        self.path_x.append(self.x)
        self.path_y.append(self.y)

        # Update Stuck Detection Memory
        self.history_x.append(self.x)
        self.history_y.append(self.y)
        if len(self.history_x) > 20:
            self.history_x.pop(0)
            self.history_y.pop(0)

    def is_stuck(self):
        """Check if robot hasn't moved 1 unit in last 20 steps"""
        if len(self.history_x) < 20: return False
        dist = np.sqrt((self.history_x[-1] - self.history_x[0]) ** 2 + (self.history_y[-1] - self.history_y[0]) ** 2)
        return dist < 1.0


# ==========================================
# 4. UNIFIED INTELLIGENT CONTROLLER
# ==========================================
class IntelligentController:
    def __init__(self, genome=None, mode='hybrid'):
        if genome is None:
            if mode == 'standard':
                self.genome = MANUAL_GENOME
            else:
                # Random Init for GA (The "DNA")
                self.genome = {
                    'safe_dist': random.uniform(10.0, 50.0),
                    'goal_gain': random.uniform(0.5, 4.0),
                    'avoid_gain': random.uniform(1.0, 10.0)
                }
        else:
            self.genome = genome

        self.recovery_timer = 0
        self.mode = mode

    def compute_outputs(self, robot, goal, obstacles):
        # 1. INPUTS
        target_angle = math.atan2(goal[1] - robot.y, goal[0] - robot.x)
        heading_error = (target_angle - robot.theta + np.pi) % (2 * np.pi) - np.pi
        obs_dist, obs_angle = robot.get_inputs(obstacles)

        # 2. RECOVERY LOGIC
        if self.recovery_timer > 0:
            self.recovery_timer -= 1
            return -1.5, 1.0  # Reverse and turn

        if robot.is_stuck():
            self.recovery_timer = 12
            return -1.5, 1.0

        # 3. FUZZY/HEURISTIC LOGIC (Driven by Genome)
        turn_cmd = self.genome['goal_gain'] * heading_error

        if obs_dist < self.genome['safe_dist']:
            obs_relative = (obs_angle - robot.theta + np.pi) % (2 * np.pi) - np.pi
            repulsion = self.genome['avoid_gain'] / (obs_dist + 0.1)

            # Emergency Avoidance
            if obs_dist < 8.0:
                turn_cmd = 0.0
                repulsion *= 2.5

            if obs_relative > 0:
                turn_cmd -= repulsion
            else:
                turn_cmd += repulsion

        # 4. OUTPUTS
        w = np.clip(turn_cmd, -2.5, 2.5)
        v = 2.0
        if abs(w) > 1.0 or obs_dist < 10.0: v = 1.0
        if obs_dist < 2.0: v = 0.0

        return v, w


# ==========================================
# 5. SIMULATION LOGIC
# ==========================================
def run_simulation_headless(genome, map_type):
    """
    Fast simulation for GA training.
    RETURNS: Fitness Score
    """
    env = Environment(map_type)
    robot = Robot(env.start)
    controller = IntelligentController(genome, mode='hybrid')
    max_steps = 1000

    fitness = 0
    start_dist = np.sqrt((env.start[0] - env.goal[0]) ** 2 + (env.start[1] - env.goal[1]) ** 2)

    for step in range(max_steps):
        v, w = controller.compute_outputs(robot, env.goal, env.obstacles)
        robot.move(v, w)

        # Distance to goal
        dist_to_goal = np.sqrt((robot.x - env.goal[0]) ** 2 + (robot.y - env.goal[1]) ** 2)

        # Check Collision
        for (ox, oy, r) in env.obstacles:
            if np.sqrt((robot.x - ox) ** 2 + (robot.y - oy) ** 2) < (r + robot.radius):
                robot.crashed = True
                # Penalize crash, but reward distance traveled towards goal
                progress = start_dist - dist_to_goal
                return progress - 200  # Heavy penalty for crashing

        # Check Goal
        if dist_to_goal < 5.0:
            # Massive reward for goal + Efficiency bonus (fewer steps is better)
            return 2000 + (max_steps - step)

    # If time runs out
    progress = start_dist - dist_to_goal
    return progress  # Fitness is just how close they got


def evolve_population(map_type, progress_callback):
    """
    TRUE GENETIC ALGORITHM implementation.
    Includes: Tournament Selection, Crossover, Mutation, Elitism.
    """
    # 1. Initialize Random Population
    population = [IntelligentController(mode='hybrid').genome for _ in range(POPULATION_SIZE)]

    for gen in range(GENERATIONS):
        # 2. Evaluate Fitness for ALL robots
        scores = []
        for genome in population:
            fit = run_simulation_headless(genome, map_type)
            scores.append((genome, fit))

        # Sort by fitness (Best first)
        scores.sort(key=lambda x: x[1], reverse=True)

        # Logging for "Learning Proof"
        best_fitness = scores[0][1]
        avg_fitness = sum(s[1] for s in scores) / len(scores)
        print(f"Gen {gen + 1}: Best={int(best_fitness)} | Avg={int(avg_fitness)}")  # Console proof
        progress_callback(f"Training Gen {gen + 1}/{GENERATIONS} | Best Fit: {int(best_fitness)}")

        # 3. Selection & Breeding (The "Learning" Part)
        next_population = []

        # ELITISM: Keep the absolute best robots to ensure we don't "un-learn"
        for i in range(ELITISM_COUNT):
            next_population.append(scores[i][0])

        # Generate the rest of the new population
        while len(next_population) < POPULATION_SIZE:
            # TOURNAMENT SELECTION: Pick 3 random, choose best (Simulates competition)
            candidates = random.sample(scores, 3)
            candidates.sort(key=lambda x: x[1], reverse=True)
            parent_a = candidates[0][0]

            candidates = random.sample(scores, 3)
            candidates.sort(key=lambda x: x[1], reverse=True)
            parent_b = candidates[0][0]

            # CROSSOVER: Mix Genes
            child = {}
            for key in parent_a:
                # 50% chance to get gene from Mom or Dad
                if random.random() > 0.5:
                    child[key] = parent_a[key]
                else:
                    child[key] = parent_b[key]

            # MUTATION: Small chance to randomly change a gene
            # This introduces "New Ideas" to the robot
            if random.random() < MUTATION_RATE:
                mutate_key = random.choice(list(child.keys()))
                # Add random noise (+/- 20%)
                child[mutate_key] *= random.uniform(0.8, 1.2)

            next_population.append(child)

        population = next_population

    # Return the best genome from the final generation
    return scores[0][0]


# ==========================================
# 6. GUI APPLICATION CLASS
# ==========================================
class RobotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Intelligent Robot Navigation System")
        self.root.geometry("900x700")

        # --- CONTROL PANEL ---
        control_frame = tk.Frame(root, pady=10)
        control_frame.pack(side=tk.TOP, fill=tk.X)

        # Map Selection
        tk.Label(control_frame, text="Select Map:", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=10)
        self.map_var = tk.StringVar(value="complex")
        tk.Radiobutton(control_frame, text="Simple Map", variable=self.map_var, value="simple").pack(side=tk.LEFT)
        tk.Radiobutton(control_frame, text="Complex Map", variable=self.map_var, value="complex").pack(side=tk.LEFT)

        # Mode Selection
        tk.Label(control_frame, text="|   Control Mode:", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=10)
        self.mode_var = tk.StringVar(value="hybrid")
        tk.Radiobutton(control_frame, text="Standard Fuzzy (Manual)", variable=self.mode_var, value="standard").pack(
            side=tk.LEFT)
        tk.Radiobutton(control_frame, text="Hybrid Evolutionary (AI)", variable=self.mode_var, value="hybrid").pack(
            side=tk.LEFT)

        # Run Button
        self.run_btn = tk.Button(control_frame, text="RUN SIMULATION", bg="green", fg="white",
                                 font=("Arial", 10, "bold"), command=self.start_thread)
        self.run_btn.pack(side=tk.LEFT, padx=20)

        # Status Label
        self.status_lbl = tk.Label(control_frame, text="Ready", fg="blue", width=40)
        self.status_lbl.pack(side=tk.LEFT, padx=10)

        # --- MATPLOTLIB FIGURE ---
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.running = False

    def update_status(self, msg):
        self.status_lbl.config(text=msg)
        self.root.update_idletasks()

    def start_thread(self):
        if self.running: return
        self.running = True
        self.run_btn.config(state=tk.DISABLED)
        threading.Thread(target=self.run_process, daemon=True).start()

    def run_process(self):
        selected_map = self.map_var.get()
        selected_mode = self.mode_var.get()

        # 1. Determine Genome (Fixed or Trained)
        final_genome = None

        if selected_mode == 'standard':
            self.update_status("Loading Standard Fuzzy Parameters...")
            final_genome = MANUAL_GENOME
            # Tiny pause to simulate loading
            self.root.after(500)
        else:
            self.update_status("Starting Genetic Algorithm Training...")
            print("\n--- STARTING EVOLUTION ---")
            # Run Training
            final_genome = evolve_population(selected_map, self.update_status)
            print("--- EVOLUTION COMPLETE ---\n")

        # 2. Run Visual Simulation
        self.update_status(f"Running Simulation ({selected_mode.upper()} Mode)...")
        self.visualize_simulation(final_genome, selected_map, selected_mode)

        self.running = False
        self.run_btn.config(state=tk.NORMAL)
        self.update_status("Simulation Complete.")

    def visualize_simulation(self, genome, map_type, mode):
        env = Environment(map_type)
        robot = Robot(env.start)
        controller = IntelligentController(genome, mode)

        max_steps = 1500
        goal_reached = False
        total_dist = 0.0
        smoothness = 0.0

        for step in range(max_steps):
            if not self.running: break  # Stop if window closed

            # Physics
            v, w = controller.compute_outputs(robot, env.goal, env.obstacles)
            robot.move(v, w)

            # Metrics
            if step > 0:
                dist = np.sqrt((robot.path_x[-1] - robot.path_x[-2]) ** 2 + (robot.path_y[-1] - robot.path_y[-2]) ** 2)
                total_dist += dist
                smoothness += abs(w)

            # Checks
            for (ox, oy, r) in env.obstacles:
                if np.sqrt((robot.x - ox) ** 2 + (robot.y - oy) ** 2) < (r + robot.radius):
                    robot.crashed = True
                    break

            dist_to_goal = np.sqrt((robot.x - env.goal[0]) ** 2 + (robot.y - env.goal[1]) ** 2)
            if dist_to_goal < 5.0:
                goal_reached = True

            # RENDER EVERY 5 FRAMES
            if step % 5 == 0 or goal_reached or robot.crashed:
                self.ax.clear()
                self.ax.set_title(f"Simulation: {mode.upper()} | {map_type.upper()} | Steps: {step}")
                self.ax.set_xlim(0, 100);
                self.ax.set_ylim(0, 100)

                # Draw Obstacles
                for (ox, oy, r) in env.obstacles:
                    self.ax.add_patch(patches.Circle((ox, oy), r, color='gray'))

                # Draw Robot & Goal
                self.ax.plot(env.goal[0], env.goal[1], 'rx', markersize=12, label="Goal")
                self.ax.plot(robot.path_x, robot.path_y, 'b--')
                self.ax.add_patch(patches.Circle((robot.x, robot.y), robot.radius, color='blue'))

                # Overlay Texts
                status_txt = "REVERSING" if controller.recovery_timer > 0 else "AUTONOMOUS"
                self.ax.text(5, 95, f"Action: {status_txt}", color='red' if "REV" in status_txt else 'green')

                if goal_reached:
                    self.ax.text(30, 50, "GOAL REACHED!", color='green', fontsize=20, weight='bold')
                if robot.crashed:
                    self.ax.text(35, 50, "CRASHED!", color='red', fontsize=20, weight='bold')

                self.canvas.draw()

            if goal_reached or robot.crashed:
                break

        # Print Final Report to Console
        print("\n" + "=" * 40)
        print(f"REPORT: {mode.upper()} on {map_type.upper()}")
        print(f"Outcome: {'SUCCESS' if goal_reached else 'FAILURE'}")
        print(f"Steps: {step}, Length: {total_dist:.2f}, Smoothness: {smoothness:.2f}")
        print("=" * 40)


# ==========================================
# MAIN EXECUTION
# ==========================================
if __name__ == "__main__":
    root = tk.Tk()
    app = RobotApp(root)
    root.mainloop()