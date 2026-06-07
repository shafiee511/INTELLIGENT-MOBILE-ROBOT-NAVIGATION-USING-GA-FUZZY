import tkinter as tk
from tkinter import ttk
import math
import random
import json
from datetime import datetime
import argparse
import sys

# ==========================================
# ENVIRONMENT SETUP
# ==========================================
TARGET_POSITION = (200, 40)
INITIAL_STATE = (360, 460, 3.14)

# DISCRETE MOVEMENT CONFIGURATION
DISCRETE_MODE = True          # Enable discrete step-based movement
STEP_SIZE = 25                # Pixels per discrete step (larger = faster training)
STEP_ANIMATION_FRAMES = 1     # Frames to animate each step (1 = instant snap)
PAUSE_BETWEEN_STEPS = 0       # No pause during training for speed

# SIMPLE MAP - Fewer obstacles, wider passages, easier navigation
SIMPLE_MAP = [
    # Boundary walls
    (0, 0, 400, 10), (0, 490, 400, 500), (0, 0, 10, 500), (390, 0, 400, 500),
    # Wide corridors - horizontal barriers
    (100, 350, 400, 360),  # Bottom horizontal wall
    (0, 220, 320, 230),    # Middle horizontal wall
    # Simple vertical obstacles
    (200, 100, 210, 220),  # Central vertical pillar
    (100, 260, 110, 350),  # Left side pillar
]

# COMPLEX MAP - More obstacles, narrow passages, challenging navigation
COMPLEX_MAP = [
    # Boundary walls
    (0, 0, 400, 10), (0, 490, 400, 500), (0, 0, 10, 500), (390, 0, 400, 500),
    # Internal barriers creating maze structure
    (80, 400, 400, 410),   # Bottom corridor wall
    (0, 300, 320, 310),    # Mid-low horizontal wall
    (80, 200, 400, 210),   # Mid horizontal wall
    (0, 120, 150, 130),    # Top-left wall
    (250, 120, 400, 130),  # Top-right wall
    # Complex obstacle clusters
    (180, 410, 190, 440),  # Bottom vertical pillar
    (240, 250, 250, 300),  # Right-center vertical
    (100, 150, 120, 180),  # Left-top vertical
    (200, 210, 210, 240),  # Center vertical
    (80, 210, 90, 250),    # Left-center vertical
    (150, 310, 160, 350),  # Left-bottom vertical
    # Additional complex obstacles
    (300, 250, 310, 290),  # Right-side pillar
    (150, 50, 160, 110),   # Top-center pillar
    (320, 150, 330, 200),  # Right-top pillar
    # Strategic additions for moderate challenge (3 obstacles for 22 total)
    (280, 350, 290, 390),  # Block right path from start
    (200, 330, 210, 370),  # Center obstacle near bottom
    (180, 140, 190, 170),  # Block center-top passage
]

# SELECT WHICH MAP TO USE (change this to switch maps)
# Options: SIMPLE_MAP or COMPLEX_MAP
# This will be set via command-line argument or GUI selection
CURRENT_MAP = None  # Will be set dynamically
OBSTACLE_MAP = None  # Will be set after map selection

# GA PARAMETERS
POPULATION_COUNT = 20
EVOLUTION_CYCLES = 100
MUTATION_PROBABILITY = 0.15
STEP_LIMIT = 600


# ==========================================
# ADAPTIVE FUZZY CONTROLLER
# ==========================================
class AdaptiveFuzzyController:
    def __init__(self, chromosome):
        """Initialize with GA-evolved parameters"""
        near_threshold, med_start, med_end, far_start = chromosome

        # Ensure logical consistency
        if med_start >= med_end:
            med_start = med_end - 1

        self.dist_near = [0, 0, near_threshold]
        self.dist_medium = [med_start, 40, med_end]
        self.dist_far = [far_start, 100, 1000]

        self.angle_cw = [-3.14, -1.0, -0.1]
        self.angle_straight = [-0.3, 0.0, 0.3]
        self.angle_ccw = [0.1, 1.0, 3.14]

        self.steer_map = {
            "Sharp_CW": -0.8, "Gentle_CW": -0.3, "Neutral": 0.0,
            "Gentle_CCW": 0.3, "Sharp_CCW": 0.8
        }
        self.speed_map = {
            "Stop": 0.0, "Slow": 2.0, "Medium": 4.0, "Fast": 7.0
        }

    def membership_func(self, val, params):
        """Triangular membership function"""
        a, b, c = params
        if val <= a or val >= c: return 0.0
        if a < val <= b: return (val - a) / (b - a)
        return (c - val) / (c - b)

    def infer_action(self, sensors, goal_bearing):
        """Fuzzy inference engine"""

        def fuzzify_distance(d):
            return {
                "N": self.membership_func(d, self.dist_near),
                "M": self.membership_func(d, self.dist_medium),
                "F": self.membership_func(d, self.dist_far)
            }

        sensor_fuzzy = [fuzzify_distance(s) for s in sensors]

        bearing_cw = self.membership_func(goal_bearing, self.angle_cw)
        bearing_straight = self.membership_func(goal_bearing, self.angle_straight)
        bearing_ccw = self.membership_func(goal_bearing, self.angle_ccw)

        steer_activation = {k: 0.0 for k in self.steer_map}
        speed_activation = {k: 0.0 for k in self.speed_map}

        def trigger_rule(strength, steer, speed):
            steer_activation[steer] = max(steer_activation[steer], strength)
            speed_activation[speed] = max(speed_activation[speed], strength)

        # Rule base
        if sensors[1] < sensors[2]:
            trigger_rule(sensor_fuzzy[0]["N"], "Sharp_CW", "Slow")
        else:
            trigger_rule(sensor_fuzzy[0]["N"], "Sharp_CCW", "Slow")

        trigger_rule(min(sensor_fuzzy[3]["N"], sensor_fuzzy[4]["N"]), "Neutral", "Medium")
        trigger_rule(min(sensor_fuzzy[3]["N"], sensor_fuzzy[4]["F"]), "Gentle_CW", "Medium")
        trigger_rule(min(sensor_fuzzy[4]["N"], sensor_fuzzy[3]["F"]), "Gentle_CCW", "Medium")
        trigger_rule(sensor_fuzzy[1]["N"], "Gentle_CW", "Slow")
        trigger_rule(sensor_fuzzy[2]["N"], "Gentle_CCW", "Slow")

        trigger_rule(min(sensor_fuzzy[4]["N"], sensor_fuzzy[2]["N"], sensor_fuzzy[0]["M"]), "Gentle_CCW", "Slow")
        trigger_rule(min(sensor_fuzzy[3]["N"], sensor_fuzzy[1]["N"], sensor_fuzzy[0]["M"]), "Gentle_CW", "Slow")

        clearance = min(max(sensor_fuzzy[0]["F"], sensor_fuzzy[0]["M"]),
                        max(sensor_fuzzy[1]["F"], sensor_fuzzy[1]["M"]),
                        max(sensor_fuzzy[2]["F"], sensor_fuzzy[2]["M"]))
        velocity = "Fast" if min(sensor_fuzzy[0]["F"], sensor_fuzzy[1]["F"]) > 0.5 else "Medium"
        trigger_rule(min(clearance, bearing_ccw), "Gentle_CCW", velocity)
        trigger_rule(min(clearance, bearing_cw), "Gentle_CW", velocity)
        trigger_rule(min(clearance, bearing_straight), "Neutral", velocity)

        # Defuzzification
        steer_num = sum(w * self.steer_map[a] for a, w in steer_activation.items())
        steer_den = sum(steer_activation.values())
        final_steer = steer_num / steer_den if steer_den != 0 else 0.0

        speed_num = sum(w * self.speed_map[s] for s, w in speed_activation.items())
        speed_den = sum(speed_activation.values())
        final_speed = 2.0 if speed_den == 0 else speed_num / speed_den

        # Convert to discrete command for discrete mode
        if DISCRETE_MODE:
            turn_angle = final_steer
            # Speed determines step distance
            if final_speed > 5.0:
                step_distance = STEP_SIZE
            elif final_speed > 3.0:
                step_distance = STEP_SIZE * 0.75
            elif final_speed > 1.0:
                step_distance = STEP_SIZE * 0.5
            else:
                step_distance = STEP_SIZE * 0.25
            return turn_angle, step_distance
        else:
            return final_speed, final_steer


# ==========================================
# GA TRAINING INTERFACE
# ==========================================
class GeneticAlgorithmTrainer:
    def __init__(self, master):
        self.master = master
        self.master.title("GA-Fuzzy Hybrid Optimization System")
        self.master.geometry("900x680")
        self.master.configure(bg="#1a1a2e")

        # Header with gradient-like effect
        header = tk.Frame(master, bg="#1e3a8a", height=90)
        header.pack(fill=tk.X)
        
        title_label = tk.Label(header, text="🧬 GENETIC ALGORITHM OPTIMIZER",
                 font=("Segoe UI", 20, "bold"), bg="#1e3a8a", fg="#ffffff")
        title_label.pack(pady=12)
        
        # Display current map with modern styling
        map_info = tk.Label(header, text=f"🗺️ Map: {'SIMPLE' if CURRENT_MAP == SIMPLE_MAP else 'COMPLEX'} • {len(OBSTACLE_MAP)} obstacles",
                           font=("Segoe UI", 10), bg="#1e3a8a", fg="#93c5fd")
        map_info.pack(pady=3)

        # Main layout with modern dark theme
        main_container = tk.Frame(master, bg="#0f172a")
        main_container.pack(fill=tk.BOTH, expand=True, padx=15, pady=12)

        # Left panel - Simulation with enhanced colors
        left_panel = tk.Frame(main_container, bg="#1e293b", bd=0, relief=tk.FLAT)
        left_panel.pack(side=tk.LEFT, padx=8, fill=tk.BOTH, expand=True)

        tk.Label(left_panel, text="⚡ TRAINING ARENA", font=("Segoe UI", 13, "bold"),
                 bg="#1e293b", fg="#06b6d4").pack(pady=8)

        self.simulation_canvas = tk.Canvas(left_panel, width=500, height=500,
                                           bg="#0f172a", highlightthickness=2, highlightbackground="#334155")
        self.simulation_canvas.pack(padx=5, pady=5)

        # Right panel - Statistics with modern design
        right_panel = tk.Frame(main_container, bg="#1e293b", bd=0, relief=tk.FLAT)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, padx=8)

        tk.Label(right_panel, text="📊 EVOLUTION METRICS", font=("Segoe UI", 15, "bold"),
                 bg="#1e293b", fg="#22d3ee").pack(pady=12)

        stats_frame = tk.Frame(right_panel, bg="#1e293b")
        stats_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=5)

        self.generation_label = tk.Label(stats_frame, text="Generation: 0",
                                         font=("Segoe UI", 13, "bold"),
                                         bg="#1e293b", fg="#fbbf24", anchor=tk.W)
        self.generation_label.pack(pady=8, fill=tk.X)

        self.individual_label = tk.Label(stats_frame, text="Individual: 0/0",
                                         font=("Segoe UI", 12),
                                         bg="#1e293b", fg="#cbd5e1", anchor=tk.W)
        self.individual_label.pack(pady=5, fill=tk.X)

        self.fitness_label = tk.Label(stats_frame, text="Best Fitness: 0.00",
                                      font=("Segoe UI", 14, "bold"),
                                      bg="#1e293b", fg="#10b981", anchor=tk.W)
        self.fitness_label.pack(pady=15, fill=tk.X)

        # Chromosome display with modern styling
        tk.Label(stats_frame, text="🧬 Current Chromosome:",
                 font=("Segoe UI", 11, "bold"), bg="#1e293b", fg="#22d3ee",
                 anchor=tk.W).pack(pady=(20, 5), fill=tk.X)

        self.chromosome_display = tk.Text(stats_frame, height=6, width=28,
                                          font=("Consolas", 10), bg="#0f172a",
                                          fg="#e2e8f0", relief=tk.FLAT,
                                          highlightthickness=2, highlightbackground="#475569")
        self.chromosome_display.pack(pady=5, fill=tk.X)

        # Progress bar with modern theme
        tk.Label(stats_frame, text="⏳ Training Progress:",
                 font=("Segoe UI", 11, "bold"), bg="#1e293b", fg="#22d3ee",
                 anchor=tk.W).pack(pady=(20, 5), fill=tk.X)

        self.progress_bar = ttk.Progressbar(stats_frame, mode='determinate',
                                            length=250, style="Modern.Horizontal.TProgressbar")
        self.progress_bar.pack(pady=5, fill=tk.X)

        # Control button with modern gradient-like effect
        self.save_button = tk.Button(stats_frame, text="💾 SAVE & EXIT",
                                     command=self.finalize_training, bg="#dc2626",
                                     fg="white", font=("Segoe UI", 12, "bold"),
                                     height=2, relief=tk.FLAT, bd=0, cursor="hand2")
        self.save_button.pack(side=tk.BOTTOM, pady=30, fill=tk.X)

        # Modern style configuration
        style = ttk.Style()
        style.theme_use('clam')
        style.configure("Modern.Horizontal.TProgressbar",
                        troughcolor='#0f172a', background='#06b6d4',
                        thickness=22, borderwidth=0)

        # Initialize GA
        self.gene_pool = [self.generate_chromosome() for _ in range(POPULATION_COUNT)]
        self.fitness_records = []
        self.current_generation = 1
        self.current_individual = 0
        self.global_best_fitness = 0.0
        self.global_best_chromosome = []

        # Enhanced robot graphics with glow effect
        self.robot_polygon = self.simulation_canvas.create_polygon(0, 0, 0, 0, fill="#06b6d4", 
                                                                    outline="#22d3ee", width=2)
        # Sensor lines with modern cyan color and glow
        self.range_sensors = [self.simulation_canvas.create_line(0, 0, 0, 0, fill="#22d3ee", 
                                                                  width=2, dash=(4, 2))
                              for _ in range(5)]
        self.trajectory = []
        self.exploration_grid = set()
        self.step_counter = 0
        self.is_active = False

        # Draw environment with modern colors
        # Modern glowing goal
        self.simulation_canvas.create_oval(TARGET_POSITION[0] - 15, TARGET_POSITION[1] - 15,
                                           TARGET_POSITION[0] + 15, TARGET_POSITION[1] + 15,
                                           fill="#10b981", outline="#34d399", width=4)
        self.simulation_canvas.create_oval(TARGET_POSITION[0] - 8, TARGET_POSITION[1] - 8,
                                           TARGET_POSITION[0] + 8, TARGET_POSITION[1] + 8,
                                           fill="#6ee7b7", outline="")
        
        # Modern gradient-style obstacles
        for i, obs in enumerate(OBSTACLE_MAP):
            if i < 4:  # Boundaries
                color = "#1e293b"
                outline = "#475569"
            elif i < 9:  # Mid obstacles
                color = "#334155"
                outline = "#64748b"
            else:  # Interior obstacles
                color = "#4f46e5"
                outline = "#6366f1"
            self.simulation_canvas.create_rectangle(obs, fill=color, outline=outline, 
                                                   width=2, tags="obstacle")

        self.launch_individual()


    def generate_chromosome(self):
        """Create random chromosome"""
        return [
            random.uniform(20, 60),  # near_threshold
            random.uniform(5, 30),  # med_start
            random.uniform(30, 80),  # med_end
            random.uniform(30, 70)  # far_start
        ]

    def launch_individual(self):
        """Initialize next individual for evaluation"""
        self.active_chromosome = self.gene_pool[self.current_individual]
        self.controller = AdaptiveFuzzyController(self.active_chromosome)

        self.robot_pose = {
            "x": INITIAL_STATE[0],
            "y": INITIAL_STATE[1],
            "theta": INITIAL_STATE[2],
            # Discrete movement state
            "movement_state": "IDLE",
            "target_x": INITIAL_STATE[0],
            "target_y": INITIAL_STATE[1],
            "target_theta": INITIAL_STATE[2],
            "animation_frame": 0,
            "start_x": INITIAL_STATE[0],
            "start_y": INITIAL_STATE[1],
            "start_theta": INITIAL_STATE[2],
            "discrete_steps": 0
        }
        self.exploration_grid = set()
        self.step_counter = 0
        self.is_active = True
        self.initial_distance = math.hypot(TARGET_POSITION[0] - self.robot_pose["x"],
                                           TARGET_POSITION[1] - self.robot_pose["y"])

        # Update UI
        self.generation_label.config(text=f"Generation: {self.current_generation} / {EVOLUTION_CYCLES}")
        self.individual_label.config(text=f"Individual: {self.current_individual + 1} / {POPULATION_COUNT}")

        chromosome_text = (f"Near Max:  {self.active_chromosome[0]:.2f}\n"
                           f"Med Start: {self.active_chromosome[1]:.2f}\n"
                           f"Med End:   {self.active_chromosome[2]:.2f}\n"
                           f"Far Start: {self.active_chromosome[3]:.2f}")
        self.chromosome_display.delete(1.0, tk.END)
        self.chromosome_display.insert(1.0, chromosome_text)

        progress = ((self.current_generation - 1) * POPULATION_COUNT + self.current_individual + 1) / (
                    EVOLUTION_CYCLES * POPULATION_COUNT) * 100
        self.progress_bar['value'] = progress

        # Clear trajectory
        for line in self.trajectory:
            self.simulation_canvas.delete(line)
        self.trajectory = []

        self.master.after(10, self.simulation_loop)

    def sense_environment(self, x, y, theta):
        """5-beam range sensor simulation"""
        beam_angles = [0, 0.785, -0.785, 1.57, -1.57]
        ranges = []

        for i, offset in enumerate(beam_angles):
            ray_angle = theta + offset
            dx, dy = math.cos(ray_angle), math.sin(ray_angle)
            min_dist = 150.0

            for x1, y1, x2, y2 in OBSTACLE_MAP:
                if abs(dx) > 0.001:
                    t1, t2 = (x1 - x) / dx, (x2 - x) / dx
                    if 0 < t1 < min_dist and y1 <= y + t1 * dy <= y2: min_dist = t1
                    if 0 < t2 < min_dist and y1 <= y + t2 * dy <= y2: min_dist = t2
                if abs(dy) > 0.001:
                    t3, t4 = (y1 - y) / dy, (y2 - y) / dy
                    if 0 < t3 < min_dist and x1 <= x + t3 * dx <= x2: min_dist = t3
                    if 0 < t4 < min_dist and x1 <= x + t4 * dx <= x2: min_dist = t4

            ranges.append(min_dist)
            self.simulation_canvas.coords(self.range_sensors[i], x, y, x + min_dist * dx, y + min_dist * dy)

        return ranges

    def evaluate_fitness(self, outcome):
        """Calculate fitness score based on discrete steps"""
        final_distance = math.hypot(TARGET_POSITION[0] - self.robot_pose["x"],
                                    TARGET_POSITION[1] - self.robot_pose["y"])
        score = (self.initial_distance - final_distance) * 2.0

        if outcome == "SUCCESS":
            # Reward reaching goal, bonus for fewer discrete steps
            score += 5000.0 + (STEP_LIMIT - self.robot_pose["discrete_steps"]) * 10
        elif outcome == "CRASH":
            score -= 200.0

        # Penalize inefficient movement (too many steps for little progress)
        if self.robot_pose["discrete_steps"] > 50:
            exploration_ratio = len(self.exploration_grid) / self.robot_pose["discrete_steps"]
            if exploration_ratio < 0.15:
                score -= 1000.0

        return max(0.0, score)

    def complete_individual(self, outcome):
        """Finish current individual evaluation"""
        fitness = self.evaluate_fitness(outcome)
        self.fitness_records.append((fitness, self.active_chromosome))

        if fitness > self.global_best_fitness:
            self.global_best_fitness = fitness
            self.global_best_chromosome = self.active_chromosome
            self.fitness_label.config(text=f"Best Fitness: {fitness:.2f}")

        self.current_individual += 1

        if self.current_individual < POPULATION_COUNT:
            self.launch_individual()
        else:
            self.evolve_generation()

    def evolve_generation(self):
        """Genetic operators: selection, crossover, mutation"""
        self.fitness_records.sort(key=lambda x: x[0], reverse=True)
        elite = [x[1] for x in self.fitness_records[:4]]

        next_population = list(elite)

        while len(next_population) < POPULATION_COUNT:
            parent1, parent2 = random.sample(elite, 2)
            crossover_point = random.randint(1, 3)
            offspring = parent1[:crossover_point] + parent2[crossover_point:]

            if random.random() < MUTATION_PROBABILITY:
                gene_idx = random.randint(0, 3)
                offspring[gene_idx] += random.uniform(-5, 5)
                offspring[gene_idx] = max(0, offspring[gene_idx])  # Ensure non-negative

            next_population.append(offspring)

        self.gene_pool = next_population
        self.fitness_records = []
        self.current_generation += 1
        self.current_individual = 0

        if self.current_generation <= EVOLUTION_CYCLES:
            self.launch_individual()
        else:
            self.finalize_training()

    def simulation_loop(self):
        """Main simulation execution loop with discrete movement"""
        if DISCRETE_MODE:
            # DISCRETE STEP-BASED MOVEMENT
            
            if self.robot_pose["movement_state"] == "IDLE":
                # Transition to PLANNING
                self.robot_pose["movement_state"] = "PLANNING"
                self.step_counter += 1
            
            elif self.robot_pose["movement_state"] == "PLANNING":
                x, y, theta = self.robot_pose["x"], self.robot_pose["y"], self.robot_pose["theta"]
                sensors = self.sense_environment(x, y, theta)
                
                dx, dy = TARGET_POSITION[0] - x, TARGET_POSITION[1] - y
                target_angle = math.atan2(dy, dx)
                bearing_error = (target_angle - theta + math.pi) % (2 * math.pi) - math.pi
                
                turn_angle, step_distance = self.controller.infer_action(sensors, bearing_error)
                
                # Calculate target position
                new_theta = theta + turn_angle
                self.robot_pose["target_theta"] = new_theta
                self.robot_pose["target_x"] = x + math.cos(new_theta) * step_distance
                self.robot_pose["target_y"] = y + math.sin(new_theta) * step_distance
                
                self.robot_pose["start_x"] = x
                self.robot_pose["start_y"] = y
                self.robot_pose["start_theta"] = theta
                self.robot_pose["animation_frame"] = 0
                
                self.robot_pose["movement_state"] = "EXECUTING"
            
            elif self.robot_pose["movement_state"] == "EXECUTING":
                self.robot_pose["animation_frame"] += 1
                progress = self.robot_pose["animation_frame"] / STEP_ANIMATION_FRAMES
                
                if progress >= 1.0:
                    # Snap to target
                    self.robot_pose["x"] = self.robot_pose["target_x"]
                    self.robot_pose["y"] = self.robot_pose["target_y"]
                    self.robot_pose["theta"] = self.robot_pose["target_theta"]
                    self.robot_pose["movement_state"] = "COMPLETE"
                else:
                    # Interpolate (visual only)
                    self.robot_pose["x"] = (self.robot_pose["start_x"] + 
                        (self.robot_pose["target_x"] - self.robot_pose["start_x"]) * progress)
                    self.robot_pose["y"] = (self.robot_pose["start_y"] + 
                        (self.robot_pose["target_y"] - self.robot_pose["start_y"]) * progress)
                    self.robot_pose["theta"] = (self.robot_pose["start_theta"] + 
                        (self.robot_pose["target_theta"] - self.robot_pose["start_theta"]) * progress)
                
                # Render robot
                new_x, new_y, new_theta = self.robot_pose["x"], self.robot_pose["y"], self.robot_pose["theta"]
                r = 10
                self.simulation_canvas.coords(self.robot_polygon,
                                              new_x + r * math.cos(new_theta), new_y + r * math.sin(new_theta),
                                              new_x + r * math.cos(new_theta + 2.5), new_y + r * math.sin(new_theta + 2.5),
                                              new_x + r * math.cos(new_theta - 2.5), new_y + r * math.sin(new_theta - 2.5))
            
            elif self.robot_pose["movement_state"] == "COMPLETE":
                # Check collision
                x, y = self.robot_pose["x"], self.robot_pose["y"]
                collision = False
                for x1, y1, x2, y2 in OBSTACLE_MAP:
                    if x1 < x < x2 and y1 < y < y2:
                        collision = True
                        break
                
                if collision:
                    self.complete_individual("CRASH")
                    return
                
                dx = TARGET_POSITION[0] - x
                dy = TARGET_POSITION[1] - y
                if math.hypot(dx, dy) < 15:
                    self.complete_individual("SUCCESS")
                    return
                
                # Increment discrete step counter
                self.robot_pose["discrete_steps"] += 1
                
                # Check step limit
                if self.robot_pose["discrete_steps"] >= STEP_LIMIT:
                    self.complete_individual("TIMEOUT")
                    return
                
                # Mark trajectory with glowing dots
                if self.robot_pose["discrete_steps"] % 5 == 0:
                    trace = self.simulation_canvas.create_oval(x - 2, y - 2, x + 2, y + 2,
                                                               fill="#22d3ee", outline="#06b6d4", width=1)
                    self.trajectory.append(trace)
                
                # Track exploration
                self.exploration_grid.add((int(x // 10), int(y // 10)))
                
                # Transition to PAUSE state
                self.robot_pose["movement_state"] = "PAUSE"
                self.robot_pose["pause_start_time"] = self.master.tk.call('clock', 'milliseconds')
            
            elif self.robot_pose["movement_state"] == "PAUSE":
                # Wait for PAUSE_BETWEEN_STEPS milliseconds
                current_time = self.master.tk.call('clock', 'milliseconds')
                elapsed = current_time - self.robot_pose["pause_start_time"]
                
                if elapsed >= PAUSE_BETWEEN_STEPS:
                    # Pause complete, return to IDLE
                    self.robot_pose["movement_state"] = "IDLE"
            
            # Continue simulation with ultra-fast updates (1ms for maximum speed - 20 gens in ~30 sec)
            self.master.after(1, self.simulation_loop)
            
        else:
            # CONTINUOUS MOVEMENT (LEGACY)
            x, y, theta = self.robot_pose["x"], self.robot_pose["y"], self.robot_pose["theta"]
            sensors = self.sense_environment(x, y, theta)

            dx, dy = TARGET_POSITION[0] - x, TARGET_POSITION[1] - y
            target_angle = math.atan2(dy, dx)
            bearing_error = (target_angle - theta + math.pi) % (2 * math.pi) - math.pi

            speed, steering = self.controller.infer_action(sensors, bearing_error)
            speed *= 2.0

            new_theta = theta + steering
            new_x = x + math.cos(new_theta) * speed
            new_y = y + math.sin(new_theta) * speed

            # Collision check
            collision = False
            for x1, y1, x2, y2 in OBSTACLE_MAP:
                if x1 < new_x < x2 and y1 < new_y < y2:
                    collision = True
                    break

            # Render robot
            r = 10
            self.simulation_canvas.coords(self.robot_polygon,
                                          new_x + r * math.cos(new_theta), new_y + r * math.sin(new_theta),
                                          new_x + r * math.cos(new_theta + 2.5), new_y + r * math.sin(new_theta + 2.5),
                                          new_x + r * math.cos(new_theta - 2.5), new_y + r * math.sin(new_theta - 2.5))

            if self.step_counter % 5 == 0:
                trace = self.simulation_canvas.create_oval(new_x - 1, new_y - 1, new_x + 1, new_y + 1,
                                                           fill="#3498db", outline="")
                self.trajectory.append(trace)

            # Check termination conditions
            if collision:
                self.complete_individual("CRASH")
                return
            elif math.hypot(dx, dy) < 15:
                self.complete_individual("SUCCESS")
                return
            elif self.step_counter >= STEP_LIMIT:
                self.complete_individual("TIMEOUT")
                return

            # Continue
            self.robot_pose = {"x": new_x, "y": new_y, "theta": new_theta}
            self.exploration_grid.add((int(new_x // 10), int(new_y // 10)))
            self.step_counter += 1

            self.master.after(1, self.simulation_loop)

    def finalize_training(self):
        """Save results and exit"""
        if not self.global_best_chromosome:
            print("Training incomplete - no data to save")
            self.master.destroy()
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"optimized_fuzzy_params_{timestamp}.json"

        results = {
            "best_chromosome": self.global_best_chromosome,
            "best_fitness": self.global_best_fitness,
            "generations_completed": self.current_generation - 1,
            "timestamp": timestamp
        }

        with open("best_params.json", "w") as f:
            json.dump(self.global_best_chromosome, f)

        with open(filename, "w") as f:
            json.dump(results, f, indent=2)

        print(f"\n{'=' * 60}")
        print(f"TRAINING COMPLETED SUCCESSFULLY")
        print(f"{'=' * 60}")
        print(f"Best Chromosome: {self.global_best_chromosome}")
        print(f"Best Fitness: {self.global_best_fitness:.2f}")
        print(f"Results saved to: {filename}")
        print(f"{'=' * 60}\n")

        self.master.destroy()


if __name__ == "__main__":
    print("=" * 60)
    print("🧬 GA-FUZZY HYBRID OPTIMIZATION SYSTEM")
    print("=" * 60)
    print("\n📍 Available Maps:")
    print("  1. SIMPLE  - 8 obstacles, wider passages, easier navigation")
    print("  2. COMPLEX - 18 obstacles, narrow passages, challenging maze")
    print()
    
    # Interactive map selection
    while True:
        choice = input("Select map (1 for SIMPLE, 2 for COMPLEX): ").strip()
        if choice == '1':
            CURRENT_MAP = SIMPLE_MAP
            map_name = "SIMPLE"
            break
        elif choice == '2':
            CURRENT_MAP = COMPLEX_MAP
            map_name = "COMPLEX"
            break
        else:
            print("❌ Invalid choice. Please enter 1 or 2.")
    
    OBSTACLE_MAP = CURRENT_MAP
    
    # Save map selection to config file for other programs
    with open("map_config.json", "w") as f:
        json.dump({"selected_map": map_name}, f)
    
    print(f"\n✅ Selected Map: {map_name}")
    print(f"📊 Total Obstacles: {len(OBSTACLE_MAP)}")
    print(f"🎯 Target Position: {TARGET_POSITION}")
    print(f"🤖 Start Position: ({INITIAL_STATE[0]}, {INITIAL_STATE[1]})")
    print(f"💾 Map config saved for other programs")
    print("=" * 60)
    print("\n🚀 Launching GUI...\n")
    
    root = tk.Tk()
    trainer = GeneticAlgorithmTrainer(root)
    root.mainloop()
