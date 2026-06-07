import tkinter as tk
from tkinter import ttk
import math
import json
import os
from datetime import datetime
import argparse
import sys
import time

# ==========================================
# ENVIRONMENT CONFIGURATION
# ==========================================
TARGET_POSITION = (200, 40)
INITIAL_STATE = (360, 460, 3.14)

# DISCRETE MOVEMENT CONFIGURATION
DISCRETE_MODE = True          # Enable discrete step-based movement
STEP_SIZE = 20                # Pixels per discrete step
STEP_ANIMATION_FRAMES = 1     # Frames to animate each step (1 = instant snap)
PAUSE_BETWEEN_STEPS = 300     # Milliseconds between steps (planning time)

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

# Baseline parameters (manually tuned)
BASELINE_PARAMS = [40, 10, 50, 40]




# ==========================================
# UNIFIED FUZZY CONTROLLER
# ==========================================
class UnifiedFuzzyController:
    def __init__(self, parameters):
        self.params = parameters
        near_max, med_min, med_max, far_min = parameters

        self.dist_near = [0, 0, near_max]
        self.dist_medium = [med_min, 40, med_max]
        self.dist_far = [far_min, 100, 1000]

        self.angle_cw = [-3.14, -1.0, -0.1]
        self.angle_straight = [-0.3, 0.0, 0.3]
        self.angle_ccw = [0.1, 1.0, 3.14]

        self.steering_outputs = {
            "Sharp_CW": -0.8, "Gentle_CW": -0.3, "Neutral": 0.0,
            "Gentle_CCW": 0.3, "Sharp_CCW": 0.8
        }
        self.velocity_outputs = {
            "Stop": 0.0, "Slow": 2.0, "Medium": 4.0, "Fast": 7.0
        }

    def triangular_mf(self, x, params):
        a, b, c = params
        if x <= a or x >= c: return 0.0
        if a < x <= b: return (x - a) / (b - a)
        return (c - x) / (c - b)

    def process(self, sensors, goal_angle):
        def get_memberships(d):
            return {
                "N": self.triangular_mf(d, self.dist_near),
                "M": self.triangular_mf(d, self.dist_medium),
                "F": self.triangular_mf(d, self.dist_far)
            }

        s_mf = [get_memberships(sensor) for sensor in sensors]

        angle_cw_mf = self.triangular_mf(goal_angle, self.angle_cw)
        angle_straight_mf = self.triangular_mf(goal_angle, self.angle_straight)
        angle_ccw_mf = self.triangular_mf(goal_angle, self.angle_ccw)

        steer_activations = {k: 0.0 for k in self.steering_outputs}
        speed_activations = {k: 0.0 for k in self.velocity_outputs}

        def fire_rule(strength, steer_action, speed_action):
            steer_activations[steer_action] = max(steer_activations[steer_action], strength)
            speed_activations[speed_action] = max(speed_activations[speed_action], strength)

        # Fuzzy rule base
        if sensors[1] < sensors[2]:
            fire_rule(s_mf[0]["N"], "Sharp_CW", "Slow")
        else:
            fire_rule(s_mf[0]["N"], "Sharp_CCW", "Slow")

        fire_rule(min(s_mf[3]["N"], s_mf[4]["N"]), "Neutral", "Medium")
        fire_rule(min(s_mf[3]["N"], s_mf[4]["F"]), "Gentle_CW", "Medium")
        fire_rule(min(s_mf[4]["N"], s_mf[3]["F"]), "Gentle_CCW", "Medium")
        fire_rule(s_mf[1]["N"], "Gentle_CW", "Slow")
        fire_rule(s_mf[2]["N"], "Gentle_CCW", "Slow")

        fire_rule(min(s_mf[4]["N"], s_mf[2]["N"], s_mf[0]["M"]), "Gentle_CCW", "Slow")
        fire_rule(min(s_mf[3]["N"], s_mf[1]["N"], s_mf[0]["M"]), "Gentle_CW", "Slow")

        clear_path = min(max(s_mf[0]["F"], s_mf[0]["M"]),
                         max(s_mf[1]["F"], s_mf[1]["M"]),
                         max(s_mf[2]["F"], s_mf[2]["M"]))
        velocity_choice = "Fast" if min(s_mf[0]["F"], s_mf[1]["F"]) > 0.5 else "Medium"
        fire_rule(min(clear_path, angle_ccw_mf), "Gentle_CCW", velocity_choice)
        fire_rule(min(clear_path, angle_cw_mf), "Gentle_CW", velocity_choice)
        fire_rule(min(clear_path, angle_straight_mf), "Neutral", velocity_choice)

        # Defuzzification using centroid method
        steer_num = sum(w * self.steering_outputs[a] for a, w in steer_activations.items())
        steer_den = sum(steer_activations.values())
        final_steering = steer_num / steer_den if steer_den != 0 else 0.0

        speed_num = sum(w * self.velocity_outputs[s] for s, w in speed_activations.items())
        speed_den = sum(speed_activations.values())
        final_speed = 2.0 if speed_den == 0 else speed_num / speed_den

        # Convert to discrete command: (turn_angle, step_distance)
        # In discrete mode, we normalize the speed to a step distance
        if DISCRETE_MODE:
            # Steering determines the turn angle
            turn_angle = final_steering
            # Speed determines if we take a full step, half step, or cautious step
            if final_speed > 5.0:
                step_distance = STEP_SIZE  # Full step
            elif final_speed > 3.0:
                step_distance = STEP_SIZE * 0.75  # Moderate step
            elif final_speed > 1.0:
                step_distance = STEP_SIZE * 0.5  # Cautious step
            else:
                step_distance = STEP_SIZE * 0.25  # Minimal step
            return turn_angle, step_distance
        else:
            # Continuous mode (legacy)
            return final_speed, final_steering


# ==========================================
# COMPARISON ANALYSIS INTERFACE
# ==========================================
class ComparisonAnalyzer:
    def __init__(self, master):
        self.master = master
        self.master.title("⚔️ Fuzzy vs GA-Fuzzy Performance Showdown")
        self.master.geometry("1200x750")
        self.master.configure(bg="#0f172a")

        # Load optimized parameters
        self.optimized_params = BASELINE_PARAMS
        if os.path.exists("best_params.json"):
            try:
                with open("best_params.json", "r") as f:
                    self.optimized_params = json.load(f)
                print("✓ Optimized parameters loaded successfully")
            except Exception as e:
                print(f"⚠ Error loading parameters: {e}")
                print("Using baseline parameters instead")
        else:
            print("⚠ No optimized parameters found. Run GA trainer first.")

        # Modern title bar with gradient effect
        title_frame = tk.Frame(master, bg="#1e3a8a", height=95)
        title_frame.pack(fill=tk.X)
        title_frame.pack_propagate(False)

        tk.Label(title_frame, text="⚔️ PERFORMANCE COMPARISON: FUZZY vs HYBRID GA-FUZZY",
                 font=("Segoe UI", 19, "bold"), bg="#1e3a8a", fg="#ffffff").pack(pady=12)
        
        # Display current map with modern styling
        map_info = tk.Label(title_frame, text=f"🗺️ {'SIMPLE' if CURRENT_MAP == SIMPLE_MAP else 'COMPLEX'} MAP • {len(OBSTACLE_MAP)} obstacles",
                           font=("Segoe UI", 10), bg="#1e3a8a", fg="#93c5fd")
        map_info.pack(pady=3)

        # Main comparison area with modern layout
        comparison_container = tk.Frame(master, bg="#0f172a")
        comparison_container.pack(fill=tk.BOTH, expand=True, padx=15, pady=12)

        # LEFT PANEL - Baseline Fuzzy with modern design
        baseline_frame = tk.Frame(comparison_container, bg="#1e293b", bd=0, relief=tk.FLAT)
        baseline_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=8)

        baseline_header = tk.Frame(baseline_frame, bg="#3b82f6", height=55)
        baseline_header.pack(fill=tk.X)
        baseline_header.pack_propagate(False)
        tk.Label(baseline_header, text="🔷 BASELINE FUZZY LOGIC",
                 font=("Segoe UI", 15, "bold"), bg="#3b82f6", fg="white").pack(pady=14)

        self.canvas_baseline = tk.Canvas(baseline_frame, width=500, height=500,
                                         bg="#0f172a", highlightthickness=2, highlightbackground="#475569")
        self.canvas_baseline.pack(padx=8, pady=8)

        baseline_info = tk.Frame(baseline_frame, bg="#1e293b")
        baseline_info.pack(fill=tk.X, padx=12, pady=8)

        self.label_baseline_params = tk.Label(baseline_info,
                                              text=self.format_parameters(BASELINE_PARAMS),
                                              font=("Consolas", 10), bg="#1e293b",
                                              fg="#e2e8f0", justify=tk.LEFT)
        self.label_baseline_params.pack(pady=6)

        self.status_baseline = tk.Label(baseline_info, text="⏳ Initializing...",
                                        font=("Segoe UI", 12, "bold"), fg="#3b82f6", bg="#1e293b")
        self.status_baseline.pack(pady=8)

        # RIGHT PANEL - GA-Optimized Fuzzy with modern design
        optimized_frame = tk.Frame(comparison_container, bg="#1e293b", bd=0, relief=tk.FLAT)
        optimized_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=8)

        optimized_header = tk.Frame(optimized_frame, bg="#10b981", height=55)
        optimized_header.pack(fill=tk.X)
        optimized_header.pack_propagate(False)
        tk.Label(optimized_header, text="⚡ GA-OPTIMIZED FUZZY",
                 font=("Segoe UI", 15, "bold"), bg="#10b981", fg="white").pack(pady=14)

        self.canvas_optimized = tk.Canvas(optimized_frame, width=500, height=500,
                                          bg="#0f172a", highlightthickness=2, highlightbackground="#475569")
        self.canvas_optimized.pack(padx=8, pady=8)

        optimized_info = tk.Frame(optimized_frame, bg="#1e293b")
        optimized_info.pack(fill=tk.X, padx=12, pady=8)

        self.label_optimized_params = tk.Label(optimized_info,
                                               text=self.format_parameters(self.optimized_params),
                                               font=("Consolas", 10), bg="#1e293b",
                                               fg="#e2e8f0", justify=tk.LEFT)
        self.label_optimized_params.pack(pady=6)

        self.status_optimized = tk.Label(optimized_info, text="⏳ Initializing...",
                                         font=("Segoe UI", 12, "bold"), fg="#10b981", bg="#1e293b")
        self.status_optimized.pack(pady=8)

        # Control panel at bottom with modern styling
        control_frame = tk.Frame(master, bg="#1e3a8a", height=65)
        control_frame.pack(fill=tk.X, side=tk.BOTTOM)
        control_frame.pack_propagate(False)

        tk.Button(control_frame, text="🔄 RESTART COMPARISON",
                  command=self.restart_comparison, bg="#dc2626", fg="white",
                  font=("Segoe UI", 12, "bold"), width=25, height=2,
                  relief=tk.FLAT, bd=0, cursor="hand2").pack(pady=12)

        # Initialize run history tracking
        self.baseline_history = []  # List of outcomes: 'SUCCESS', 'COLLISION', 'TIMEOUT'
        self.optimized_history = []  # List of outcomes: 'SUCCESS', 'COLLISION', 'TIMEOUT'
        self.run_count = 0

        # Initialize robots
        self.initialize_simulation()
        self.execute_simulation()

    def format_parameters(self, params):
        return (f"Near Threshold: {params[0]:.2f} px\n"
                f"Medium Start:   {params[1]:.2f} px\n"
                f"Medium End:     {params[2]:.2f} px\n"
                f"Far Start:      {params[3]:.2f} px")

    def initialize_simulation(self):
        # Preserve history references if this is a restart
        baseline_history_ref = getattr(self, 'baseline_history', [])
        optimized_history_ref = getattr(self, 'optimized_history', [])
        
        # Robot A - Baseline
        self.robot_baseline = {
            "x": INITIAL_STATE[0], "y": INITIAL_STATE[1], "theta": INITIAL_STATE[2],
            "active": True, "steps": 0, "controller": UnifiedFuzzyController(BASELINE_PARAMS),
            "canvas": self.canvas_baseline, "shape": None, "sensors": [],
            "status_label": self.status_baseline, "color": "#1f6feb", "path": [],
            # Discrete movement state
            "movement_state": "IDLE",  # IDLE, PLANNING, EXECUTING, COMPLETE
            "target_x": INITIAL_STATE[0],
            "target_y": INITIAL_STATE[1],
            "target_theta": INITIAL_STATE[2],
            "animation_frame": 0,
            "start_x": INITIAL_STATE[0],
            "start_y": INITIAL_STATE[1],
            "start_theta": INITIAL_STATE[2],
            "discrete_steps": 0,  # Count of discrete steps taken
            # Timing and tracking
            "start_time": time.time(),
            "completion_time": None,
            "outcome": None,  # 'SUCCESS', 'COLLISION', or 'TIMEOUT'
            "history_ref": baseline_history_ref
        }

        # Robot B - GA-Optimized
        self.robot_optimized = {
            "x": INITIAL_STATE[0], "y": INITIAL_STATE[1], "theta": INITIAL_STATE[2],
            "active": True, "steps": 0, "controller": UnifiedFuzzyController(self.optimized_params),
            "canvas": self.canvas_optimized, "shape": None, "sensors": [],
            "status_label": self.status_optimized, "color": "#238636", "path": [],
            # Discrete movement state
            "movement_state": "IDLE",
            "target_x": INITIAL_STATE[0],
            "target_y": INITIAL_STATE[1],
            "target_theta": INITIAL_STATE[2],
            "animation_frame": 0,
            "start_x": INITIAL_STATE[0],
            "start_y": INITIAL_STATE[1],
            "start_theta": INITIAL_STATE[2],
            "discrete_steps": 0,
            # Timing and tracking
            "start_time": time.time(),
            "completion_time": None,
            "outcome": None,  # 'SUCCESS', 'COLLISION', or 'TIMEOUT'
            "history_ref": optimized_history_ref
        }

        self.render_environment(self.canvas_baseline)
        self.render_environment(self.canvas_optimized)
        self.setup_robot_graphics(self.robot_baseline)
        self.setup_robot_graphics(self.robot_optimized)

    def render_environment(self, canvas):
        """Draw maze and goal with modern colors"""
        canvas.delete("all")
        # Glowing goal design
        canvas.create_oval(TARGET_POSITION[0] - 15, TARGET_POSITION[1] - 15,
                           TARGET_POSITION[0] + 15, TARGET_POSITION[1] + 15,
                           fill="#10b981", outline="#34d399", width=4)
        canvas.create_oval(TARGET_POSITION[0] - 8, TARGET_POSITION[1] - 8,
                           TARGET_POSITION[0] + 8, TARGET_POSITION[1] + 8,
                           fill="#6ee7b7", outline="")

        # Modern gradient-style obstacles
        for i, obstacle in enumerate(OBSTACLE_MAP):
            if i < 4:  # Boundaries
                fill_color = "#1e293b"
                outline_color = "#475569"
            elif i < 9:  # Mid obstacles
                fill_color = "#334155"
                outline_color = "#64748b"
            else:  # Interior obstacles
                fill_color = "#4f46e5"
                outline_color = "#6366f1"
            canvas.create_rectangle(obstacle, fill=fill_color, outline=outline_color, 
                                   width=2, tags="obstacle")


    def setup_robot_graphics(self, robot):
        """Initialize robot visualization with enhanced design"""
        canvas = robot["canvas"]
        # Enhanced robot with outline
        robot["shape"] = canvas.create_polygon(0, 0, 0, 0, fill=robot["color"],
                                               outline="#ffffff", width=3)
        # Glowing sensor beams
        robot["sensors"] = [canvas.create_line(0, 0, 0, 0, fill="#22d3ee", width=2, dash=(4, 2))
                            for _ in range(5)]
        self.update_robot_visual(robot)

    def restart_comparison(self):
        """Reset both simulations while preserving run history"""
        self.robot_baseline["active"] = False
        self.robot_optimized["active"] = False
        self.run_count += 1
        self.master.after(150, self.initialize_simulation)

    def detect_obstacles(self, x, y, theta, canvas, sensor_lines):
        """Range sensor simulation"""
        angles = [0, 0.785, -0.785, 1.57, -1.57]
        distances = []

        for i, offset in enumerate(angles):
            beam_angle = theta + offset
            dx, dy = math.cos(beam_angle), math.sin(beam_angle)
            min_range = 150.0

            for x1, y1, x2, y2 in OBSTACLE_MAP:
                if abs(dx) > 0.001:
                    t1, t2 = (x1 - x) / dx, (x2 - x) / dx
                    if 0 < t1 < min_range and y1 <= y + t1 * dy <= y2: min_range = t1
                    if 0 < t2 < min_range and y1 <= y + t2 * dy <= y2: min_range = t2
                if abs(dy) > 0.001:
                    t3, t4 = (y1 - y) / dy, (y2 - y) / dy
                    if 0 < t3 < min_range and x1 <= x + t3 * dx <= x2: min_range = t3
                    if 0 < t4 < min_range and x1 <= x + t4 * dx <= x2: min_range = t4

            distances.append(min_range)
            if sensor_lines:
                canvas.coords(sensor_lines[i], x, y, x + min_range * dx, y + min_range * dy)

        return distances

    def update_robot(self, robot):
        """Execute discrete step-based movement using state machine"""
        if not robot["active"]:
            return

        x, y, theta = robot["x"], robot["y"], robot["theta"]
        
        if DISCRETE_MODE:
            # DISCRETE STEP-BASED MOVEMENT STATE MACHINE
            
            if robot["movement_state"] == "IDLE":
                # Transition to PLANNING state
                robot["movement_state"] = "PLANNING"
                robot["steps"] += 1
            
            elif robot["movement_state"] == "PLANNING":
                # Get sensor data and compute discrete command
                sensor_data = self.detect_obstacles(x, y, theta, robot["canvas"], robot["sensors"])
                
                dx, dy = TARGET_POSITION[0] - x, TARGET_POSITION[1] - y
                target_heading = math.atan2(dy, dx)
                angle_error = (target_heading - theta + math.pi) % (2 * math.pi) - math.pi
                
                # Get discrete command from fuzzy controller
                turn_angle, step_distance = robot["controller"].process(sensor_data, angle_error)
                
                # Calculate target position for this discrete step
                new_theta = theta + turn_angle
                robot["target_theta"] = new_theta
                robot["target_x"] = x + math.cos(new_theta) * step_distance
                robot["target_y"] = y + math.sin(new_theta) * step_distance
                
                # Save starting position for animation
                robot["start_x"] = x
                robot["start_y"] = y
                robot["start_theta"] = theta
                robot["animation_frame"] = 0
                
                # Transition to EXECUTING state
                robot["movement_state"] = "EXECUTING"
            
            elif robot["movement_state"] == "EXECUTING":
                # Animate the movement over STEP_ANIMATION_FRAMES
                robot["animation_frame"] += 1
                progress = robot["animation_frame"] / STEP_ANIMATION_FRAMES
                
                if progress >= 1.0:
                    # Movement complete - snap to target
                    robot["x"] = robot["target_x"]
                    robot["y"] = robot["target_y"]
                    robot["theta"] = robot["target_theta"]
                    robot["movement_state"] = "COMPLETE"
                else:
                    # Interpolate position (visual only)
                    robot["x"] = robot["start_x"] + (robot["target_x"] - robot["start_x"]) * progress
                    robot["y"] = robot["start_y"] + (robot["target_y"] - robot["start_y"]) * progress
                    robot["theta"] = robot["start_theta"] + (robot["target_theta"] - robot["start_theta"]) * progress
                
                self.update_robot_visual(robot)
            
            elif robot["movement_state"] == "COMPLETE":
                # Check for collision at final position
                x, y = robot["x"], robot["y"]
                crashed = False
                for x1, y1, x2, y2 in OBSTACLE_MAP:
                    if x1 < x < x2 and y1 < y < y2:
                        crashed = True
                        break
                
                if crashed:
                    robot["active"] = False
                    robot["completion_time"] = time.time() - robot["start_time"]
                    robot["outcome"] = "COLLISION"
                    robot["history_ref"].append("COLLISION")
                    
                    # Calculate success rate
                    total_runs = len(robot["history_ref"])
                    successes = robot["history_ref"].count("SUCCESS")
                    success_rate = (successes / total_runs * 100) if total_runs > 0 else 0
                    
                    robot["status_label"].config(
                        text=f"💥 COLLISION! Time: {robot['completion_time']:.1f}s | Steps: {robot['discrete_steps']} | Success Rate: {success_rate:.0f}% ({successes}/{total_runs})",
                        fg="#f85149"
                    )
                    robot["canvas"].itemconfig(robot["shape"], fill="#da3633")
                    return
                
                # Check if goal reached
                dx, dy = TARGET_POSITION[0] - x, TARGET_POSITION[1] - y
                if math.hypot(dx, dy) < 15:
                    robot["active"] = False
                    robot["completion_time"] = time.time() - robot["start_time"]
                    robot["outcome"] = "SUCCESS"
                    robot["history_ref"].append("SUCCESS")
                    
                    # Calculate success rate
                    total_runs = len(robot["history_ref"])
                    successes = robot["history_ref"].count("SUCCESS")
                    success_rate = (successes / total_runs * 100) if total_runs > 0 else 0
                    
                    robot["status_label"].config(
                        text=f"🎯 SUCCESS! Time: {robot['completion_time']:.1f}s | Steps: {robot['discrete_steps']} | Success Rate: {success_rate:.0f}% ({successes}/{total_runs})",
                        fg="#3fb950"
                    )
                    robot["canvas"].itemconfig(robot["shape"], fill="#ffd700") 
                    return
                
                # Mark this discrete step location
                robot["discrete_steps"] += 1
                if robot["discrete_steps"] % 2 == 0:
                    trace = robot["canvas"].create_oval(x - 3, y - 3, x + 3, y + 3,
                                                        fill=robot["color"], outline="white", width=1)
                    robot["path"].append(trace)
                
                # Update status with elapsed time
                elapsed_time = time.time() - robot["start_time"]
                robot["status_label"].config(
                    text=f"⏸ PAUSE - Time: {elapsed_time:.1f}s | Steps: {robot['discrete_steps']} | Distance: {math.hypot(dx, dy):.1f} px"
                )
                
                # Transition to PAUSE state (wait before next planning)
                robot["movement_state"] = "PAUSE"
                robot["pause_start_time"] = self.master.tk.call('clock', 'milliseconds')
            
            elif robot["movement_state"] == "PAUSE":
                # Wait for PAUSE_BETWEEN_STEPS milliseconds
                current_time = self.master.tk.call('clock', 'milliseconds')
                elapsed = current_time - robot["pause_start_time"]
                
                if elapsed >= PAUSE_BETWEEN_STEPS:
                    # Pause complete, return to IDLE
                    robot["movement_state"] = "IDLE"
                
        else:
            # CONTINUOUS MOVEMENT (LEGACY MODE)
            sensor_data = self.detect_obstacles(x, y, theta, robot["canvas"], robot["sensors"])

            dx, dy = TARGET_POSITION[0] - x, TARGET_POSITION[1] - y
            target_heading = math.atan2(dy, dx)
            angle_error = (target_heading - theta + math.pi) % (2 * math.pi) - math.pi

            velocity, steering = robot["controller"].process(sensor_data, angle_error)

            new_theta = theta + steering
            new_x = x + math.cos(new_theta) * velocity
            new_y = y + math.sin(new_theta) * velocity

            # Collision detection
            crashed = False
            for x1, y1, x2, y2 in OBSTACLE_MAP:
                if x1 < new_x < x2 and y1 < new_y < y2:
                    crashed = True
                    break

            if crashed:
                robot["active"] = False
                robot["completion_time"] = time.time() - robot["start_time"]
                robot["outcome"] = "COLLISION"
                robot["history_ref"].append("COLLISION")
                
                # Calculate success rate
                total_runs = len(robot["history_ref"])
                successes = robot["history_ref"].count("SUCCESS")
                success_rate = (successes / total_runs * 100) if total_runs > 0 else 0
                
                robot["status_label"].config(
                    text=f"💥 COLLISION! Time: {robot['completion_time']:.1f}s | Steps: {robot['steps']} | Success Rate: {success_rate:.0f}% ({successes}/{total_runs})",
                    fg="#f85149"
                )
                robot["canvas"].itemconfig(robot["shape"], fill="#da3633")
            elif math.hypot(dx, dy) < 15:
                robot["active"] = False
                robot["completion_time"] = time.time() - robot["start_time"]
                robot["outcome"] = "SUCCESS"
                robot["history_ref"].append("SUCCESS")
                
                # Calculate success rate
                total_runs = len(robot["history_ref"])
                successes = robot["history_ref"].count("SUCCESS")
                success_rate = (successes / total_runs * 100) if total_runs > 0 else 0
                
                robot["status_label"].config(
                    text=f"🎯 SUCCESS! Time: {robot['completion_time']:.1f}s | Steps: {robot['steps']} | Success Rate: {success_rate:.0f}% ({successes}/{total_runs})",
                    fg="#3fb950"
                )
                robot["canvas"].itemconfig(robot["shape"], fill="#ffd700")
            else:
                robot["x"], robot["y"], robot["theta"] = new_x, new_y, new_theta
                robot["steps"] += 1
                elapsed_time = time.time() - robot["start_time"]
                robot["status_label"].config(
                    text=f"🏃 Running... Time: {elapsed_time:.1f}s | Steps: {robot['steps']} | Distance: {math.hypot(dx, dy):.1f} px"
                )
                self.update_robot_visual(robot)

                # Draw trajectory
                if robot["steps"] % 6 == 0:
                    trace = robot["canvas"].create_oval(new_x - 2, new_y - 2, new_x + 2, new_y + 2,
                                                        fill=robot["color"], outline="")
                    robot["path"].append(trace)

    def update_robot_visual(self, robot):
        """Render robot shape"""
        x, y, theta = robot["x"], robot["y"], robot["theta"]
        size = 13
        robot["canvas"].coords(robot["shape"],
                               x + size * math.cos(theta), y + size * math.sin(theta),
                               x + size * math.cos(theta + 2.6), y + size * math.sin(theta + 2.6),
                               x + size * math.cos(theta - 2.6), y + size * math.sin(theta - 2.6)
                               )


    def execute_simulation(self):
        """Main simulation loop"""
        self.update_robot(self.robot_baseline)
        self.update_robot(self.robot_optimized)
        self.master.after(30, self.execute_simulation)


if __name__ == "__main__":
    print("=" * 60)
    print("⚖️  COMPARISON ANALYZER: FUZZY vs GA-FUZZY HYBRID")
    print("=" * 60)
    
    # Load map from config file (set by trainFuzzyGA.py)
    try:
        with open("map_config.json", "r") as f:
            config = json.load(f)
            map_name = config.get("selected_map", "SIMPLE")
    except (FileNotFoundError, json.JSONDecodeError):
        print("⚠️  No map config found. Using SIMPLE map.")
        print("💡 Run trainFuzzyGA.py first to set the map.")
        map_name = "SIMPLE"
    
    # Set map based on config
    if map_name == "SIMPLE":
        CURRENT_MAP = SIMPLE_MAP
    else:
        CURRENT_MAP = COMPLEX_MAP
    
    OBSTACLE_MAP = CURRENT_MAP
    
    print(f"💾 Loaded map from training config: {map_name}")
    print(f"📊 Total Obstacles: {len(OBSTACLE_MAP)}")
    print("=" * 60)
    print("🚀 Launching GUI...\n")
    
    root = tk.Tk()
    analyzer = ComparisonAnalyzer(root)
    root.mainloop()
