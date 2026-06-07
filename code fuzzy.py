import tkinter as tk
from tkinter import ttk
import math
import argparse
import sys
import json

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


# ==========================================
# FUZZY CONTROLLER ENGINE
# ==========================================
class FuzzyNavigationController:
    def __init__(self):
        # Input fuzzy sets - Distance measurements
        self.proximity_near = [0, 0, 40]
        self.proximity_moderate = [10, 40, 50]
        self.proximity_distant = [40, 100, 1000]

        # Input fuzzy sets - Angle measurements
        self.heading_clockwise = [-3.14, -1.0, -0.1]
        self.heading_center = [-0.3, 0.0, 0.3]
        self.heading_counter = [0.1, 1.0, 3.14]

        # Output actions - Steering
        self.steering_actions = {
            "Sharp_CW": -0.8, "Mild_CW": -0.3, "Forward": 0.0,
            "Mild_CCW": 0.3, "Sharp_CCW": 0.8
        }

        # Output actions - Velocity
        self.velocity_levels = {
            "Halt": 0.0, "Creep": 2.0, "Cruise": 4.0, "Rush": 7.0
        }

    def triangular_membership(self, value, parameters):
        """Calculate triangular membership function"""
        left, peak, right = parameters
        if value <= left or value >= right:
            return 0.0
        if left < value <= peak:
            return (value - left) / (peak - left)
        return (right - value) / (right - peak)

    def process_inputs(self, sensor_data, target_bearing):
        # Fuzzify sensor readings
        def calculate_memberships(distance):
            return {
                "Near": self.triangular_membership(distance, self.proximity_near),
                "Moderate": self.triangular_membership(distance, self.proximity_moderate),
                "Distant": self.triangular_membership(distance, self.proximity_distant)
            }

        sensor_fuzzy = [calculate_memberships(reading) for reading in sensor_data]

        # Fuzzify target angle
        angle_CW = self.triangular_membership(target_bearing, self.heading_clockwise)
        angle_Center = self.triangular_membership(target_bearing, self.heading_center)
        angle_CCW = self.triangular_membership(target_bearing, self.heading_counter)

        # Initialize rule strengths
        steer_strengths = {action: 0.0 for action in self.steering_actions}
        speed_strengths = {level: 0.0 for level in self.velocity_levels}

        def activate_rule(strength, steer_cmd, speed_cmd):
            """Fire a fuzzy rule"""
            steer_strengths[steer_cmd] = max(steer_strengths[steer_cmd], strength)
            speed_strengths[speed_cmd] = max(speed_strengths[speed_cmd], strength)

        # EMERGENCY AVOIDANCE - Front obstacle critical
        emergency_level = sensor_fuzzy[0]["Near"]
        if sensor_data[1] < sensor_data[2]:
            activate_rule(emergency_level, "Sharp_CW", "Creep")
        else:
            activate_rule(emergency_level, "Sharp_CCW", "Creep")

        # NARROW PASSAGE - Both sides constrained
        passage_constraint = min(sensor_fuzzy[3]["Near"], sensor_fuzzy[4]["Near"])
        activate_rule(passage_constraint, "Forward", "Cruise")

        # BALANCE CONTROL - Maintain center position
        activate_rule(min(sensor_fuzzy[3]["Near"], sensor_fuzzy[4]["Distant"]), "Mild_CW", "Cruise")
        activate_rule(min(sensor_fuzzy[4]["Near"], sensor_fuzzy[3]["Distant"]), "Mild_CCW", "Cruise")

        # DIAGONAL CORRECTION
        activate_rule(sensor_fuzzy[1]["Near"], "Mild_CW", "Creep")
        activate_rule(sensor_fuzzy[2]["Near"], "Mild_CCW", "Creep")

        # TARGET APPROACH - When path is clear
        clear_ahead = max(sensor_fuzzy[0]["Distant"], sensor_fuzzy[0]["Moderate"])
        clear_left_diag = max(sensor_fuzzy[1]["Distant"], sensor_fuzzy[1]["Moderate"])
        clear_right_diag = max(sensor_fuzzy[2]["Distant"], sensor_fuzzy[2]["Moderate"])
        path_clearance = min(clear_ahead, clear_left_diag, clear_right_diag)

        velocity_choice = "Rush" if min(sensor_fuzzy[0]["Distant"], sensor_fuzzy[1]["Distant"]) > 0.5 else "Cruise"
        activate_rule(min(path_clearance, angle_CCW), "Mild_CCW", velocity_choice)
        activate_rule(min(path_clearance, angle_CW), "Mild_CW", velocity_choice)
        activate_rule(min(path_clearance, angle_Center), "Forward", velocity_choice)

        # SYMMETRY RULE - Balanced environment
        symmetry_factor = min(sensor_fuzzy[3]["Moderate"], sensor_fuzzy[4]["Moderate"])
        activate_rule(symmetry_factor, "Forward", "Cruise")

        # CORNER NAVIGATION ENHANCEMENT
        right_corner = min(sensor_fuzzy[4]["Near"], sensor_fuzzy[2]["Near"], sensor_fuzzy[0]["Moderate"])
        activate_rule(right_corner, "Mild_CCW", "Creep")

        left_corner = min(sensor_fuzzy[3]["Near"], sensor_fuzzy[1]["Near"], sensor_fuzzy[0]["Moderate"])
        activate_rule(left_corner, "Mild_CW", "Creep")

        # Defuzzification - Center of Gravity method
        steer_numerator = sum(strength * self.steering_actions[action]
                              for action, strength in steer_strengths.items())
        steer_denominator = sum(steer_strengths.values())
        final_steering = steer_numerator / steer_denominator if steer_denominator != 0 else 0.0

        speed_numerator = sum(strength * self.velocity_levels[level]
                              for level, strength in speed_strengths.items())
        speed_denominator = sum(speed_strengths.values())
        final_velocity = 2.0 if speed_denominator == 0 else speed_numerator / speed_denominator

        # Convert to discrete command in discrete mode
        if DISCRETE_MODE:
            # Steering determines the turn angle
            turn_angle = final_steering
            # Velocity determines step distance
            if final_velocity > 5.0:
                step_distance = STEP_SIZE  # Full step
            elif final_velocity > 3.0:
                step_distance = STEP_SIZE * 0.75  # Moderate step
            elif final_velocity > 1.5:
                step_distance = STEP_SIZE * 0.5  # Cautious step
            else:
                step_distance = STEP_SIZE * 0.25  # Minimal step
            return turn_angle, step_distance
        else:
            # Continuous mode (legacy)
            return final_velocity, final_steering


# ==========================================
# SIMULATION INTERFACE
# ==========================================
class NavigationSimulator:
    def __init__(self, master):
        self.master = master
        self.master.title("🤖 Intelligent Fuzzy Navigation System")
        self.master.geometry("650x720")
        self.master.configure(bg="#0f172a")

        # Modern header with gradient effect
        header = tk.Frame(master, bg="#1e3a8a", height=75)
        header.pack(fill=tk.X, pady=(0, 10))
        
        title_label = tk.Label(header, text="🤖 FUZZY LOGIC NAVIGATOR",
                 font=("Segoe UI", 17, "bold"), bg="#1e3a8a", fg="#ffffff")
        title_label.pack(pady=12)
        
        # Display current map with modern styling
        map_info = tk.Label(header, text=f"🗺️ {'SIMPLE' if CURRENT_MAP == SIMPLE_MAP else 'COMPLEX'} MAP • {len(OBSTACLE_MAP)} obstacles",
                           font=("Segoe UI", 9), bg="#1e3a8a", fg="#93c5fd")
        map_info.pack(pady=3)

        # Main canvas with modern dark background
        canvas_frame = tk.Frame(master, bg="#1e293b", bd=0, relief=tk.FLAT)
        canvas_frame.pack(padx=18, pady=5)

        self.display = tk.Canvas(canvas_frame, width=500, height=500, bg="#0f172a",
                                 highlightthickness=2, highlightbackground="#475569")
        self.display.pack(padx=3, pady=3)

        # Info panel with modern dark theme
        info_frame = tk.Frame(master, bg="#1e293b", bd=0, relief=tk.FLAT)
        info_frame.pack(fill=tk.X, padx=18, pady=10)

        self.status_display = tk.Label(info_frame, text="📊 Initializing System...",
                                       font=("Segoe UI", 11), bg="#1e293b", fg="#10b981",
                                       anchor=tk.W, padx=12)
        self.status_display.pack(fill=tk.X, pady=10)

        # Control panel with modern buttons
        control_panel = tk.Frame(master, bg="#0f172a")
        control_panel.pack(pady=8)

        self.reset_btn = tk.Button(control_panel, text="🔄 RESTART", command=self.initialize_robot,
                                   bg="#dc2626", fg="white", font=("Segoe UI", 11, "bold"),
                                   width=12, height=2, relief=tk.FLAT, bd=0, cursor="hand2")
        self.reset_btn.pack(side=tk.LEFT, padx=8)

        self.pause_btn = tk.Button(control_panel, text="⏸ PAUSE", command=self.toggle_simulation,
                                   bg="#f59e0b", fg="white", font=("Segoe UI", 11, "bold"),
                                   width=12, height=2, relief=tk.FLAT, bd=0, cursor="hand2")
        self.pause_btn.pack(side=tk.LEFT, padx=8)

        # Draw environment with modern colors
        # Glowing goal
        self.display.create_oval(TARGET_POSITION[0] - 15, TARGET_POSITION[1] - 15,
                                 TARGET_POSITION[0] + 15, TARGET_POSITION[1] + 15,
                                 fill="#10b981", outline="#34d399", width=4)
        self.display.create_oval(TARGET_POSITION[0] - 8, TARGET_POSITION[1] - 8,
                                 TARGET_POSITION[0] + 8, TARGET_POSITION[1] + 8,
                                 fill="#6ee7b7", outline="")

        # Modern gradient obstacles
        for idx, obstacle in enumerate(OBSTACLE_MAP):
            if idx < 4:  # Boundaries
                color = "#1e293b"
                outline = "#475569"
            elif idx < 9:  # Mid obstacles
                color = "#334155"
                outline = "#64748b"
            else:  # Interior obstacles
                color = "#4f46e5"
                outline = "#6366f1"
            self.display.create_rectangle(obstacle, fill=color, outline=outline, 
                                         width=2, tags="obstacle")

        # Enhanced robot visualization with glow
        self.robot_shape = self.display.create_polygon(0, 0, 0, 0, fill="#06b6d4",
                                                       outline="#22d3ee", width=3)
        self.sensor_beams = [self.display.create_line(0, 0, 0, 0, fill="#22d3ee",
                                                      width=2, dash=(4, 2)) for _ in range(5)]

        self.controller = FuzzyNavigationController()
        self.simulation_paused = False
        self.initialize_robot()
        self.execute_simulation()


    def toggle_simulation(self):
        self.simulation_paused = not self.simulation_paused
        if self.simulation_paused:
            self.pause_btn.config(text="▶ RESUME", bg="#10b981")
        else:
            self.pause_btn.config(text="⏸ PAUSE", bg="#f59e0b")

    def initialize_robot(self):
        self.robot_state = {
            "position_x": INITIAL_STATE[0],
            "position_y": INITIAL_STATE[1],
            "orientation": INITIAL_STATE[2],
            "operational": True,
            # Discrete movement state
            "movement_state": "IDLE",  # IDLE, PLANNING, EXECUTING, COMPLETE
            "target_x": INITIAL_STATE[0],
            "target_y": INITIAL_STATE[1],
            "target_theta": INITIAL_STATE[2],
            "animation_frame": 0,
            "start_x": INITIAL_STATE[0],
            "start_y": INITIAL_STATE[1],
            "start_theta": INITIAL_STATE[2],
            "discrete_steps": 0
        }
        self.display.itemconfig(self.robot_shape, fill="#06b6d4", outline="#22d3ee")
        self.status_display.config(text="📊 System Active | Navigation Ready", fg="#10b981")

    def scan_environment(self, x_pos, y_pos, theta):
        """Simulate 5-beam range sensor array"""
        beam_directions = [0, 0.785, -0.785, 1.57, -1.57]  # Forward, FL, FR, Left, Right
        measurements = []
        SENSOR_RANGE = 150.0

        for beam_idx, angle_offset in enumerate(beam_directions):
            beam_angle = theta + angle_offset
            direction_x, direction_y = math.cos(beam_angle), math.sin(beam_angle)

            minimum_distance = SENSOR_RANGE

            for obs_x1, obs_y1, obs_x2, obs_y2 in OBSTACLE_MAP:
                # Ray-box intersection tests
                if abs(direction_x) > 0.0001:
                    intersect1 = (obs_x1 - x_pos) / direction_x
                    if 0 < intersect1 < minimum_distance:
                        hit_y = y_pos + intersect1 * direction_y
                        if obs_y1 <= hit_y <= obs_y2:
                            minimum_distance = intersect1

                    intersect2 = (obs_x2 - x_pos) / direction_x
                    if 0 < intersect2 < minimum_distance:
                        hit_y = y_pos + intersect2 * direction_y
                        if obs_y1 <= hit_y <= obs_y2:
                            minimum_distance = intersect2

                if abs(direction_y) > 0.0001:
                    intersect3 = (obs_y1 - y_pos) / direction_y
                    if 0 < intersect3 < minimum_distance:
                        hit_x = x_pos + intersect3 * direction_x
                        if obs_x1 <= hit_x <= obs_x2:
                            minimum_distance = intersect3

                    intersect4 = (obs_y2 - y_pos) / direction_y
                    if 0 < intersect4 < minimum_distance:
                        hit_x = x_pos + intersect4 * direction_x
                        if obs_x1 <= hit_x <= obs_x2:
                            minimum_distance = intersect4

            measurements.append(minimum_distance)

            # Visualize sensor beam
            endpoint_x = x_pos + minimum_distance * direction_x
            endpoint_y = y_pos + minimum_distance * direction_y
            self.display.coords(self.sensor_beams[beam_idx], x_pos, y_pos, endpoint_x, endpoint_y)

        return measurements

    def execute_simulation(self):
        if self.simulation_paused:
            self.master.after(100, self.execute_simulation)
            return

        if not self.robot_state["operational"]:
            self.master.after(100, self.execute_simulation)
            return

        if DISCRETE_MODE:
            # DISCRETE STEP-BASED MOVEMENT STATE MACHINE
            
            if self.robot_state["movement_state"] == "IDLE":
                # Transition to PLANNING state
                self.robot_state["movement_state"] = "PLANNING"
            
            elif self.robot_state["movement_state"] == "PLANNING":
                # Get current position
                x = self.robot_state["position_x"]
                y = self.robot_state["position_y"]
                theta = self.robot_state["orientation"]
                
                # Scan environment
                sensor_readings = self.scan_environment(x, y, theta)
                
                # Calculate target direction
                delta_x, delta_y = TARGET_POSITION[0] - x, TARGET_POSITION[1] - y
                desired_heading = math.atan2(delta_y, delta_x)
                heading_error = (desired_heading - theta + math.pi) % (2 * math.pi) - math.pi
                
                # Get discrete command from fuzzy controller
                turn_angle, step_distance = self.controller.process_inputs(sensor_readings, heading_error)
                
                # Calculate target position for this discrete step
                new_theta = theta + turn_angle
                self.robot_state["target_theta"] = new_theta
                self.robot_state["target_x"] = x + math.cos(new_theta) * step_distance
                self.robot_state["target_y"] = y + math.sin(new_theta) * step_distance
                
                # Save starting position for animation
                self.robot_state["start_x"] = x
                self.robot_state["start_y"] = y
                self.robot_state["start_theta"] = theta
                self.robot_state["animation_frame"] = 0
                
                # Transition to EXECUTING
                self.robot_state["movement_state"] = "EXECUTING"
            
            elif self.robot_state["movement_state"] == "EXECUTING":
                # Animate the movement over STEP_ANIMATION_FRAMES
                self.robot_state["animation_frame"] += 1
                progress = self.robot_state["animation_frame"] / STEP_ANIMATION_FRAMES
                
                if progress >= 1.0:
                    # Movement complete - snap to target
                    self.robot_state["position_x"] = self.robot_state["target_x"]
                    self.robot_state["position_y"] = self.robot_state["target_y"]
                    self.robot_state["orientation"] = self.robot_state["target_theta"]
                    self.robot_state["movement_state"] = "COMPLETE"
                else:
                    # Interpolate position (visual only)
                    self.robot_state["position_x"] = (self.robot_state["start_x"] + 
                        (self.robot_state["target_x"] - self.robot_state["start_x"]) * progress)
                    self.robot_state["position_y"] = (self.robot_state["start_y"] + 
                        (self.robot_state["target_y"] - self.robot_state["start_y"]) * progress)
                    self.robot_state["orientation"] = (self.robot_state["start_theta"] + 
                        (self.robot_state["target_theta"] - self.robot_state["start_theta"]) * progress)
                
                # Render robot
                new_x = self.robot_state["position_x"]
                new_y = self.robot_state["position_y"]
                new_theta = self.robot_state["orientation"]
                
                robot_radius = 12
                robot_vertices = [
                    new_x + robot_radius * math.cos(new_theta),
                    new_y + robot_radius * math.sin(new_theta),
                    new_x + robot_radius * math.cos(new_theta + 2.5),
                    new_y + robot_radius * math.sin(new_theta + 2.5),
                    new_x + robot_radius * math.cos(new_theta - 2.5),
                    new_y + robot_radius * math.sin(new_theta - 2.5)
                ]
                self.display.coords(self.robot_shape, *robot_vertices)
            
            elif self.robot_state["movement_state"] == "COMPLETE":
                # Check for collision at final position
                x = self.robot_state["position_x"]
                y = self.robot_state["position_y"]
                
                collision_detected = False
                for obs_x1, obs_y1, obs_x2, obs_y2 in OBSTACLE_MAP:
                    if obs_x1 < x < obs_x2 and obs_y1 < y < obs_y2:
                        collision_detected = True
                        break
                
                if collision_detected:
                    self.robot_state["operational"] = False
                    self.display.itemconfig(self.robot_shape, fill="#c0392b", outline="#922b21")
                    self.status_display.config(
                        text=f"❌ COLLISION! Discrete steps: {self.robot_state['discrete_steps']}", 
                        fg="#e74c3c")
                    return
                
                # Check if goal reached
                delta_x = TARGET_POSITION[0] - x
                delta_y = TARGET_POSITION[1] - y
                distance_to_goal = math.hypot(delta_x, delta_y)
                
                if distance_to_goal < 15:
                    self.robot_state["operational"] = False
                    self.display.itemconfig(self.robot_shape, fill="#27ae60", outline="#1e8449")
                    self.status_display.config(
                        text=f"✅ TARGET REACHED! Steps: {self.robot_state['discrete_steps']}", 
                        fg="#27ae60")
                    return
                
                # Increment discrete step counter
                self.robot_state["discrete_steps"] += 1
                
                # Update status display to show PAUSE
                self.status_display.config(
                    text=f"⏸ PAUSE - Steps: {self.robot_state['discrete_steps']} | Distance: {distance_to_goal:.1f} px",
                    fg="#f39c12"
                )
                
                # Transition to PAUSE state
                self.robot_state["movement_state"] = "PAUSE"
                self.robot_state["pause_start_time"] = self.master.tk.call('clock', 'milliseconds')
            
            elif self.robot_state["movement_state"] == "PAUSE":
                # Wait for PAUSE_BETWEEN_STEPS milliseconds
                current_time = self.master.tk.call('clock', 'milliseconds')
                elapsed = current_time - self.robot_state["pause_start_time"]
                
                if elapsed >= PAUSE_BETWEEN_STEPS:
                    # Pause complete, return to IDLE state
                    self.robot_state["movement_state"] = "IDLE"
            
            # Schedule next update
            self.master.after(30, self.execute_simulation)
            
        else:
            # CONTINUOUS MOVEMENT (LEGACY MODE)
            x, y, theta = (self.robot_state["position_x"],
                           self.robot_state["position_y"],
                           self.robot_state["orientation"])

            sensor_readings = self.scan_environment(x, y, theta)

            # Calculate target direction
            delta_x, delta_y = TARGET_POSITION[0] - x, TARGET_POSITION[1] - y
            desired_heading = math.atan2(delta_y, delta_x)
            heading_error = (desired_heading - theta + math.pi) % (2 * math.pi) - math.pi

            # Get fuzzy controller output
            velocity, steering = self.controller.process_inputs(sensor_readings, heading_error)

            # Update robot state
            new_theta = theta + steering
            new_x = x + math.cos(new_theta) * velocity
            new_y = y + math.sin(new_theta) * velocity

            # Collision detection
            collision_detected = False
            for obs_x1, obs_y1, obs_x2, obs_y2 in OBSTACLE_MAP:
                if obs_x1 < new_x < obs_x2 and obs_y1 < new_y < obs_y2:
                    collision_detected = True
                    break

            if collision_detected:
                self.robot_state["operational"] = False
                self.display.itemconfig(self.robot_shape, fill="#c0392b", outline="#922b21")
                self.status_display.config(text="❌ System Status: COLLISION DETECTED | Mission Failed", fg="#e74c3c")
            else:
                self.robot_state["position_x"] = new_x
                self.robot_state["position_y"] = new_y
                self.robot_state["orientation"] = new_theta

            # Goal check
            distance_to_goal = math.hypot(delta_x, delta_y)
            if distance_to_goal < 15:
                self.robot_state["operational"] = False
                self.display.itemconfig(self.robot_shape, fill="#27ae60", outline="#1e8449")
                self.status_display.config(text="✅ System Status: TARGET REACHED | Mission Successful", fg="#27ae60")

            # Render robot
            robot_radius = 12
            robot_vertices = [
                new_x + robot_radius * math.cos(new_theta),
                new_y + robot_radius * math.sin(new_theta),
                new_x + robot_radius * math.cos(new_theta + 2.5),
                new_y + robot_radius * math.sin(new_theta + 2.5),
                new_x + robot_radius * math.cos(new_theta - 2.5),
                new_y + robot_radius * math.sin(new_theta - 2.5)
            ]
            self.display.coords(self.robot_shape, *robot_vertices)

            if self.robot_state["operational"]:
                self.status_display.config(
                    text=f"📊 Velocity: {velocity:.2f} px/s | Steering: {steering:.3f} rad | Distance: {distance_to_goal:.1f} px",
                    fg="#2ecc71"
                )

            self.master.after(30, self.execute_simulation)


if __name__ == "__main__":
    print("=" * 60)
    print("🤖 FUZZY LOGIC ROBOT NAVIGATOR")
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
    simulator = NavigationSimulator(root)
    root.mainloop()
