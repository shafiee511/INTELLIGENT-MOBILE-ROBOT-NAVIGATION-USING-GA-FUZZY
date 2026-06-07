# file name: fuzzy_logic_visualization.py

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch
import matplotlib.patches as mpatches
import os

# ==========================================
# FUZZY LOGIC PARAMETERS (from your code)
# ==========================================
# Distance membership functions
DIST_NEAR = [0, 0, 40]
DIST_MEDIUM = [10, 40, 50]
DIST_FAR = [40, 100, 1000]

# Angle membership functions
ANGLE_CW = [-3.14, -1.0, -0.1]
ANGLE_STRAIGHT = [-0.3, 0.0, 0.3]
ANGLE_CCW = [0.1, 1.0, 3.14]

# Steering outputs (crisp values) - will create triangular membership functions
STEERING_OUTPUTS = {
    "Sharp_CW": -0.8,
    "Gentle_CW": -0.3,
    "Neutral": 0.0,
    "Gentle_CCW": 0.3,
    "Sharp_CCW": 0.8
}

# Velocity outputs (crisp values) - will create triangular membership functions
VELOCITY_OUTPUTS = {
    "Stop": 0.0,
    "Slow": 2.0,
    "Medium": 4.0,
    "Fast": 7.0
}


# ==========================================
# TRIANGULAR MEMBERSHIP FUNCTION
# ==========================================
def triangular_mf(x, params):
    """Calculate triangular membership function value"""
    a, b, c = params
    if x <= a or x >= c:
        return 0.0
    if a < x <= b:
        return (x - a) / (b - a)
    return (c - x) / (c - b)


def create_output_triangular_mf(center, width=0.3):
    """Create triangular membership function for output centered at a value"""
    return [center - width, center, center + width]


# ==========================================
# CREATE COMPREHENSIVE VISUALIZATION
# ==========================================
def create_fuzzy_visualization():
    """Create a comprehensive figure showing all membership functions"""
    
    # Create figure with subplots
    fig = plt.figure(figsize=(20, 12))
    fig.suptitle('FUZZY LOGIC CONTROL SYSTEM - MEMBERSHIP FUNCTIONS', 
                 fontsize=22, fontweight='bold', y=0.98)
    
    # Create grid for subplots
    gs = fig.add_gridspec(3, 3, hspace=0.35, wspace=0.3, 
                          left=0.05, right=0.95, top=0.93, bottom=0.05)
    
    # ==========================================
    # 1. DISTANCE SENSOR INPUT (Top Left)
    # ==========================================
    ax1 = fig.add_subplot(gs[0, 0])
    x_dist = np.linspace(0, 150, 500)
    
    # Calculate membership values for each fuzzy set
    near_vals = [triangular_mf(x, DIST_NEAR) for x in x_dist]
    medium_vals = [triangular_mf(x, DIST_MEDIUM) for x in x_dist]
    far_vals = [triangular_mf(x, DIST_FAR) for x in x_dist]
    
    # Plot membership functions
    ax1.plot(x_dist, near_vals, 'r-', linewidth=3, label='NEAR')
    ax1.plot(x_dist, medium_vals, 'y-', linewidth=3, label='MEDIUM')
    ax1.plot(x_dist, far_vals, 'g-', linewidth=3, label='FAR')
    
    # Fill areas under curves
    ax1.fill_between(x_dist, near_vals, alpha=0.3, color='red')
    ax1.fill_between(x_dist, medium_vals, alpha=0.3, color='yellow')
    ax1.fill_between(x_dist, far_vals, alpha=0.3, color='green')
    
    # Formatting
    ax1.set_xlabel('Distance (pixels)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Membership Degree', fontsize=12, fontweight='bold')
    ax1.set_title('INPUT: Distance Sensor', fontsize=14, fontweight='bold', pad=10)
    ax1.legend(loc='upper right', fontsize=11, framealpha=0.9)
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.set_xlim(0, 150)
    ax1.set_ylim(0, 1.1)
    
    # Add parameter annotations
    ax1.annotate(f'NEAR: [0, 0, 40]', xy=(20, 0.95), fontsize=9, 
                bbox=dict(boxstyle='round', facecolor='red', alpha=0.3))
    ax1.annotate(f'MEDIUM: [10, 40, 50]', xy=(30, 0.75), fontsize=9,
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.3))
    ax1.annotate(f'FAR: [40, 100, 1000]', xy=(80, 0.55), fontsize=9,
                bbox=dict(boxstyle='round', facecolor='green', alpha=0.3))
    
    # ==========================================
    # 2. GOAL ANGLE INPUT (Top Middle)
    # ==========================================
    ax2 = fig.add_subplot(gs[0, 1])
    x_angle = np.linspace(-3.14, 3.14, 500)
    
    # Calculate membership values
    cw_vals = [triangular_mf(x, ANGLE_CW) for x in x_angle]
    straight_vals = [triangular_mf(x, ANGLE_STRAIGHT) for x in x_angle]
    ccw_vals = [triangular_mf(x, ANGLE_CCW) for x in x_angle]
    
    # Plot membership functions
    ax2.plot(x_angle, cw_vals, 'b-', linewidth=3, label='CLOCKWISE')
    ax2.plot(x_angle, straight_vals, 'm-', linewidth=3, label='STRAIGHT')
    ax2.plot(x_angle, ccw_vals, 'c-', linewidth=3, label='COUNTER-CW')
    
    # Fill areas
    ax2.fill_between(x_angle, cw_vals, alpha=0.3, color='blue')
    ax2.fill_between(x_angle, straight_vals, alpha=0.3, color='magenta')
    ax2.fill_between(x_angle, ccw_vals, alpha=0.3, color='cyan')
    
    # Formatting
    ax2.set_xlabel('Angle (radians)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Membership Degree', fontsize=12, fontweight='bold')
    ax2.set_title('INPUT: Goal Angle', fontsize=14, fontweight='bold', pad=10)
    ax2.legend(loc='upper right', fontsize=11, framealpha=0.9)
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.set_xlim(-3.14, 3.14)
    ax2.set_ylim(0, 1.1)
    
    # Add parameter annotations
    ax2.annotate(f'CW: [-3.14, -1.0, -0.1]', xy=(-2.5, 0.95), fontsize=9,
                bbox=dict(boxstyle='round', facecolor='blue', alpha=0.3))
    ax2.annotate(f'STRAIGHT: [-0.3, 0.0, 0.3]', xy=(-0.2, 0.75), fontsize=9,
                bbox=dict(boxstyle='round', facecolor='magenta', alpha=0.3))
    ax2.annotate(f'CCW: [0.1, 1.0, 3.14]', xy=(1.5, 0.55), fontsize=9,
                bbox=dict(boxstyle='round', facecolor='cyan', alpha=0.3))
    
    # ==========================================
    # 3. STEERING OUTPUT - TRIANGULAR (Top Right)
    # ==========================================
    ax3 = fig.add_subplot(gs[0, 2])
    
    x_steer = np.linspace(-1.0, 1.0, 500)
    colors_steer = ['#d62728', '#ff7f0e', '#2ca02c', '#1f77b4', '#9467bd']
    
    # Create triangular membership functions for each steering output
    for i, (name, center) in enumerate(STEERING_OUTPUTS.items()):
        # Create triangular MF centered at the crisp value
        mf_params = create_output_triangular_mf(center, width=0.25)
        steer_vals = [triangular_mf(x, mf_params) for x in x_steer]
        
        ax3.plot(x_steer, steer_vals, linewidth=3, label=name, color=colors_steer[i])
        ax3.fill_between(x_steer, steer_vals, alpha=0.2, color=colors_steer[i])
    
    # Formatting
    ax3.set_xlabel('Steering Value (radians)', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Membership Degree', fontsize=12, fontweight='bold')
    ax3.set_title('OUTPUT: Steering Control', fontsize=14, fontweight='bold', pad=10)
    ax3.legend(loc='upper right', fontsize=10, framealpha=0.9)
    ax3.grid(True, alpha=0.3, linestyle='--')
    ax3.axvline(x=0, color='black', linewidth=2, linestyle='-', alpha=0.3)
    ax3.set_xlim(-1.0, 1.0)
    ax3.set_ylim(0, 1.1)
    
    # ==========================================
    # 4. VELOCITY OUTPUT - TRIANGULAR (Middle Left)
    # ==========================================
    ax4 = fig.add_subplot(gs[1, 0])
    
    x_vel = np.linspace(-1, 9, 500)
    colors_vel = ['#7f7f7f', '#bcbd22', '#17becf', '#2ca02c']
    
    # Create triangular membership functions for each velocity output
    for i, (name, center) in enumerate(VELOCITY_OUTPUTS.items()):
        # Create triangular MF centered at the crisp value
        if name == "Stop":
            width = 1.0
        elif name == "Slow":
            width = 1.2
        elif name == "Medium":
            width = 1.5
        else:  # Fast
            width = 1.5
        
        mf_params = create_output_triangular_mf(center, width=width)
        vel_vals = [triangular_mf(x, mf_params) for x in x_vel]
        
        ax4.plot(x_vel, vel_vals, linewidth=3, label=name, color=colors_vel[i])
        ax4.fill_between(x_vel, vel_vals, alpha=0.2, color=colors_vel[i])
    
    # Formatting
    ax4.set_xlabel('Velocity (pixels/step)', fontsize=12, fontweight='bold')
    ax4.set_ylabel('Membership Degree', fontsize=12, fontweight='bold')
    ax4.set_title('OUTPUT: Velocity Control', fontsize=14, fontweight='bold', pad=10)
    ax4.legend(loc='upper right', fontsize=10, framealpha=0.9)
    ax4.grid(True, alpha=0.3, linestyle='--')
    ax4.set_xlim(-1, 9)
    ax4.set_ylim(0, 1.1)
    
    # ==========================================
    # 5. SENSOR CONFIGURATION (Middle Middle)
    # ==========================================
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.axis('off')
    
    # Title
    ax5.text(0.5, 0.95, 'SENSOR CONFIGURATION', 
            ha='center', va='top', fontsize=14, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
    
    # Draw robot with sensors
    robot_x, robot_y = 0.5, 0.5
    robot_size = 0.08
    
    # Draw robot body (circle)
    robot = plt.Circle((robot_x, robot_y), robot_size, facecolor='blue', alpha=0.6, edgecolor='black', linewidth=2)
    ax5.add_patch(robot)
    
    # Draw 5 sensors
    sensor_angles = [0, 0.785, -0.785, 1.57, -1.57]  # Forward, FL, FR, Left, Right
    sensor_names = ['S0\n(Front)', 'S1\n(Front-L)', 'S2\n(Front-R)', 'S3\n(Left)', 'S4\n(Right)']
    sensor_colors = ['red', 'orange', 'orange', 'yellow', 'yellow']
    
    for i, (angle, name, color) in enumerate(zip(sensor_angles, sensor_names, sensor_colors)):
        # Sensor beam
        beam_length = 0.25
        end_x = robot_x + beam_length * np.cos(angle)
        end_y = robot_y + beam_length * np.sin(angle)
        ax5.arrow(robot_x, robot_y, (end_x - robot_x) * 0.9, (end_y - robot_y) * 0.9,
                 head_width=0.03, head_length=0.02, fc=color, ec='black', linewidth=1.5, alpha=0.7)
        
        # Sensor label
        label_x = robot_x + (beam_length + 0.1) * np.cos(angle)
        label_y = robot_y + (beam_length + 0.1) * np.sin(angle)
        ax5.text(label_x, label_y, name, ha='center', va='center', fontsize=9, fontweight='bold',
                bbox=dict(boxstyle='round', facecolor=color, alpha=0.4, edgecolor='black'))
    
    ax5.set_xlim(0, 1)
    ax5.set_ylim(0, 1)
    ax5.set_aspect('equal')
    
    # ==========================================
    # 6. FUZZY RULES EXPLANATION (Middle Right)
    # ==========================================
    ax6 = fig.add_subplot(gs[1, 2])
    ax6.axis('off')
    
    rules_text = """
FUZZY RULE BASE
═══════════════════════════════════════

EMERGENCY AVOIDANCE:
• IF Front sensor is NEAR
  THEN Turn Sharp (away from closer diagonal)
       Speed = SLOW
  WHY: Immediate obstacle - emergency turn

CORRIDOR NAVIGATION:
• IF Left is NEAR AND Right is NEAR
  THEN Steering = NEUTRAL, Speed = MEDIUM
  WHY: In narrow passage - go straight carefully

WALL FOLLOWING:
• IF Left is NEAR AND Right is FAR
  THEN Turn Gentle_CW, Speed = MEDIUM
• IF Right is NEAR AND Left is FAR
  THEN Turn Gentle_CCW, Speed = MEDIUM
  WHY: Balance between walls

DIAGONAL CORRECTION:
• IF Front-Left sensor is NEAR
  THEN Turn Gentle_CW, Speed = SLOW
• IF Front-Right sensor is NEAR
  THEN Turn Gentle_CCW, Speed = SLOW
  WHY: Avoid diagonal obstacles
"""
    
    ax6.text(0.05, 0.95, rules_text, ha='left', va='top', fontsize=9,
            family='monospace', verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8, edgecolor='black', linewidth=2))
    
    # ==========================================
    # 7. MORE RULES (Bottom Left)
    # ==========================================
    ax7 = fig.add_subplot(gs[2, 0])
    ax7.axis('off')
    
    more_rules = """
GOAL-ORIENTED NAVIGATION:
═══════════════════════════════════════

WHEN PATH IS CLEAR:
• Calculate: clear_path = min of (Front, 
  Front-Left, Front-Right) being FAR/MEDIUM

• IF clear_path AND goal angle is CCW
  THEN Turn Gentle_CCW
       Speed = FAST (if very clear) else MEDIUM
       
• IF clear_path AND goal angle is CW
  THEN Turn Gentle_CW
       Speed = FAST (if very clear) else MEDIUM
       
• IF clear_path AND goal angle is STRAIGHT
  THEN Steering = NEUTRAL
       Speed = FAST (if very clear) else MEDIUM

WHY: When safe, navigate toward goal
     Speed depends on path clarity
"""
    
    ax7.text(0.05, 0.95, more_rules, ha='left', va='top', fontsize=9,
            family='monospace',
            bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8, edgecolor='black', linewidth=2))
    
    # ==========================================
    # 8. DEFUZZIFICATION METHOD (Bottom Middle)
    # ==========================================
    ax8 = fig.add_subplot(gs[2, 1])
    ax8.axis('off')
    
    defuzz_text = """
DEFUZZIFICATION: CENTER OF GRAVITY
═══════════════════════════════════════

After all rules fire, we have activation
levels for each output:

STEERING:
final = Σ(activation × crisp_value) / Σ(activation)

Example:
  Sharp_CW (0.3) × (-0.8) = -0.24
  Gentle_CW (0.7) × (-0.3) = -0.21
  ─────────────────────────────
  Sum numerator = -0.45
  Sum denominator = 1.0
  RESULT: -0.45 radians (gentle clockwise)

VELOCITY:
Same method:
  Slow (0.5) × 2.0 = 1.0
  Medium (0.8) × 4.0 = 3.2
  ─────────────────────────────
  Sum numerator = 4.2
  Sum denominator = 1.3
  RESULT: 3.23 pixels/step
"""
    
    ax8.text(0.05, 0.95, defuzz_text, ha='left', va='top', fontsize=9,
            family='monospace',
            bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.8, edgecolor='black', linewidth=2))
    
    # ==========================================
    # 9. SYSTEM SUMMARY (Bottom Right)
    # ==========================================
    ax9 = fig.add_subplot(gs[2, 2])
    ax9.axis('off')
    
    summary_text = """
COMPLETE SYSTEM SUMMARY
═══════════════════════════════════════

INPUTS (Fuzzified):
  • 5 Distance Sensors (S0-S4)
    Each → {NEAR, MEDIUM, FAR}
  • 1 Goal Angle
    → {CLOCKWISE, STRAIGHT, COUNTER-CW}

INFERENCE ENGINE:
  • 11 Fuzzy Rules (shown left)
  • MAX aggregation (highest activation wins)
  • MIN for rule antecedents (AND operation)

OUTPUTS (Defuzzified):
  • Steering: -0.8 to +0.8 radians
    (negative = CW, positive = CCW)
  • Velocity: 0.0 to 7.0 pixels/step
    (0 = stop, 7 = fast)

HOW IT WORKS:
1. Read all 5 sensors
2. Fuzzify inputs (calculate memberships)
3. Fire all rules simultaneously
4. Aggregate results (max activation)
5. Defuzzify to crisp output
6. Apply to robot motion

CYCLE TIME: ~30ms per iteration
"""
    
    ax9.text(0.05, 0.95, summary_text, ha='left', va='top', fontsize=9,
            family='monospace',
            bbox=dict(boxstyle='round', facecolor='lavender', alpha=0.8, edgecolor='black', linewidth=2))
    
    # ==========================================
    # Save the figure
    # ==========================================
    output_filename = 'fuzzy_membership_functions.png'
    plt.savefig(output_filename, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"✓ Saved membership function visualization to: {os.path.abspath(output_filename)}")
    
    plt.show()


# ==========================================
# PRINT DETAILED EXPLANATION TO CONSOLE
# ==========================================
def print_detailed_explanation():
    """Print comprehensive explanation to console"""
    
    print("\n" + "="*80)
    print("FUZZY LOGIC CONTROL SYSTEM - DETAILED EXPLANATION")
    print("="*80)
    
    print("\n📊 INPUTS:")
    print("-" * 80)
    print("1. DISTANCE SENSORS (5 sensors):")
    print("   • Sensor 0 (Front): Measures distance directly ahead")
    print("   • Sensor 1 (Front-Left): 45° to the left")
    print("   • Sensor 2 (Front-Right): 45° to the right")
    print("   • Sensor 3 (Left): 90° to the left")
    print("   • Sensor 4 (Right): 90° to the right")
    print("\n   Each sensor has 3 fuzzy sets:")
    print(f"   • NEAR: {DIST_NEAR} pixels")
    print(f"   • MEDIUM: {DIST_MEDIUM} pixels")
    print(f"   • FAR: {DIST_FAR} pixels")
    
    print("\n2. GOAL ANGLE:")
    print("   • Angle difference between robot heading and goal direction")
    print("   • 3 fuzzy sets:")
    print(f"   • CLOCKWISE: {ANGLE_CW} radians")
    print(f"   • STRAIGHT: {ANGLE_STRAIGHT} radians")
    print(f"   • COUNTER-CLOCKWISE: {ANGLE_CCW} radians")
    
    print("\n📤 OUTPUTS:")
    print("-" * 80)
    print("1. STEERING CONTROL (Triangular MFs):")
    for name, value in STEERING_OUTPUTS.items():
        direction = "↶ Left" if value > 0 else "↷ Right" if value < 0 else "→ Straight"
        print(f"   • {name}: {value:+.2f} rad {direction}")
    
    print("\n2. VELOCITY CONTROL (Triangular MFs):")
    for name, value in VELOCITY_OUTPUTS.items():
        print(f"   • {name}: {value:.1f} pixels/step")
    
    print("\n📋 FUZZY RULES (11 TOTAL):")
    print("-" * 80)
    
    rules = [
        ("RULE 1-2", "Emergency Avoidance",
         "IF Front sensor is NEAR\nTHEN Turn sharp away from closer diagonal obstacle\n     Speed = SLOW",
         "Immediate obstacle detected - must turn quickly to avoid collision"),
        
        ("RULE 3", "Narrow Corridor",
         "IF Left is NEAR AND Right is NEAR\nTHEN Steering = NEUTRAL, Speed = MEDIUM",
         "Robot is in a narrow passage - go straight carefully"),
        
        ("RULE 4-5", "Wall Following",
         "IF Left is NEAR AND Right is FAR → Turn Gentle_CW\nIF Right is NEAR AND Left is FAR → Turn Gentle_CCW",
         "Balance the robot between walls to stay centered"),
        
        ("RULE 6-7", "Diagonal Obstacle Avoidance",
         "IF Front-Left is NEAR → Turn Gentle_CW\nIF Front-Right is NEAR → Turn Gentle_CCW",
         "Avoid obstacles detected at diagonal angles"),
        
        ("RULE 8-9", "Corner Navigation",
         "IF Right+Front-Right+Front is obstacle → Turn CCW\nIF Left+Front-Left+Front is obstacle → Turn CW",
         "Detect corner situations and navigate around them"),
        
        ("RULE 10-12", "Goal-Oriented Navigation",
         "IF path is clear AND goal angle direction\nTHEN Turn toward goal, Speed = FAST or MEDIUM",
         "When safe, navigate directly toward the target"),
    ]
    
    for rule_num, title, condition, reason in rules:
        print(f"\n{rule_num}: {title}")
        print(f"   Condition: {condition}")
        print(f"   Why: {reason}")
    
    print("\n⚙️ HOW THE SYSTEM WORKS:")
    print("-" * 80)
    print("1. FUZZIFICATION:")
    print("   • Read all 5 sensor values (crisp inputs)")
    print("   • Calculate membership degrees for each fuzzy set")
    print("   • Example: If distance = 30px")
    print("      - NEAR membership = 0.25")
    print("      - MEDIUM membership = 0.75")
    print("      - FAR membership = 0.0")
    
    print("\n2. RULE EVALUATION:")
    print("   • All 11 rules are evaluated in parallel")
    print("   • MIN operator for AND conditions")
    print("   • MAX operator for aggregation")
    print("   • Example: IF sensor[0] is NEAR (0.8) AND sensor[1] is NEAR (0.6)")
    print("      - Rule strength = MIN(0.8, 0.6) = 0.6")
    
    print("\n3. DEFUZZIFICATION (Center of Gravity):")
    print("   • Combine all activated outputs")
    print("   • Formula: output = Σ(activation × value) / Σ(activation)")
    print("   • Produces crisp steering angle and velocity")
    
    print("\n4. EXECUTION:")
    print("   • Apply steering to robot heading")
    print("   • Move robot by velocity amount")
    print("   • Repeat cycle every ~30ms")
    
    print("\n" + "="*80)
    print("END OF EXPLANATION")
    print("="*80 + "\n")


# ==========================================
# MAIN EXECUTION
# ==========================================
if __name__ == "__main__":
    print("\n🎨 Generating Fuzzy Logic Membership Function Visualization...\n")
    
    # Print detailed explanation to console
    print_detailed_explanation()
    
    # Create and display visualization
    create_fuzzy_visualization()
    
    print("\n✅ Visualization complete!")
    print("📊 Check the generated image in your current directory")
    print("📝 All explanations printed above\n")
