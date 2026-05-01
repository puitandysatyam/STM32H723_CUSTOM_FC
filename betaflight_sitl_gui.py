import time
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from betaflight_math import BetaflightPID, mix_quad_x

# SITL Frequency
SIM_HZ = 50.0
DT = 1.0 / SIM_HZ

pid = BetaflightPID(DT)

# Physics Engine State
state = {
    't': 0.0,
    'roll_rate': 0.0, 'pitch_rate': 0.0, 'yaw_rate': 0.0,
    'm': [0, 0, 0, 0],
    'setpoint_roll': 0.0,
    'setpoint_pitch': 0.0
}

# Telemetry History for Graphing
history_t = []
history_roll = []
history_sp_roll = []

# Create the Dashboard GUI
plt.style.use('dark_background')
fig, (ax_drone, ax_graph) = plt.subplots(1, 2, figsize=(14, 6))
fig.canvas.manager.set_window_title('Betaflight Custom SITL Dashboard')

def update_physics():
    """ Simulates raw aerodynamic momentum and torque based on motor RPM """
    m = state['m']
    
    # Motor Geometry Torque Matrix
    # M1=RR, M2=FR, M3=RL, M4=FL
    torque_roll = ((m[3] + m[2]) - (m[1] + m[0])) * 0.07   # Left motors vs Right motors
    torque_pitch = ((m[2] + m[0]) - (m[3] + m[1])) * 0.07  # Rear vs Front
    
    # Aerodynamic drag dampening
    drag_roll = state['roll_rate'] * 0.15
    drag_pitch = state['pitch_rate'] * 0.15
    
    # Integrate Angular Acceleration into Gyro Rates
    state['roll_rate'] += (torque_roll - drag_roll) * DT
    state['pitch_rate'] += (torque_pitch - drag_pitch) * DT
    
    # Inject external wind perturbation/turbulence every few seconds
    state['roll_rate'] += math.sin(state['t'] * 2.5) * 35.0

def run_pid_loop():
    """ 100% Native Betaflight PID Logic """
    # Simulate pilot moving the RC Sticks in a sine wave to test PID tracking
    state['setpoint_roll'] = math.sin(state['t'] * 1.2) * 120.0 
    
    throttle = 200.0
    
    # Execute C-ported Betaflight calculations
    pid.apply(0, state['roll_rate'], state['setpoint_roll'])
    pid.apply(1, state['pitch_rate'], state['setpoint_pitch'])
    pid.apply(2, state['yaw_rate'], 0.0)
    
    # Final QuadX Mixing
    state['m'] = mix_quad_x(pid, throttle)

def animate(frame):
    state['t'] += DT
    
    # 1. Tick Physics Engine
    update_physics()
    
    # 2. Tick Betaflight Controller
    run_pid_loop()
    
    # 3. Render Drone Visuals
    ax_drone.clear()
    ax_drone.set_xlim(-2, 2)
    ax_drone.set_ylim(-2, 2)
    ax_drone.set_title("Quadcopter Aerodynamic Motor Strain", fontsize=14, pad=20)
    ax_drone.axis('off')
    
    # Draw Chassis Arms
    ax_drone.plot([-1, 1], [1, -1], color='#555555', lw=5) # FL to RR
    ax_drone.plot([-1, 1], [-1, 1], color='#555555', lw=5) # RL to FR
    
    # Draw Rendered Motors mapped directly to calculated thrust arrays
    m = state['m']
    # Coordinates: FL(-1, 1), FR(1, 1), RL(-1, -1), RR(1, -1)
    # Betaflight index: 0=RR, 1=FR, 2=RL, 3=FL
    coords = [(1, -1), (1, 1), (-1, -1), (-1, 1)]
    colors = ['#ff4444', '#ff4444', '#44aaff', '#44aaff'] # Red rear, Blue front
    labels = ['M1 (RR)', 'M2 (FR)', 'M3 (RL)', 'M4 (FL)']
    
    for i in range(4):
        size = 200 + (m[i] * 3) # Graphic motor circle expands with PWM thrust
        ax_drone.scatter(coords[i][0], coords[i][1], s=size, c=colors[i], alpha=0.8, edgecolors='white', linewidths=2)
        ax_drone.text(coords[i][0], coords[i][1], f"{labels[i]}\n{int(m[i])}", ha='center', va='center', color='white', fontweight='bold')
        
    title = f"Wind Perturbation: {math.sin(state['t'] * 2.5) * 35.0:+.1f} deg/s"
    ax_drone.text(0, -1.8, title, ha='center', color='yellow')

    # 4. Render Telemetry Graph
    history_t.append(state['t'])
    history_roll.append(state['roll_rate'])
    history_sp_roll.append(state['setpoint_roll'])
    
    if len(history_t) > 150:
        history_t.pop(0)
        history_roll.pop(0)
        history_sp_roll.pop(0)
        
    ax_graph.clear()
    ax_graph.set_title("Betaflight PID Tracking (Roll Axis)", fontsize=14)
    ax_graph.plot(history_t, history_sp_roll, '#ffaa00', label="Setpoint (RC Target)", lw=2)
    ax_graph.plot(history_t, history_roll, '#00ffff', label="Gyro (Actual Model)", lw=2)
    ax_graph.legend(loc="upper right", facecolor='#222222', edgecolor='none')
    ax_graph.set_ylim(-250, 250)
    ax_graph.grid(True, linestyle=':', alpha=0.4)
    ax_graph.set_xlabel("Time (s)")
    ax_graph.set_ylabel("Degrees / Sec")

print("Launching UI...")
ani = animation.FuncAnimation(fig, animate, interval=int(DT*1000), cache_frame_data=False)
plt.show()
