import socket
import struct
import time
import errno
import pygame
from pymavlink import mavutil

# --- CONFIGURATION ---
ESP32_IP = "10.175.52.29"   
ESP32_PORT = 8888
LISTEN_PORT = 9999          

JMAVSIM_IP = "172.20.104.34"      
JMAVSIM_PORT = 14560

# --- PYGAME FOR KEYBOARD ---
pygame.init()
screen = pygame.display.set_mode((300, 100))
pygame.display.set_caption("BRIDGE FOCUS: CLICK HERE")
font = pygame.font.SysFont("Arial", 14)

# --- MAVLINK SETUP ---
print(f"[{time.strftime('%H:%M:%S')}] Connecting to jMAVSim at {JMAVSIM_IP}:{JMAVSIM_PORT}...")
mav = mavutil.mavlink_connection(f'udpout:{JMAVSIM_IP}:{JMAVSIM_PORT}', source_system=1, source_component=1)

# Status / Telemetry
last_heartbeat = 0
last_sticks = 0
hb_count = 0
rx_pkts = 0
is_armed = False
live_p, live_r, live_y = True, True, True
motors = [1000, 1000, 1000, 1000]
gx, gy, gz = 0, 0, 0
thr, rol, pit, yaw = 1000, 1500, 1500, 1500

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)
try: sock.bind(('', LISTEN_PORT))
except Exception as e: print(f"UDP Bind Error: {e}")

def send_heartbeat():
    global hb_count
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED | (mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if is_armed else 0),
        0, 
        mavutil.mavlink.MAV_STATE_ACTIVE
    )
    hb_count += 1
    if hb_count % 5 == 0:
        print(f"[{time.strftime('%H:%M:%S')}] Sent Heartbeat #{hb_count} (Armed: {is_armed})")

def send_sticks():
    # Packet: 0xAA 0xAA [T, R, P, Y, Aux1, Aux2] [CS]
    aux1 = 2000 if is_armed else 1000
    p_a = struct.pack(">HHHHHH", int(thr), int(rol), int(pit), int(yaw), aux1, 1000)
    cs = 0
    for b in p_a: cs ^= b
    sock.sendto(b'\xAA\xAA' + p_a + bytes([cs]), (ESP32_IP, ESP32_PORT))

print(">>> BRIDGE STARTED: MAVLink <=> Custom 0xFF <<<")
print("Controls: W/S=Throttle, Arrows=Roll/Pitch, A=Arm/Disarm, R=Reset Sim")

running = True
while running:
    now = time.time()
    
    # Heartbeat
    if now - last_heartbeat > 1.0:
        send_heartbeat()
        last_heartbeat = now

    # Stick Logic (20Hz loop for sticks is enough for keyboard)
    if now - last_sticks > 0.05:
        send_sticks()
        last_sticks = now

    # 1. KEYBOARD HANDLING
    for event in pygame.event.get():
        if event.type == pygame.QUIT: running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_a: is_armed = not is_armed
            if event.key == pygame.K_r: # MAVLink Command to reset simulation
                mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
            # Axis Toggles for Debugging
            if event.key == pygame.K_p: live_p = not live_p # Toggle Pitch
            if event.key == pygame.K_o: live_r = not live_r # Toggle Roll
            if event.key == pygame.K_i: live_y = not live_y # Toggle Yaw
        
    keys = pygame.key.get_pressed()
    thr = 1000 + (1000 if keys[pygame.K_w] else 0) if is_armed else 1000
    rol = 1500 + ((1 if keys[pygame.K_RIGHT] else 0) - (1 if keys[pygame.K_LEFT] else 0)) * 500
    pit = 1500 + ((1 if keys[pygame.K_UP] else 0) - (1 if keys[pygame.K_DOWN] else 0)) * 500
    yaw = 1500 + ((1 if keys[pygame.K_l] else 0) - (1 if keys[pygame.K_j] else 0)) * 500 # J=Left, L=Right

    # UI for Focus Window
    # ... (skipping some UI code for brevity)
    screen.fill((20, 20, 25))
    screen.blit(font.render(f"STICKS | T:{thr} R:{rol} P:{pit} Y:{yaw}", True, (0, 200, 255)), (10, 5))
    screen.blit(font.render("MOTORS | " + " ".join([f"M{i+1}:{v}" for i, v in enumerate(motors)]), True, (0, 255, 100)), (10, 25))
    screen.blit(font.render(f"GYRO   | X:{int(gx)} Y:{int(gy)} Z:{int(gz)}", True, (255, 200, 0)), (10, 45))
    screen.blit(font.render(f"LIVE   | P:{live_p} R:{live_r} Y:{live_y}", True, (255, 0, 255)), (10, 65))
    screen.blit(font.render("'R' RESET | 'P/O/I' Toggle | 'J/L' Yaw", True, (150, 150, 150)), (10, 85))
    pygame.display.flip()

    # 2. RECEIVE FROM STM32
    # ... (motor receive logic)
    try:
        data, addr = sock.recvfrom(128)
        if len(data) >= 19 and data[0] == 0xFF:
            rx_pkts += 1
            motors = list(struct.unpack(">HHHH", data[2:10]))
            px4_controls = [0.0] * 16
            px4_controls[0] = (motors[1] - 1000.0) / 1000.0  # FR (M2)
            px4_controls[1] = (motors[2] - 1000.0) / 1000.0  # RL (M3)
            px4_controls[2] = (motors[3] - 1000.0) / 1000.0  # FL (M4)
            px4_controls[3] = (motors[0] - 1000.0) / 1000.0  # RR (M1)
            mode = 160 if is_armed else 32 
            mav.mav.hil_actuator_controls_send(int(now * 1000000), px4_controls, mode, 0)
    except (BlockingIOError, socket.error): pass

    # 3. RECEIVE FROM jMAVSim
    try:
        msg = mav.recv_msg()
        if msg and msg.get_type() == 'HIL_SENSOR':
            import math
            if all(math.isfinite(v) for v in [msg.xgyro, msg.ygyro, msg.zgyro]):
                gx = -msg.xgyro * 57.3  if live_r else 0
                gy = -msg.ygyro * 57.3  if live_p else 0
                gz = -msg.zgyro * 57.3  if live_y else 0
                
                def cl(v): return max(-32768, min(32767, int(v)))
                sensor_payload = struct.pack(">hhhhhh", cl(gx), cl(gy), cl(gz), 0, 0, 1000)
                cs = 0
                for b in sensor_payload: cs ^= b
                sock.sendto(b'\xAA\xCC' + sensor_payload + bytes([cs]), (ESP32_IP, ESP32_PORT))
    except socket.error as e:
        if e.errno != errno.ECONNRESET: pass
    except Exception as e:
        print(f"Bridge Telemetry Error: {e}")

    time.sleep(0.001)

pygame.quit()
