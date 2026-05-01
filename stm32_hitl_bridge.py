import socket
import struct
import pygame
import time
import math

# --- CONFIGURATION ---
ESP32_IP = "10.175.52.29" 
ESP32_PORT = 8888 
LISTEN_PORT = 9999

# --- PHYSICS CONSTANTS (SI Units: kg, m, s, N) ---
MASS = 1.0           
GRAVITY = 9.81       
DRAG_COEFF_LINEAR = 0.15
DRAG_COEFF_ANGULAR = 0.5
INERTIA = 0.045      
MAX_THRUST_TOTAL = 80.0 
MAX_THRUST_MOTOR = MAX_THRUST_TOTAL / 4.0

# --- WORLD STATE ---
pos = [0.0, 0.0, 0.0]        # X (Side+), Y (Up+), Z (Forward+)
vel = [0.0, 0.0, 0.0]        
attitude = [0.0, 0.0, 0.0]   # Roll, Pitch, Yaw (Degrees)
gyro     = [0.0, 0.0, 0.0]   # Angular Velocity d/s
motors   = [1000, 1000, 1000, 1000] # STM32 PWM Output (M1, M2, M3, M4)
sticks   = [1500, 1500, 1500, 1000] # R, P, Y, T
rx_packets = 0 
is_armed = False
mode_offline = False 

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)
try: sock.bind(('', LISTEN_PORT))
except: pass

pygame.init()
WIDTH, HEIGHT = 1280, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("BETAFLIGHT-HITL 10.2: STABILIZED PRO FLIGHT")
font_sm = pygame.font.SysFont("Consolas", 16, bold=True)
font_md = pygame.font.SysFont("Consolas", 20, bold=True)
font_lg = pygame.font.SysFont("Consolas", 30, bold=True)
clock = pygame.time.Clock()

def project(x, y, z):
    cx, cy, cz = pos[0], pos[1] + 3.0, pos[2] - 12.0
    rx, ry, rz = x - cx, y - cy, z - cz
    if rz < 0.1: return None 
    scale = 800 / rz
    return (int(WIDTH/2 + rx * scale), int(HEIGHT/2 - ry * scale))

def transform_to_world(lx, ly, lz, roll, pitch, yaw):
    r, p, ya = math.radians(roll), math.radians(pitch), math.radians(yaw)
    x1 = lx * math.cos(r) + ly * math.sin(r); y1 = -lx * math.sin(r) + ly * math.cos(r); z1 = lz
    x2 = x1; y2 = y1 * math.cos(p) - z1 * math.sin(p); z2 = y1 * math.sin(p) + z1 * math.cos(p)
    wx = x2 * math.cos(ya) + z2 * math.sin(ya); wy = y2; wz = -x2 * math.sin(ya) + z2 * math.cos(ya)
    return [wx, wy, wz]

def draw_environment():
    for i in range(HEIGHT // 2):
        lv = i / (HEIGHT//2); c = (20 + int(lv*30), 40 + int(lv*40), 100 + int(lv*50))
        pygame.draw.line(screen, c, (0, i), (WIDTH, i))
    for i in range(HEIGHT // 2, HEIGHT):
        lv = (i - HEIGHT // 2) / (HEIGHT // 2); c = (20 + int(lv*40), 60 + int(lv*50), 20 + int(lv*30))
        pygame.draw.line(screen, c, (0, i), (WIDTH, i))
    pygame.draw.line(screen, (0, 255, 255, 120), (0, HEIGHT//2), (WIDTH, HEIGHT//2), 3)

def draw_grid():
    sz = 10; lx, lz = (pos[0] // sz) * sz, (pos[2] // sz) * sz
    for i in range(-25, 26):
        c_val = max(30, 110 - abs(i)*4)
        p1 = project(lx + i*sz, 0, lz-200); p2 = project(lx + i*sz, 0, lz+200)
        if p1 and p2: pygame.draw.line(screen, (0, c_val, 0), p1, p2, 1)
        p1 = project(lx-200, 0, lz + i*sz); p2 = project(lx+200, 0, lz + i*sz)
        if p1 and p2: pygame.draw.line(screen, (0, c_val, 0), p1, p2, 1)

def draw_drone():
    alen = 2.0 
    pts = [(-alen,0,alen), (alen,0,alen), (alen,0,-alen), (-alen,0,-alen)]
    rot = [transform_to_world(pt[0], pt[1], pt[2], attitude[0], attitude[1], attitude[2]) for pt in pts]
    proj = [project(pos[0]+r[0], pos[1]+r[1], pos[2]+r[2]) for r in rot]
    center_p = project(pos[0], pos[1], pos[2])
    if not center_p: return
    if pos[1] < 20:
        s_sz = max(5, 60 - int(pos[1]*3)); s_p = project(pos[0], 0, pos[2])
        if s_p: pygame.draw.circle(screen, (0, 0, 0, 150), s_p, s_sz)
    colors = [(255, 50, 50), (255, 50, 50), (220, 220, 220), (220, 220, 220)]
    for i, p in enumerate(proj):
        if p:
            pygame.draw.line(screen, colors[i], center_p, p, 14)
            pygame.draw.circle(screen, (30, 30, 30), p, 15)
            m_id = {0:3, 1:1, 2:0, 3:2}[i]
            thr = (motors[m_id] - 1000) / 1000.0
            if is_armed and thr > 0.02:
                p_rad = int(28 + thr * 14); p_color = (255, 255, 255, int(40 + thr * 160))
                if (time.time() * 60) % 2 > 1:
                    temp_s = pygame.Surface((p_rad*2, p_rad*2), pygame.SRCALPHA)
                    pygame.draw.circle(temp_s, p_color, (p_rad, p_rad), p_rad, 2); screen.blit(temp_s, (p[0]-p_rad, p[1]-p_rad))
    h_f = transform_to_world(0, 0.6, 2.5, attitude[0], attitude[1], attitude[2])
    h_p = project(pos[0]+h_f[0], pos[1]+h_f[1], pos[2]+h_f[2])
    if h_p:
        pygame.draw.circle(screen, (255, 255, 0), h_p, 12); pygame.draw.line(screen, (255, 255, 0), center_p, h_p, 5)

def draw_hud(acc_y, r_trq, p_trq):
    hud_bg = pygame.Surface((580, 400), pygame.SRCALPHA)
    pygame.draw.rect(hud_bg, (10, 20, 35, 220), (0,0,580,400), border_radius=15)
    pygame.draw.rect(hud_bg, (0, 255, 255, 120), (0,0,580,400), 2, border_radius=15)
    screen.blit(hud_bg, (30, 30))
    screen.blit(font_lg.render(">>> MISSION CONTROL v10.2 <<<", True, (0, 255, 255)), (50, 50))
    pygame.draw.line(screen, (0, 200, 255), (50, 90), (560, 90), 2)

    lines = [
        ("STATUS    :", "ARMED" if is_armed else "DISARMED", (0, 255, 0) if is_armed else (255, 50, 50)),
        ("PHYSICS   :", "[GAME]" if mode_offline else "[HITL]", (255, 255, 255)),
        ("ALTITUDE  :", f"{pos[1]:.2f} m", (255, 255, 0)),
        ("VELOCITY  :", f"{vel[0]:.1f}, {vel[1]:.1f}, {vel[2]:.1f} m/s", (200, 200, 255)),
        ("ATTITUDE  :", f"R:{attitude[0]:.1f} P:{attitude[1]:.1f} Y:{attitude[2]:.1f}", (255, 255, 255)),
        ("LIFT      :", f"{acc_y/9.81 + 1.0:.2f} G", (150, 255, 150)),
        ("PKTS RX   :", f"{rx_packets}", (200, 200, 200)),
    ]
    for i, (l, v, c) in enumerate(lines):
        screen.blit(font_md.render(l, True, (180, 180, 180)), (60, 110 + i * 26))
        screen.blit(font_md.render(v, True, c), (220, 110 + i * 26))

    m_x, m_y = 60, 380; labels = ["RR", "FR", "RL", "FL"]
    for i in range(4):
        val = (motors[i] - 1000) / 1000.0; bar_h = int(val * 80)
        pygame.draw.rect(screen, (50, 50, 50), (m_x + i*60, m_y - 80, 40, 80))
        pygame.draw.rect(screen, (0, 255, 100), (m_x + i*60, m_y - bar_h, 40, bar_h))
        screen.blit(font_sm.render(f"M{i+1}:{labels[i]}", True, (255,255,255)), (m_x + i*60 - 5, m_y + 5))

    hx, hy = WIDTH - 180, 180; pygame.draw.circle(screen, (20, 30, 45, 180), (hx, hy), 100)
    pygame.draw.circle(screen, (0, 255, 255, 100), (hx, hy), 100, 3)
    ra = math.radians(attitude[0]); po = int(attitude[1] * 1.8)
    p1 = (hx - 80*math.cos(ra), hy - 80*math.sin(ra) + po); p2 = (hx + 80*math.cos(ra), hy + 80*math.sin(ra) + po)
    pygame.draw.line(screen, (0, 255, 255), p1, p2, 5); pygame.draw.circle(screen, (255, 255, 0), (hx, hy), 6)
    screen.blit(font_sm.render("'R' RESET | 'A' ARM | 'W' THR", True, (255, 255, 255, 120)), (WIDTH//2 - 140, HEIGHT - 50))

def calc_thrust(p): 
    return max(0.0, (p - 1000.0) / 1000.0 * MAX_THRUST_MOTOR)

running = True
while running:
    dt = 1.0 / 200.0
    try:
        while True:
            data, addr = sock.recvfrom(64)
            if data[0] == 0xFF and len(data) >= 19:
                rx_packets += 1; payload = struct.unpack(">HHHHH", data[2:12]); motors = list(payload[0:4])
    except: pass

    keys = pygame.key.get_pressed()
    sticks[3] = 1000 + (1000 if keys[pygame.K_w] else 0)
    sticks[0] = 1500 + ((1 if keys[pygame.K_RIGHT] else 0) - (1 if keys[pygame.K_LEFT] else 0)) * 500
    sticks[1] = 1500 + ((1 if keys[pygame.K_UP] else 0) - (1 if keys[pygame.K_DOWN] else 0)) * 500

    total_thrust, r_trq, p_trq = 0.0, 0, 0
    if mode_offline:
        total_thrust = calc_thrust(sticks[3]) * 4.0 if is_armed else 0.0
        gyro[0] = (sticks[0] - 1500) * 0.4; gyro[1] = (sticks[1] - 1500) * 0.4
    else:
        if rx_packets > 0:
            m = [calc_thrust(v) for v in motors]; total_thrust = sum(m)
            r_trq = (m[2] + m[3]) - (m[0] + m[1]); p_trq = (m[0] + m[2]) - (m[1] + m[3])
            gyro[0] += (r_trq / INERTIA) * dt * 57.3; gyro[1] += (p_trq / INERTIA) * dt * 57.3
        else: gyro[0], gyro[1], total_thrust = 0, 0, 0

    gyro[0] *= (1.0 - DRAG_COEFF_ANGULAR * dt * 10); gyro[1] *= (1.0 - DRAG_COEFF_ANGULAR * dt * 10)
    attitude[0] += gyro[0] * dt; attitude[1] += gyro[1] * dt
    rad_p, rad_r = math.radians(attitude[1]), math.radians(attitude[0])
    
    vertical_thrust = total_thrust * math.cos(rad_p) * math.cos(rad_r)
    acc_y = (vertical_thrust / MASS) - GRAVITY
    vel[1] += acc_y * dt; pos[1] = max(0, pos[1] + vel[1] * dt)
    if pos[1] == 0: vel[1] = 0

    acc_x = (total_thrust * math.sin(rad_r) / MASS); acc_z = -(total_thrust * math.sin(rad_p) / MASS)
    vel[0] = (vel[0] + acc_x * dt) * (1.0 - DRAG_COEFF_LINEAR * dt); vel[2] = (vel[2] + acc_z * dt) * (1.0 - DRAG_COEFF_LINEAR * dt)
    pos[0] += vel[0] * dt; pos[2] += vel[2] * dt

    aux1 = 2000 if is_armed else 1000
    p_c = struct.pack(">hhhhhh", int(gyro[0]), int(gyro[1]), int(gyro[2]), 0, 0, 1000)
    p_a = struct.pack(">HHHHHH", int(sticks[3]), int(sticks[0]), int(sticks[1]), int(sticks[2]), aux1, 1000)
    cs_c = 0; cs_a = 0
    for b in p_c: cs_c ^= b
    for b in p_a: cs_a ^= b
    sock.sendto(b'\xAA\xCC' + p_c + bytes([cs_c]), (ESP32_IP, ESP32_PORT))
    sock.sendto(b'\xAA\xAA' + p_a + bytes([cs_a]), (ESP32_IP, ESP32_PORT))

    screen.fill((0,0,0)); draw_environment(); draw_grid(); draw_drone(); draw_hud(acc_y, r_trq, p_trq)
    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT: running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_TAB: mode_offline = not mode_offline
            if event.key == pygame.K_a: is_armed = not is_armed
            if event.key == pygame.K_r: pos, vel, gyro, attitude = [0,0,0], [0,0,0], [0,0,0], [0,0,0]
    clock.tick(200)

pygame.quit()
