import math

class PT1Filter:
    def __init__(self, cutoff_freq, dt):
        self.state = 0.0
        rc = 1.0 / (2.0 * math.pi * cutoff_freq)
        self.k = dt / (rc + dt)

    def apply(self, input_val):
        self.state = self.state + self.k * (input_val - self.state)
        return self.state

class BetaflightPID:
    def __init__(self, dt):
        self.dt = dt
        self.iterm_limit = 400.0
        self.dterm_limit = 400.0
        self.pid_sum_limit = 500.0
        
        # P, I, D, F, Sum
        self.data = [{'P': 0.0, 'I': 0.0, 'D': 0.0, 'F': 0.0, 'Sum': 0.0} for _ in range(3)]
        self.previous_gyro = [0.0, 0.0, 0.0]
        self.previous_setpoint = [0.0, 0.0, 0.0]
        
        # D-term Lowpass filters (Aggressive Cascade of 50Hz and 80Hz for noisy frames)
        self.dterm_lpf1 = [PT1Filter(50.0, dt) for _ in range(3)]
        self.dterm_lpf2 = [PT1Filter(80.0, dt) for _ in range(3)]
        
        # Default tuning gains (P, I, D, F)
        self.gains = [
            {'Kp': 45.0, 'Ki': 50.0, 'Kd': 25.0, 'Kf': 60.0}, # ROLL
            {'Kp': 47.0, 'Ki': 54.0, 'Kd': 27.0, 'Kf': 65.0}, # PITCH
            {'Kp': 45.0, 'Ki': 50.0, 'Kd': 0.0,  'Kf': 60.0}  # YAW
        ]

        self.PTERM_SCALE = 0.032029
        self.ITERM_SCALE = 0.244381
        self.DTERM_SCALE = 0.000529
        self.FEEDFORWARD_SCALE = 0.00244

    def constrain(self, val, min_val, max_val):
        return max(min_val, min(max_val, val))

    def apply(self, axis, gyro_rate, setpoint):
        gains = self.gains[axis]
        data = self.data[axis]
        dt = self.dt

        # 1. Error calculation
        error = setpoint - gyro_rate

        # 2. P-Term
        data['P'] = error * gains['Kp'] * self.PTERM_SCALE

        # 3. I-Term (Anti-windup limit)
        data['I'] += error * gains['Ki'] * self.ITERM_SCALE * dt
        data['I'] = self.constrain(data['I'], -self.iterm_limit, self.iterm_limit)

        # 4. D-Term (Measurement based to prevent setpoint kick)
        delta_gyro = gyro_rate - self.previous_gyro[axis]
        self.previous_gyro[axis] = gyro_rate
        
        d_term_raw = -(delta_gyro / dt) * gains['Kd'] * self.DTERM_SCALE
        
        # Apply cascade PT1
        d_term_raw = self.dterm_lpf1[axis].apply(d_term_raw)
        d_term_raw = self.dterm_lpf2[axis].apply(d_term_raw)
        data['D'] = self.constrain(d_term_raw, -self.dterm_limit, self.dterm_limit)

        # 5. Feedforward
        delta_setpoint = setpoint - self.previous_setpoint[axis]
        self.previous_setpoint[axis] = setpoint
        data['F'] = (delta_setpoint / dt) * gains['Kf'] * self.FEEDFORWARD_SCALE

        # 6. Final PID Payload
        data['Sum'] = data['P'] + data['I'] + data['D'] + data['F']
        data['Sum'] = self.constrain(data['Sum'], -self.pid_sum_limit, self.pid_sum_limit)

def mix_quad_x(pid, throttle):
    # Returns [m1, m2, m3, m4] corresponding to 0-1000 bounds
    pitch_sum = pid.data[1]['Sum'] # PITCH
    roll_sum = pid.data[0]['Sum']  # ROLL
    yaw_sum = pid.data[2]['Sum']   # YAW

    m1 = throttle - pitch_sum - roll_sum + yaw_sum
    m2 = throttle + pitch_sum - roll_sum - yaw_sum
    m3 = throttle - pitch_sum + roll_sum - yaw_sum
    m4 = throttle + pitch_sum + roll_sum + yaw_sum

    return [
        max(0.0, min(1000.0, m1)),
        max(0.0, min(1000.0, m2)),
        max(0.0, min(1000.0, m3)),
        max(0.0, min(1000.0, m4))
    ]
