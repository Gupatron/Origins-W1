import time
import json
import numpy as np

config = json.load(open('config.json'))

# NEW: Load max bypass duty from config
max_bypass_duty = config.get('max_bypass_duty_percent', 20) / 100.0

class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, desired, current):
        error = desired - current
        self.integral += error * self.dt
        der = (error - self.prev_error) / self.dt if self.dt > 0 else 0
        u = self.kp * error + self.ki * self.integral + self.kd * der
        self.prev_error = error
        return u, error

def pid_thread(buffer, stop_event):
    dt = 1 / config['pid_frequency']
    pid = PID(buffer.kp, buffer.ki, buffer.kd, dt)
    while not stop_event.is_set():
        with buffer.lock:
            if buffer.running:
                pid.kp = buffer.kp
                pid.ki = buffer.ki
                pid.kd = buffer.kd
                if buffer.bypass_pid:  # Bypass mode
                    u = buffer.direct_duty
                    u = np.clip(u, -max_bypass_duty, max_bypass_duty)  # NEW: Clamp for safety
                    buffer.u_duty = u
                    buffer.error = 0.0
                    # Reset PID state to avoid jumps when switching back to PID mode
                    pid.integral = 0.0
                    pid.prev_error = 0.0
                else:  # Normal PID mode
                    current = buffer.omega_filteredrpm if buffer.use_filtered_on else buffer.omega_rpm
                    u, error = pid.compute(buffer.omega_d, current)
                    if abs(error) > config['error_threshold']:
                        u = 0.0
                    u = np.clip(u, -1.0, 1.0)
                    buffer.u_duty = u
                    buffer.error = error
            else:
                pid.integral = 0.0
                pid.prev_error = 0.0
                buffer.u_duty = 0.0
                buffer.error = 0.0
        time.sleep(dt)
