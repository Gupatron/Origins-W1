import time
import json
import numpy as np

config = json.load(open('config.json'))

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
                if buffer.bypass_pid:  # NEW: Check for bypass mode
                    buffer.u_duty = buffer.direct_duty
                    buffer.error = 0.0
                    # NEW: Reset PID state to avoid jumps when switching back to PID mode
                    pid.integral = 0.0
                    pid.prev_error = 0.0
                else:  # Normal PID mode
                    current = buffer.omega_filteredrpm if buffer.use_filtered_on else buffer.omega_rpm
                    u, error = pid.compute(buffer.omega_d, current)
                    if abs(error) > config['error_threshold']:
                        u = 0.0
                    min_duty = config.get('min_duty', 0.05)  # Add to config.json, e.g., 0.05
                    if abs(u) > 0 and abs(u) < min_duty:
                        u = min_duty * np.sign(u)
                    u = np.clip(u, -1.0, 1.0)
                    buffer.u_duty = u
                    buffer.error = error
            else:
                pid.integral = 0.0
                pid.prev_error = 0.0
                buffer.u_duty = 0.0
                buffer.error = 0.0
        time.sleep(dt)