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
            # MODIFIED: Logic now depends on the bypass_pid flag
            if buffer.bypass_pid:
                # --- PID BYPASS MODE ---
                if buffer.running:
                    # In bypass mode, duty cycle is set directly from the GUI command
                    buffer.u_duty = np.clip(buffer.direct_duty, -1.0, 1.0)
                else:
                    buffer.u_duty = 0.0
                
                # Reset PID state so it starts fresh if we switch back
                pid.integral = 0.0
                pid.prev_error = 0.0
                buffer.error = 0.0 # No error in bypass mode

            else:
                # --- NORMAL PID MODE ---
                if buffer.running:
                    current = buffer.omega_filteredrpm if buffer.use_filtered_on else buffer.omega_rpm
                    pid.kp = buffer.kp
                    pid.ki = buffer.ki
                    pid.kd = buffer.kd
                    u, error = pid.compute(buffer.omega_d, current)
                    if abs(error) > config['error_threshold']:
                        u = 0.0
                    u = np.clip(u, -1.0, 1.0)
                    buffer.u_duty = u
                    buffer.error = error
                else:
                    # Reset PID state when not running
                    pid.integral = 0.0
                    pid.prev_error = 0.0
                    buffer.u_duty = 0.0
                    buffer.error = 0.0
                    
        time.sleep(dt)
