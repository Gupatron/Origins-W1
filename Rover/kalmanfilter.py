import time
import json
import numpy as np

config = json.load(open('config.json'))

class KalmanFilter:
    def __init__(self, x0=0.0, P0=100.0, Q=0.1, R=10.0):
        self.x = x0
        self.P = P0
        self.Q = Q
        self.R = R

    def predict(self):
        self.P += self.Q

    def update(self, z):
        K = self.P / (self.P + self.R)
        self.x += K * (z - self.x)
        self.P = (1 - K) * self.P

def kalman_thread(buffer, stop_event):
    kf = KalmanFilter()
    dt = 1 / config['kalman_frequency']
    while not stop_event.is_set():
        with buffer.lock:
            kf.predict()
            kf.update(buffer.omega_erpm)
            buffer.omega_filterederpm = kf.x
            buffer.omega_filteredrpm = kf.x / config['wheel_rpm_const']
        time.sleep(dt)