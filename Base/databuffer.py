import threading

class DataBuffer:
    def __init__(self):
        self.lock = threading.Lock()
        self.omega_erpm = 0.0
        self.omega_rpm = 0.0
        self.omega_filterederpm = 0.0
        self.omega_filteredrpm = 0.0
        self.u_duty = 0.0
        self.omega_d = 0.0
        self.error = 0.0
        self.running = False
        self.use_filtered_on = False
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.times = []
        self.omega_ds = []
        self.omega_rpms = []
        self.omega_filteredrpms = []
        self.errors = []
        self.duties = []
        self.servo_offset = 0
        self.servo_angle = 90
        self.servo_offsets = []
        self.servo_angles = []
