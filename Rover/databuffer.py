import threading

class DataBuffer:
    def __init__(self):
        self.lock = threading.Lock()

        # Telemetry (from VESC)
        self.omega_erpm = 0.0
        self.omega_rpm = 0.0
        self.omega_filterederpm = 0.0
        self.omega_filteredrpm = 0.0

        # Controller IO
        self.u_duty = 0.0        # what we send to the VESC ([-1, 1])
        self.omega_d = 0.0       # desired RPM (can be +/-)
        self.error = 0.0

        # State/control flags
        self.running = False
        self.use_filtered_on = False

        # PID gains
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0

        # --- Added fields for control modes / braking ---
        self.bypass_pid = False              # When True: map desired RPM -> duty directly
        self.brake_active = False            # When True: aggressive decel toward 0 duty
        self.max_rpm_for_bypass = 200.0      # Linear mapping scale for bypass

        # Historical arrays (if you plot/log)
        self.times = []
        self.omega_ds = []
        self.omega_rpms = []
        self.omega_filteredrpms = []
        self.errors = []
        self.duties = []

        # Steering (unchanged)
        self.servo_offset = 0
        self.servo_angle = 90
        self.servo_offsets = []
        self.servo_angles = []
