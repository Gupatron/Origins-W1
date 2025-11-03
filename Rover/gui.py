import json
import time

from PyQt6.QtWidgets import (
    QMainWindow, QVBoxLayout, QLabel, QLineEdit, QPushButton,
    QFileDialog, QSlider, QWidget, QHBoxLayout, QCheckBox
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QIntValidator, QKeyEvent
from PyQt6.QtWidgets import QDialog
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from databuffer import DataBuffer

COMMAND_KEY = 'f1tenth_pid/command'

config = json.load(open('config.json'))

SERVO_MIN = 50
SERVO_MAX = 130
MAX_RPM = 200
# NEW: Load max bypass duty from config for safety
MAX_BYPASS_DUTY = config.get('max_bypass_duty_percent', 20)

# MODIFIED: Decoupled acceleration rates for PID and Bypass modes.
RPM_ACCEL_DELTA = 10      # RPM increase/decrease per update (for PID mode)
DUTY_ACCEL_DELTA = 0.1     # Duty Cycle % increase/decrease per update (for Bypass mode)
FRICTION_DELTA = 5        # Natural deceleration per update
BRAKE_DELTA = 20          # Active braking deceleration per update
STEER_DELTA = 5           # Angle change per update
RETURN_DELTA = 1          # Angle return to center per update
CONTROL_INTERVAL = 20     # ms

class PlotWindow(QDialog):
    def __init__(self, title, buffer, plot_type):
        super().__init__()
        self.setWindowTitle(title)
        self.figure = Figure()
        self.canvas = FigureCanvasQTAgg(self.figure)
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)
        self.ax = self.figure.add_subplot(111)
        self.buffer = buffer
        self.plot_type = plot_type
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)

    def update_plot(self):
        with self.buffer.lock:
            times = self.buffer.times
            if self.plot_type == 'rpm':
                ds = self.buffer.omega_ds
                rpms = self.buffer.omega_rpms
                filtered = self.buffer.omega_filteredrpms
                self.ax.clear()
                self.ax.plot(times, ds, label='Desired')
                self.ax.plot(times, rpms, label='From Electronic Sensor')
                self.ax.plot(times, filtered, label='Filtered')
                self.ax.set_xlabel('Time (s)')
                self.ax.set_ylabel('RPM')
                self.ax.legend()
            elif self.plot_type == 'error':
                errors = self.buffer.errors
                self.ax.clear()
                self.ax.plot(times, errors, label='Error')
                self.ax.set_xlabel('Time (s)')
                self.ax.set_ylabel('Error')
                self.ax.legend()
            elif self.plot_type == 'duty':
                duties = self.buffer.duties
                self.ax.clear()
                self.ax.plot(times, duties, label='Duty Cycle')
                self.ax.set_xlabel('Time (s)')
                self.ax.set_ylabel('Duty Cycle')
                self.ax.legend()
            self.canvas.draw()

class GUI(QMainWindow):
    def __init__(self, buffer: DataBuffer, zenoh_session):
        super().__init__()
        self.buffer = buffer
        self.zenoh_session = zenoh_session
        self.plot_windows = []
        self.w_pressed = False
        self.s_pressed = False
        self.a_pressed = False
        self.d_pressed = False
        self.space_pressed = False
        self.is_pid_bypassed = False

        central_widget = QWidget()
        layout = QVBoxLayout()

        # PID config
        pid_layout = QHBoxLayout()
        kp_label = QLabel('Kp:')
        self.kp_edit = QLineEdit()
        self.kp_edit.returnPressed.connect(self.update_pid)
        ki_label = QLabel('Ki:')
        self.ki_edit = QLineEdit()
        self.ki_edit.returnPressed.connect(self.update_pid)
        kd_label = QLabel('Kd:')
        self.kd_edit = QLineEdit()
        self.kd_edit.returnPressed.connect(self.update_pid)
        pid_layout.addWidget(kp_label); pid_layout.addWidget(self.kp_edit)
        pid_layout.addWidget(ki_label); pid_layout.addWidget(self.ki_edit)
        pid_layout.addWidget(kd_label); pid_layout.addWidget(self.kd_edit)

        # Desired throttle
        self.desired_label = QLabel('Desired RPM:')
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(-MAX_RPM, MAX_RPM)
        self.slider.valueChanged.connect(self.update_desired)
        self.desired_edit = QLineEdit('0')
        self.desired_edit.setValidator(QIntValidator(-MAX_RPM, MAX_RPM))
        self.desired_edit.returnPressed.connect(self.update_slider)

        # PID Bypass Checkbox
        self.pid_bypass_checkbox = QCheckBox("Bypass PID Controller (Direct Duty Cycle Control)")
        self.pid_bypass_checkbox.stateChanged.connect(self.toggle_pid_bypass)

        # Omega s + duty displays
        self.omega_s_label = QLabel('Omega S: 0')
        self.duty_label = QLabel('Duty: 0')

        # Servo controls
        servo_header = QLabel('Servo Angle (50–130°):')
        self.servo_edit = QLineEdit('90')
        self.servo_edit.setValidator(QIntValidator(SERVO_MIN, SERVO_MAX))
        self.servo_edit.returnPressed.connect(self.publish_servo_angle)
        servo_row = QHBoxLayout()
        servo_set_btn = QPushButton('Set Servo')
        servo_set_btn.clicked.connect(self.publish_servo_angle)
        servo_left_btn = QPushButton('◀ Left')
        servo_left_btn.clicked.connect(lambda: self.jog_servo(-1))
        servo_right_btn = QPushButton('Right ▶')
        servo_right_btn.clicked.connect(lambda: self.jog_servo(+1))
        servo_row.addWidget(self.servo_edit)
        servo_row.addWidget(servo_set_btn)
        servo_row.addWidget(servo_left_btn)
        servo_row.addWidget(servo_right_btn)

        # Save/load
        save_load_layout = QHBoxLayout()
        save_btn = QPushButton('Save Gains')
        save_btn.clicked.connect(self.save_gains)
        load_btn = QPushButton('Load Gains')
        load_btn.clicked.connect(self.load_gains)
        save_load_layout.addWidget(save_btn)
        save_load_layout.addWidget(load_btn)

        # Run controls
        run_layout = QHBoxLayout()
        run_btn = QPushButton('Run Motor')
        run_btn.clicked.connect(self.run_motor)
        stop_btn = QPushButton('Stop Motor')
        stop_btn.clicked.connect(self.stop_motor)
        self.time_edit = QLineEdit('0.0')
        timed_run_btn = QPushButton('Timed Run (s)')
        timed_run_btn.clicked.connect(self.timed_run)
        run_layout.addWidget(run_btn)
        run_layout.addWidget(stop_btn)
        run_layout.addWidget(self.time_edit)
        run_layout.addWidget(timed_run_btn)

        # Plots
        plot_layout = QHBoxLayout()
        rpm_plot_btn = QPushButton('Plot RPM')
        rpm_plot_btn.clicked.connect(lambda: self.open_plot('rpm'))
        error_plot_btn = QPushButton('Plot Error')
        error_plot_btn.clicked.connect(lambda: self.open_plot('error'))
        duty_plot_btn = QPushButton('Plot Duty')
        duty_plot_btn.clicked.connect(lambda: self.open_plot('duty'))
        plot_layout.addWidget(rpm_plot_btn)
        plot_layout.addWidget(error_plot_btn)
        plot_layout.addWidget(duty_plot_btn)

        # Assemble layout
        layout.addLayout(pid_layout)
        layout.addWidget(self.desired_label)
        layout.addWidget(self.slider)
        layout.addWidget(self.desired_edit)
        layout.addWidget(self.pid_bypass_checkbox)
        layout.addWidget(self.omega_s_label)
        layout.addWidget(self.duty_label)
        layout.addWidget(servo_header)
        layout.addLayout(servo_row)
        layout.addLayout(save_load_layout)
        layout.addLayout(run_layout)
        layout.addLayout(plot_layout)

        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        # Timers
        self.control_timer = QTimer()
        self.control_timer.timeout.connect(self.handle_keyboard_controls)
        self.control_timer.start(CONTROL_INTERVAL)

        self.display_timer = QTimer()
        self.display_timer.timeout.connect(self.update_displays)
        self.display_timer.start(100)

        # Load initial PID
        with self.buffer.lock:
            self.kp_edit.setText(str(self.buffer.kp))
            self.ki_edit.setText(str(self.buffer.ki))
            self.kd_edit.setText(str(self.buffer.kd))

    def keyPressEvent(self, event: QKeyEvent):
        if event.key() == Qt.Key.Key_W: self.w_pressed = True
        elif event.key() == Qt.Key.Key_S: self.s_pressed = True
        elif event.key() == Qt.Key.Key_A: self.a_pressed = True
        elif event.key() == Qt.Key.Key_D: self.d_pressed = True
        elif event.key() == Qt.Key.Key_Space: self.space_pressed = True

    def keyReleaseEvent(self, event: QKeyEvent):
        if event.key() == Qt.Key.Key_W: self.w_pressed = False
        elif event.key() == Qt.Key.Key_S: self.s_pressed = False
        elif event.key() == Qt.Key.Key_A: self.a_pressed = False
        elif event.key() == Qt.Key.Key_D: self.d_pressed = False
        elif event.key() == Qt.Key.Key_Space: self.space_pressed = False

    def handle_keyboard_controls(self):
        with self.buffer.lock:
            current_val = self.buffer.omega_d
            current_angle = self.buffer.servo_angle

        val_changed = False
        angle_changed = False

        # Handle throttle
        delta_val = 0
        if self.is_pid_bypassed:
            if self.w_pressed: delta_val += DUTY_ACCEL_DELTA
            if self.s_pressed: delta_val -= DUTY_ACCEL_DELTA
        else:
            if self.w_pressed: delta_val += RPM_ACCEL_DELTA
            if self.s_pressed: delta_val -= RPM_ACCEL_DELTA

        new_val = current_val
        if self.w_pressed or self.s_pressed:
            new_val += delta_val
        else:
            if current_val > 0: new_val = max(0, current_val - FRICTION_DELTA)
            elif current_val < 0: new_val = min(0, current_val + FRICTION_DELTA)

        if self.space_pressed:
            if new_val > 0: new_val = max(0, new_val - BRAKE_DELTA)
            elif new_val < 0: new_val = min(0, new_val + BRAKE_DELTA)

        if self.is_pid_bypassed:
            new_val = max(-MAX_BYPASS_DUTY, min(MAX_BYPASS_DUTY, new_val))
        else:
            new_val = max(-MAX_RPM, min(MAX_RPM, new_val))

        if new_val != current_val:
            val_changed = True
            self.desired_edit.blockSignals(True)
            self.slider.blockSignals(True)
            self.desired_edit.setText(str(int(new_val)))
            self.slider.setValue(int(new_val))
            self.desired_edit.blockSignals(False)
            self.slider.blockSignals(False)
            with self.buffer.lock: self.buffer.omega_d = new_val

        # Handle steering
        delta_angle = 0
        if self.a_pressed: delta_angle -= STEER_DELTA
        if self.d_pressed: delta_angle += STEER_DELTA

        new_angle = current_angle
        if self.a_pressed or self.d_pressed: new_angle += delta_angle
        else:
            if current_angle > 90: new_angle = max(90, current_angle - RETURN_DELTA)
            elif current_angle < 90: new_angle = min(90, current_angle + RETURN_DELTA)

        new_angle = max(SERVO_MIN, min(SERVO_MAX, new_angle))

        if new_angle != current_angle:
            angle_changed = True
            self.servo_edit.setText(str(int(new_angle)))
            with self.buffer.lock: self.buffer.servo_angle = new_angle

        if val_changed or angle_changed: self._publish_command()

    def _publish_command(self, is_timed_run=False, run_time=0.0):
        with self.buffer.lock:
            kp, ki, kd = self.buffer.kp, self.buffer.ki, self.buffer.kd
            omega_d = self.buffer.omega_d
            running = self.buffer.running
            servo_angle = self.buffer.servo_angle

        msg = {
            'kp': kp, 'ki': ki, 'kd': kd,
            'running': running,
            'timed_run': is_timed_run,
            'run_time': run_time,
            'servo_angle': servo_angle,
            'bypass_pid': self.is_pid_bypassed
        }
        
        if self.is_pid_bypassed:
            msg['direct_duty'] = omega_d / 100.0
            msg['omega_d'] = 0
        else:
            msg['omega_d'] = omega_d
            msg['direct_duty'] = 0.0
            
        self.zenoh_session.put(COMMAND_KEY, json.dumps(msg))

    def publish_servo_angle(self):
        try:
            angle = int(self.servo_edit.text())
            angle = max(SERVO_MIN, min(SERVO_MAX, angle))
            with self.buffer.lock: self.buffer.servo_angle = angle
            self._publish_command()
        except ValueError: pass

    def jog_servo(self, delta):
        try:
            cur = int(self.servo_edit.text())
        except Exception: cur = 90
        cur = max(SERVO_MIN, min(SERVO_MAX, cur + int(delta)))
        self.servo_edit.setText(str(cur))
        with self.buffer.lock: self.buffer.servo_angle = cur
        self._publish_command()

    def toggle_pid_bypass(self, state):
        self.is_pid_bypassed = (state == Qt.CheckState.Checked.value)
        self.slider.setValue(0)
        if self.is_pid_bypassed:
            self.desired_label.setText("Desired Duty Cycle (%):")
            self.slider.setRange(-MAX_BYPASS_DUTY, MAX_BYPASS_DUTY)  # NEW: Limit range for safety
            self.desired_edit.setValidator(QIntValidator(-MAX_BYPASS_DUTY, MAX_BYPASS_DUTY))  # NEW: Limit validator
        else:
            self.desired_label.setText("Desired RPM:")
            self.slider.setRange(-MAX_RPM, MAX_RPM)
            self.desired_edit.setValidator(QIntValidator(-MAX_RPM, MAX_RPM))
        self.update_slider()

    def update_pid(self):
        try:
            with self.buffer.lock:
                self.buffer.kp = float(self.kp_edit.text())
                self.buffer.ki = float(self.ki_edit.text())
                self.buffer.kd = float(self.kd_edit.text())
            self._publish_command()
        except ValueError: print("Invalid PID values entered")

    def update_desired(self, value):
        self.desired_edit.setText(str(value))
        with self.buffer.lock: self.buffer.omega_d = float(value)
        self._publish_command()

    def update_slider(self):
        try:
            value = int(self.desired_edit.text())
            self.slider.setValue(value)
            with self.buffer.lock: self.buffer.omega_d = float(value)
            self._publish_command()
        except ValueError: pass

    def update_displays(self):
        with self.buffer.lock:
            self.omega_s_label.setText(f'Omega S: {self.buffer.omega_erpm:.1f}')
            self.duty_label.setText(f'Duty: {self.buffer.u_duty:.4f}')

    def save_gains(self):
        try:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f'gain_{timestamp}.json'
            gains = { 'kp': float(self.kp_edit.text()), 'ki': float(self.ki_edit.text()), 'kd': float(self.kd_edit.text()) }
            with open(filename, 'w') as f: json.dump(gains, f)
        except ValueError: print("Invalid values for saving gains")

    def load_gains(self):
        filename, _ = QFileDialog.getOpenFileName(self, 'Load Gains', '', 'JSON (*.json)')
        if filename:
            try:
                with open(filename) as f:
                    gains = json.load(f)
                    self.kp_edit.setText(str(gains['kp']))
                    self.ki_edit.setText(str(gains['ki']))
                    self.kd_edit.setText(str(gains['kd']))
                    self.update_pid()
            except Exception as e: print(f"Error loading gains: {e}")

    def run_motor(self):
        with self.buffer.lock: self.buffer.running = True
        self._publish_command()

    def stop_motor(self):
        with self.buffer.lock: self.buffer.running = False
        self._publish_command()

    def timed_run(self):
        try:
            run_time = float(self.time_edit.text())
            if run_time > 0:
                with self.buffer.lock: self.buffer.running = True
                self._publish_command(is_timed_run=True, run_time=run_time)
        except ValueError: print("Invalid run time entered")

    def open_plot(self, plot_type):
        window = PlotWindow(f'{plot_type.capitalize()} Plot', self.buffer, plot_type)
        window.show()
        self.plot_windows.append(window)
