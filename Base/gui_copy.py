import json
import time

from PyQt6.QtWidgets import (
    QMainWindow, QVBoxLayout, QLabel, QLineEdit, QPushButton,
    QFileDialog, QSlider, QWidget, QHBoxLayout
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
        central_widget = QWidget()
        layout = QVBoxLayout()

        # ===================== PID config =====================
        kp_label = QLabel('Kp:')
        self.kp_edit = QLineEdit()
        self.kp_edit.returnPressed.connect(self.update_pid)
        ki_label = QLabel('Ki:')
        self.ki_edit = QLineEdit()
        self.ki_edit.returnPressed.connect(self.update_pid)
        kd_label = QLabel('Kd:')
        self.kd_edit = QLineEdit()
        self.kd_edit.returnPressed.connect(self.update_pid)

        # ===================== Desired throttle =====================
        desired_label = QLabel('Desired RPM:')
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, int(config['max_erpm'] / config['wheel_rpm_const']))
        self.slider.valueChanged.connect(self.update_desired)
        self.desired_edit = QLineEdit('0')
        self.desired_edit.setValidator(QIntValidator(0, int(config['max_erpm'] / config['wheel_rpm_const'])))
        self.desired_edit.returnPressed.connect(self.update_slider)

        # Omega s + duty displays
        self.omega_s_label = QLabel('Omega S: 0')
        self.duty_label = QLabel('Duty: 0')

        # ===================== Servo controls (NEW) =====================
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
        save_btn = QPushButton('Save Gains')
        save_btn.clicked.connect(self.save_gains)
        load_btn = QPushButton('Load Gains')
        load_btn.clicked.connect(self.load_gains)

        # Run/stop/timed
        run_btn = QPushButton('Run Motor')
        run_btn.clicked.connect(self.run_motor)
        stop_btn = QPushButton('Stop Motor')
        stop_btn.clicked.connect(self.stop_motor)
        time_label = QLabel('Run Time (s):')
        self.time_edit = QLineEdit('20')
        timed_btn = QPushButton('Run for Time')
        timed_btn.clicked.connect(self.timed_run)

        # Plots
        error_plot_btn = QPushButton('Error Plot')
        error_plot_btn.clicked.connect(lambda: self.open_plot('error'))
        rpm_plot_btn = QPushButton('RPM Plot')
        rpm_plot_btn.clicked.connect(lambda: self.open_plot('rpm'))
        duty_plot_btn = QPushButton('Duty Plot')
        duty_plot_btn.clicked.connect(lambda: self.open_plot('duty'))

        # ===== Layout pack =====
        layout.addWidget(kp_label); layout.addWidget(self.kp_edit)
        layout.addWidget(ki_label); layout.addWidget(self.ki_edit)
        layout.addWidget(kd_label); layout.addWidget(self.kd_edit)

        layout.addWidget(desired_label)
        layout.addWidget(self.slider)
        layout.addWidget(self.desired_edit)

        # Servo UI
        layout.addWidget(servo_header)
        container = QWidget(); container.setLayout(servo_row)
        layout.addWidget(container)

        layout.addWidget(self.omega_s_label)
        layout.addWidget(self.duty_label)
        layout.addWidget(save_btn); layout.addWidget(load_btn)
        layout.addWidget(run_btn); layout.addWidget(stop_btn)
        layout.addWidget(time_label); layout.addWidget(self.time_edit); layout.addWidget(timed_btn)
        layout.addWidget(error_plot_btn); layout.addWidget(rpm_plot_btn); layout.addWidget(duty_plot_btn)

        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        # Update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_displays)
        self.timer.start(100)

        # Load default gains into GUI
        try:
            with open('gain_default.json') as f:
                gains = json.load(f)
                self.kp_edit.setText(str(gains['kp']))
                self.ki_edit.setText(str(gains['ki']))
                self.kd_edit.setText(str(gains['kd']))
        except FileNotFoundError:
            print("Default gains file not found")

    # ------------ Keyboard arrows for jogging ------------
    def keyPressEvent(self, e: QKeyEvent):
        if e.key() == Qt.Key.Key_Left:
            self.jog_servo(-1)
        elif e.key() == Qt.Key.Key_Right:
            self.jog_servo(+1)
        else:
            super().keyPressEvent(e)

    def publish_servo_angle(self):
        try:
            angle = int(self.servo_edit.text())
            angle = max(SERVO_MIN, min(SERVO_MAX, angle))
            with self.buffer.lock:
                self.buffer.servo_angle = angle
            msg = {
                'kp': self.buffer.kp,
                'ki': self.buffer.ki,
                'kd': self.buffer.kd,
                'omega_d': self.buffer.omega_d,
                'running': self.buffer.running,
                'timed_run': False,
                'run_time': 0.0,
                'servo_angle': angle
            }
            self.zenoh_session.put(COMMAND_KEY, json.dumps(msg))
        except ValueError:
            pass

    def jog_servo(self, delta):
        # Send a small relative change; rover will apply and clamp.
        msg = {
            'kp': self.buffer.kp,
            'ki': self.buffer.ki,
            'kd': self.buffer.kd,
            'omega_d': self.buffer.omega_d,
            'running': self.buffer.running,
            'timed_run': False,
            'run_time': 0.0,
            'servo_jog': int(delta)
        }
        self.zenoh_session.put(COMMAND_KEY, json.dumps(msg))

        # Also update local display optimistically
        try:
            cur = int(self.servo_edit.text())
        except Exception:
            cur = 90
        cur = max(SERVO_MIN, min(SERVO_MAX, cur + int(delta)))
        self.servo_edit.setText(str(cur))

    # ------------ Existing motor GUI code ------------
    def update_pid(self):
        try:
            with self.buffer.lock:
                self.buffer.kp = float(self.kp_edit.text())
                self.buffer.ki = float(self.ki_edit.text())
                self.buffer.kd = float(self.kd_edit.text())
            msg = {
                'kp': self.buffer.kp,
                'ki': self.buffer.ki,
                'kd': self.buffer.kd,
                'omega_d': self.buffer.omega_d,
                'running': self.buffer.running,
                'timed_run': False,
                'run_time': 0.0
            }
            self.zenoh_session.put(COMMAND_KEY, json.dumps(msg))
        except ValueError:
            print("Invalid PID values entered")

    def update_desired(self, value):
        self.desired_edit.setText(str(value))
        with self.buffer.lock:
            self.buffer.omega_d = float(value)
        msg = {
            'kp': self.buffer.kp,
            'ki': self.buffer.ki,
            'kd': self.buffer.kd,
            'omega_d': self.buffer.omega_d,
            'running': self.buffer.running,
            'timed_run': False,
            'run_time': 0.0
        }
        self.zenoh_session.put(COMMAND_KEY, json.dumps(msg))

    def update_slider(self):
        try:
            value = int(self.desired_edit.text())
            self.slider.setValue(value)
            with self.buffer.lock:
                self.buffer.omega_d = float(value)
            msg = {
                'kp': self.buffer.kp,
                'ki': self.buffer.ki,
                'kd': self.buffer.kd,
                'omega_d': self.buffer.omega_d,
                'running': self.buffer.running,
                'timed_run': False,
                'run_time': 0.0
            }
            self.zenoh_session.put(COMMAND_KEY, json.dumps(msg))
        except ValueError:
            pass

    def update_displays(self):
        with self.buffer.lock:
            self.omega_s_label.setText(f'Omega S: {self.buffer.omega_erpm:.1f}')
            self.duty_label.setText(f'Duty: {self.buffer.u_duty:.4f}')

    def save_gains(self):
        try:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f'gain_{timestamp}.json'
            gains = {
                'kp': float(self.kp_edit.text()),
                'ki': float(self.ki_edit.text()),
                'kd': float(self.kd_edit.text())
            }
            with open(filename, 'w') as f:
                json.dump(gains, f)
        except ValueError:
            print("Invalid values for saving gains")

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
            except Exception as e:
                print(f"Error loading gains: {e}")

    def run_motor(self):
        with self.buffer.lock:
            self.buffer.running = True
        msg = {
            'kp': self.buffer.kp,
            'ki': self.buffer.ki,
            'kd': self.buffer.kd,
            'omega_d': self.buffer.omega_d,
            'running': True,
            'timed_run': False,
            'run_time': 0.0
        }
        self.zenoh_session.put(COMMAND_KEY, json.dumps(msg))

    def stop_motor(self):
        with self.buffer.lock:
            self.buffer.running = False
        msg = {
            'kp': self.buffer.kp,
            'ki': self.buffer.ki,
            'kd': self.buffer.kd,
            'omega_d': self.buffer.omega_d,
            'running': False,
            'timed_run': False,
            'run_time': 0.0
        }
        self.zenoh_session.put(COMMAND_KEY, json.dumps(msg))

    def timed_run(self):
        try:
            run_time = float(self.time_edit.text())
            if run_time > 0:
                with self.buffer.lock:
                    self.buffer.running = True
                msg = {
                    'kp': self.buffer.kp,
                    'ki': self.buffer.ki,
                    'kd': self.buffer.kd,
                    'omega_d': self.buffer.omega_d,
                    'running': True,
                    'timed_run': True,
                    'run_time': run_time
                }
                self.zenoh_session.put(COMMAND_KEY, json.dumps(msg))
                QTimer.singleShot(int(run_time * 1000), self.stop_motor)
        except ValueError:
            print("Invalid run time entered")

    def open_plot(self, plot_type):
        window = PlotWindow(f'{plot_type.capitalize()} Plot', self.buffer, plot_type)
        window.show()
        self.plot_windows.append(window)


