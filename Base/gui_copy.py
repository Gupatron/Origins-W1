# gui_merged.py
import json
import time
import pygame
from threading import Thread

from PyQt6.QtWidgets import (
    QMainWindow, QVBoxLayout, QLabel, QLineEdit, QPushButton,
    QFileDialog, QSlider, QWidget, QHBoxLayout, QCheckBox, QFrame
)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt6.QtGui import QIntValidator, QKeyEvent, QPainter, QColor
from PyQt6.QtWidgets import QDialog
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from databuffer import DataBuffer

COMMAND_KEY = 'f1tenth_pid/command'

# --- CONFIG LOAD ---
try:
    config = json.load(open('config.json'))
except:
    config = {}

SERVO_MIN = 50
SERVO_MAX = 130
MAX_RPM = 200

MIN_START_DUTY_PERCENT = 2  # Adjust this if the minimum duty to start moving is different from 2%
TRIGGER_MAX = 0.8  # Change this to the R2 value you see when fully pressing the trigger
# CHANGED: Max duty in bypass mode = 15%
MAX_BYPASS_DUTY = config.get('max_bypass_duty_percent', 20)  # DEFAULT 15

# Acceleration / deceleration constants
RPM_ACCEL_DELTA   = 10      # per update in PID mode
DUTY_ACCEL_DELTA  = 4    # per update in Bypass mode
FRICTION_DELTA    = 3
BRAKE_DELTA       = 1
STEER_DELTA       = 5
RETURN_DELTA      = 1
CONTROL_INTERVAL  = 20      # ms


# ----------------------------------------------------------------------
# Plot windows
# ----------------------------------------------------------------------
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


# ----------------------------------------------------------------------
# Virtual controller widget
# ----------------------------------------------------------------------
class ControllerWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(180, 240)
        self.left_x = self.left_y = self.r2 = self.l2 = 0.0

    def update_values(self, left_x, left_y, r2, l2):
        self.left_x, self.left_y, self.r2, self.l2 = left_x, left_y, r2, l2
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor(30, 30, 30))

        # left stick
        cx, cy = 60, 120
        radius = 30
        p.setPen(QColor(200, 200, 200))
        p.drawEllipse(cx - radius, cy - radius, radius * 2, radius * 2)
        p.setBrush(QColor(100, 180, 255))
        p.drawEllipse(cx + int(self.left_x * radius) - 8,
                      cy + int(self.left_y * radius) - 8, 16, 16)

        # trigger bars
        bar_w, bar_h = 80, 12
        p.setPen(Qt.PenStyle.NoPen)

        # R2
        p.setBrush(QColor(255, 100, 100))
        p.drawRect(90, 30, int(self.r2 * bar_w), bar_h)
        p.setPen(QColor(200, 200, 200))
        p.drawRect(90, 30, bar_w, bar_h)

        # L2
        p.setBrush(QColor(100, 255, 100))
        p.drawRect(90, 50, int(self.l2 * bar_w), bar_h)
        p.setPen(QColor(200, 200, 200))
        p.drawRect(90, 50, bar_w, bar_h)

        # text labels
        p.setPen(QColor(255, 255, 255))
        p.drawText(10, 20, f"LX: {self.left_x:+.2f}")
        p.drawText(10, 40, f"LY: {self.left_y:+.2f}")
        p.drawText(10, 80, f"R2: {self.r2:.2f}")
        p.drawText(10, 100, f"L2: {self.l2:.2f}")


# ----------------------------------------------------------------------
# Background joystick poller
# ----------------------------------------------------------------------
class JoystickWorker(QObject):
    values_changed = pyqtSignal(float, float, float, float)   # LX, LY, R2, L2

    def __init__(self):
        super().__init__()
        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        self.running = True

    def run(self):
        while self.running:
            if self.joystick:
                pygame.event.pump()
                lx = self.joystick.get_axis(0)
                ly = self.joystick.get_axis(1)
                l2_raw = self.joystick.get_axis(2)
                r2_raw = self.joystick.get_axis(5)
                r2 = (r2_raw + 1) / 2
                l2 = (l2_raw + 1) / 2
                self.values_changed.emit(lx, ly, r2, l2)
            time.sleep(0.02)

    def stop(self):
        self.running = False


# ----------------------------------------------------------------------
# Main GUI
# ----------------------------------------------------------------------
class GUI(QMainWindow):
    def __init__(self, buffer: DataBuffer, zenoh_session):
        super().__init__()
        self.buffer = buffer
        self.zenoh_session = zenoh_session
        self.plot_windows = []

        # keyboard state
        self.w_pressed = self.s_pressed = self.a_pressed = self.d_pressed = self.space_pressed = False
        self.is_pid_bypassed = False

        # joystick
        self.joystick_worker = JoystickWorker()
        self.joystick_thread = None
        self.controller_widget = ControllerWidget()

        # UI
        central = QWidget()
        layout = QVBoxLayout()

        # PID
        pid_lay = QHBoxLayout()
        pid_lay.addWidget(QLabel('Kp:'))
        self.kp_edit = QLineEdit()
        self.kp_edit.setFixedWidth(80)
        self.kp_edit.returnPressed.connect(self.update_pid)
        pid_lay.addWidget(self.kp_edit)

        pid_lay.addWidget(QLabel('Ki:'))
        self.ki_edit = QLineEdit()
        self.ki_edit.setFixedWidth(80)
        self.ki_edit.returnPressed.connect(self.update_pid)
        pid_lay.addWidget(self.ki_edit)

        pid_lay.addWidget(QLabel('Kd:'))
        self.kd_edit = QLineEdit()
        self.kd_edit.setFixedWidth(80)
        self.kd_edit.returnPressed.connect(self.update_pid)
        pid_lay.addWidget(self.kd_edit)

        # Desired
        self.desired_label = QLabel('Desired RPM:')
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(-MAX_RPM, MAX_RPM)
        self.slider.valueChanged.connect(self.update_desired)
        self.desired_edit = QLineEdit('0')
        self.desired_edit.setValidator(QIntValidator(-MAX_RPM, MAX_RPM))
        self.desired_edit.returnPressed.connect(self.update_slider)

        # Checkboxes
        self.pid_bypass_checkbox = QCheckBox("Bypass PID Controller (Direct Duty Cycle Control)")
        self.pid_bypass_checkbox.stateChanged.connect(self.toggle_pid_bypass)

        self.controller_checkbox = QCheckBox("Enable Controller")
        self.controller_checkbox.stateChanged.connect(self.toggle_controller)
        if not self.joystick_worker.joystick:
            self.controller_checkbox.setEnabled(False)
            self.controller_checkbox.setText("Enable Controller (none detected)")

        # Controller view
        ctrl_frame = QFrame()
        ctrl_frame.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Sunken)
        ctrl_lay = QHBoxLayout()
        ctrl_lay.addWidget(QLabel("Controller:"))
        ctrl_lay.addWidget(self.controller_widget)
        ctrl_frame.setLayout(ctrl_lay)

        # Status
        self.omega_s_label = QLabel('Omega S: 0')
        self.duty_label = QLabel('Duty: 0')

        # Servo
        servo_head = QLabel('Servo Angle (50–130 degrees):')
        self.servo_edit = QLineEdit('90')
        self.servo_edit.setValidator(QIntValidator(SERVO_MIN, SERVO_MAX))
        self.servo_edit.returnPressed.connect(self.publish_servo_angle)

        servo_row = QHBoxLayout()
        servo_row.addWidget(self.servo_edit)
        servo_row.addWidget(QPushButton('Set Servo', clicked=self.publish_servo_angle))
        servo_row.addWidget(QPushButton('Left', clicked=lambda: self.jog_servo(-1)))
        servo_row.addWidget(QPushButton('Right', clicked=lambda: self.jog_servo(+1)))

        # Save/Load
        save_load = QHBoxLayout()
        save_load.addWidget(QPushButton('Save Gains', clicked=self.save_gains))
        save_load.addWidget(QPushButton('Load Gains', clicked=self.load_gains))

        # Run
        run_lay = QHBoxLayout()
        run_lay.addWidget(QPushButton('Run Motor', clicked=self.run_motor))
        run_lay.addWidget(QPushButton('Stop Motor', clicked=self.stop_motor))
        self.time_edit = QLineEdit('0.0')
        run_lay.addWidget(self.time_edit)
        run_lay.addWidget(QPushButton('Timed Run (s)', clicked=self.timed_run))

        # Plot
        plot_lay = QHBoxLayout()
        for txt, typ in zip(('Plot RPM', 'Plot Error', 'Plot Duty'), ('rpm', 'error', 'duty')):
            btn = QPushButton(txt)
            btn.clicked.connect(lambda _, t=typ: self.open_plot(t))
            plot_lay.addWidget(btn)

        # Assemble
        for item in (pid_lay, self.desired_label, self.slider, self.desired_edit,
                     self.pid_bypass_checkbox, self.controller_checkbox, ctrl_frame,
                     self.omega_s_label, self.duty_label, servo_head, servo_row,
                     save_load, run_lay, plot_lay):
            if isinstance(item, QHBoxLayout):
                layout.addLayout(item)
            else:
                layout.addWidget(item)

        central.setLayout(layout)
        self.setCentralWidget(central)

        # Timers
        self.control_timer = QTimer()
        self.control_timer.timeout.connect(self.handle_controls)
        self.control_timer.start(CONTROL_INTERVAL)

        self.display_timer = QTimer()
        self.display_timer.timeout.connect(self.update_displays)
        self.display_timer.start(100)

        # Load PID
        with self.buffer.lock:
            self.kp_edit.setText(str(self.buffer.kp))
            self.ki_edit.setText(str(self.buffer.ki))
            self.kd_edit.setText(str(self.buffer.kd))

    # --- Keyboard ---
    def keyPressEvent(self, event: QKeyEvent):
        k = event.key()
        if k == Qt.Key.Key_W: self.w_pressed = True
        elif k == Qt.Key.Key_S: self.s_pressed = True
        elif k == Qt.Key.Key_A: self.a_pressed = True
        elif k == Qt.Key.Key_D: self.d_pressed = True
        elif k == Qt.Key.Key_Space: self.space_pressed = True

    def keyReleaseEvent(self, event: QKeyEvent):
        k = event.key()
        if k == Qt.Key.Key_W: self.w_pressed = False
        elif k == Qt.Key.Key_S: self.s_pressed = False
        elif k == Qt.Key.Key_A: self.a_pressed = False
        elif k == Qt.Key.Key_D: self.d_pressed = False
        elif k == Qt.Key.Key_Space: self.space_pressed = False

    # --- Controller Toggle ---
    def toggle_controller(self, state):
        if state == Qt.CheckState.Checked.value and self.joystick_worker.joystick:
            if self.joystick_thread is None:
                self.joystick_thread = Thread(target=self.joystick_worker.run, daemon=True)
                self.joystick_thread.start()
                self.joystick_worker.values_changed.connect(self.on_joystick_values)
        else:
            if self.joystick_thread:
                self.joystick_worker.stop()
                self.joystick_thread.join()
                self.joystick_thread = None
                try:
                    self.joystick_worker.values_changed.disconnect()
                except:
                    pass

    def on_joystick_values(self, lx, ly, r2, l2):
        self.controller_widget.update_values(lx, ly, r2, l2)

    # --- Control Loop ---
    def handle_controls(self):
        with self.buffer.lock:
            cur_val = self.buffer.omega_d
            cur_angle = self.buffer.servo_angle

        val_changed = angle_changed = False
        accel_step = DUTY_ACCEL_DELTA if self.is_pid_bypassed else RPM_ACCEL_DELTA

        throttle_input = steer_input = 0.0
        brake = False

        use_controller = (self.controller_checkbox.isChecked() and
                          self.joystick_worker.joystick and
                          self.joystick_thread is not None)

        if use_controller:
            lx = self.controller_widget.left_x
            r2 = self.controller_widget.r2
            l2 = self.controller_widget.l2
            throttle_input = r2 - l2
            steer_input = lx
        else:
            if self.w_pressed: throttle_input += 1.0
            if self.s_pressed: throttle_input -= 1.0
            if self.a_pressed: steer_input -= 1.0
            if self.d_pressed: steer_input += 1.0
            brake = self.space_pressed

        # Apply dead-zones regardless of input method
        if abs(throttle_input) < 0.01:
            throttle_input = 0.0  # Lower dead-zone for throttle
        if abs(steer_input) < 0.05:
            steer_input = 0.0  # Lower dead-zone for steering

        # Throttle
        max_val = MAX_BYPASS_DUTY if self.is_pid_bypassed else MAX_RPM
        min_start = MIN_START_DUTY_PERCENT if self.is_pid_bypassed else 0

        if throttle_input != 0:
            abs_throttle = abs(throttle_input)
            sign = 1 if throttle_input > 0 else -1
            scaled_throttle = min(1.0, abs_throttle / TRIGGER_MAX)
            target = min_start + (max_val - min_start) * scaled_throttle
            new_val = sign * target
        else:
            if cur_val > 0:
                new_val = max(0, cur_val - FRICTION_DELTA)
            elif cur_val < 0:
                new_val = min(0, cur_val + FRICTION_DELTA)
            else:
                new_val = 0

        if brake:
            if new_val > 0:
                new_val = max(0, new_val - BRAKE_DELTA)
            elif new_val < 0:
                new_val = min(0, new_val + BRAKE_DELTA)

        # Clamp
        new_val = max(-max_val, min(max_val, new_val))

        if new_val != cur_val:
            val_changed = True
            self.desired_edit.blockSignals(True)
            self.slider.blockSignals(True)
            self.desired_edit.setText(str(int(new_val)))
            self.slider.setValue(int(new_val))
            self.desired_edit.blockSignals(False)
            self.slider.blockSignals(False)
            with self.buffer.lock:
                self.buffer.omega_d = new_val

        # Steering
        delta_angle = steer_input * STEER_DELTA
        new_angle = cur_angle + delta_angle

        if steer_input == 0:
            if cur_angle > 90:
                new_angle = max(90, cur_angle - RETURN_DELTA)
            elif cur_angle < 90:
                new_angle = min(90, cur_angle + RETURN_DELTA)

        new_angle = max(SERVO_MIN, min(SERVO_MAX, new_angle))

        if new_angle != cur_angle:
            angle_changed = True
            self.servo_edit.setText(str(int(new_angle)))
            with self.buffer.lock:
                self.buffer.servo_angle = new_angle

        self._publish_command()

    # --- Publish ---
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

    # --- Servo ---
    def publish_servo_angle(self):
        try:
            a = int(self.servo_edit.text())
            a = max(SERVO_MIN, min(SERVO_MAX, a))
            with self.buffer.lock:
                self.buffer.servo_angle = a
            self._publish_command()
        except:
            pass

    def jog_servo(self, delta):
        try:
            cur = int(self.servo_edit.text())
        except:
            cur = 90
        cur = max(SERVO_MIN, min(SERVO_MAX, cur + delta))
        self.servo_edit.setText(str(cur))
        with self.buffer.lock:
            self.buffer.servo_angle = cur
        self._publish_command()

    # --- PID Bypass ---
    def toggle_pid_bypass(self, state):
        self.is_pid_bypassed = (state == Qt.CheckState.Checked.value)
        self.slider.setValue(0)
        if self.is_pid_bypassed:
            self.desired_label.setText("Desired Duty Cycle (%):")
            self.slider.setRange(-MAX_BYPASS_DUTY, MAX_BYPASS_DUTY)
            self.desired_edit.setValidator(QIntValidator(-MAX_BYPASS_DUTY, MAX_BYPASS_DUTY))
        else:
            self.desired_label.setText("Desired RPM:")
            self.slider.setRange(-MAX_RPM, MAX_RPM)
            self.desired_edit.setValidator(QIntValidator(-MAX_RPM, MAX_RPM))
        self.update_slider()

    # --- PID Update ---
    def update_pid(self):
        try:
            with self.buffer.lock:
                self.buffer.kp = float(self.kp_edit.text())
                self.buffer.ki = float(self.ki_edit.text())
                self.buffer.kd = float(self.kd_edit.text())
            self._publish_command()
        except:
            print("Invalid PID values")

    # --- Slider Sync ---
    def update_desired(self, value):
        self.desired_edit.setText(str(value))
        with self.buffer.lock:
            self.buffer.omega_d = float(value)
        self._publish_command()

    def update_slider(self):
        try:
            v = int(self.desired_edit.text())
            self.slider.setValue(v)
            with self.buffer.lock:
                self.buffer.omega_d = float(v)
            self._publish_command()
        except:
            pass

    # --- Display ---
    def update_displays(self):
        with self.buffer.lock:
            self.omega_s_label.setText(f'Omega S: {self.buffer.omega_erpm:.1f}')
            self.duty_label.setText(f'Duty: {self.buffer.u_duty:.4f}')

    # --- Save/Load ---
    def save_gains(self):
        try:
            ts = time.strftime("%Y%m%d_%H%M%S")
            fn = f'gain_{ts}.json'
            data = {
                'kp': float(self.kp_edit.text()),
                'ki': float(self.ki_edit.text()),
                'kd': float(self.kd_edit.text())
            }
            with open(fn, 'w') as f:
                json.dump(data, f)
        except Exception as e:
            print("Save error:", e)

    def load_gains(self):
        fn, _ = QFileDialog.getOpenFileName(self, 'Load Gains', '', 'JSON (*.json)')
        if fn:
            try:
                with open(fn) as f:
                    d = json.load(f)
                self.kp_edit.setText(str(d['kp']))
                self.ki_edit.setText(str(d['ki']))
                self.kd_edit.setText(str(d['kd']))
                self.update_pid()
            except Exception as e:
                print("Load error:", e)

    # --- Motor ---
    def run_motor(self):
        with self.buffer.lock:
            self.buffer.running = True
        self._publish_command()

    def stop_motor(self):
        with self.buffer.lock:
            self.buffer.running = False
        self._publish_command()

    def timed_run(self):
        try:
            t = float(self.time_edit.text())
            if t > 0:
                with self.buffer.lock:
                    self.buffer.running = True
                self._publish_command(is_timed_run=True, run_time=t)
        except:
            print("Invalid time")

    # --- Plot ---
    def open_plot(self, plot_type):
        win = PlotWindow(f'{plot_type.capitalize()} Plot', self.buffer, plot_type)
        win.show()
        self.plot_windows.append(win)
