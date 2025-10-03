
"""
Servo control thread for Jetson Nano.

Reads desired servo targets from DataBuffer and transmits them over a serial link
(e.g., to an Arduino that actually generates the PWM). Runs in its own thread.

- Accepts either absolute degree input (buffer.servo_angle) or "offset" input
  (buffer.servo_offset in range [-40, +40]) which is mapped to a degree range.

This keeps the GUI and transport decoupled: GUI publishes servo_angle or jogs,
the rover process updates the DataBuffer via Zenoh, and this thread streams
changes to the micro that drives the servo.

If you do not use an external micro and want the Jetson to drive PWM directly,
you can replace the SerialDriver class with a GPIO driver (Jetson.GPIO or pigpio).
"""

import time
import serial

# ====== Configurable ranges (keep in sync with your hardware) ======
OFFSET_MIN = -40
OFFSET_MAX =  40

SERVO_MIN  =  50   # degrees (as understood by your MCU)
SERVO_MAX  = 130   # degrees
# ===================================================================

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def map_range(x, in_min, in_max, out_min, out_max):
    if in_max == in_min:
        raise ValueError("Input range cannot be zero width")
    frac = (x - in_min) / (in_max - in_min)
    return out_min + frac * (out_max - out_min)

def offset_to_angle(offset):
    o = clamp(int(offset), OFFSET_MIN, OFFSET_MAX)
    angle = map_range(o, OFFSET_MIN, OFFSET_MAX, SERVO_MIN, SERVO_MAX)
    return int(round(clamp(angle, SERVO_MIN, SERVO_MAX)))

class SerialDriver:
    def __init__(self, port="/dev/ttyUSB0", baud=230400):
        # Open lazily and retry if the USB device appears late
        self.port = port
        self.baud = baud
        self.ser = None
        self._ensure_open()

    def _ensure_open(self):
        if self.ser is not None:
            return
        tries = 0
        while self.ser is None and tries < 10:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            except Exception as e:
                print(f"[servo] Serial open failed ({e}); retrying...")
                time.sleep(0.5)
                tries += 1
        if self.ser:
            # give the MCU a moment to reset if applicable
            time.sleep(1.5)

    def send_angle(self, angle_deg: int):
        self._ensure_open()
        if not self.ser:
            return
        # transmit as single byte (0..255). Your 50..130 range fits in one byte.
        try:
            self.ser.write(bytes([int(angle_deg) & 0xFF]))
        except Exception as e:
            print(f"[servo] Serial write error: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None  # force reopen on next call

    def close(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass

def servo_thread(buffer, stop_event, serial_port="/dev/ttyUSB0", baud=230400):
    driver = SerialDriver(serial_port, baud)
    last_sent = None
    last_offset_seen = None
    try:
        while not stop_event.is_set():
            with buffer.lock:
                # Prefer absolute angle if it's been set; otherwise map offset.
                desired_angle = int(buffer.servo_angle)
                # Track and map offset if GUI uses offsets
                if buffer.servo_offset != last_offset_seen:
                    mapped = offset_to_angle(buffer.servo_offset)
                    # If the GUI only updates offset, keep angle in sync
                    desired_angle = mapped
                    buffer.servo_angle = mapped
                    last_offset_seen = buffer.servo_offset

                desired_angle = clamp(desired_angle, SERVO_MIN, SERVO_MAX)

            if desired_angle != last_sent:
                driver.send_angle(desired_angle)
                last_sent = desired_angle
                # Optional: store a tiny history for plotting/debug
                with buffer.lock:
                    buffer.servo_angles.append(desired_angle)
                    buffer.times.append(time.time())

            time.sleep(0.02)  # 50 Hz update loop
    finally:
        driver.close()


