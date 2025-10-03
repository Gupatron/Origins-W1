#!/usr/bin/env python3
import time
import numpy as np
import glob
import serial
import threading
import json
from pyvesc import SetDutyCycle, encode

config = json.load(open('config.json'))

BAUD = 115200

def _crc16_xmodem(data: bytes) -> int:
    crc = 0x0000
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if (crc & 0x8000) != 0:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def pack_comm(payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("Use long frame for payload >255")
    start = b"\x02"
    length = bytes([len(payload)])
    crc = _crc16_xmodem(payload)
    crc_bytes = bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    end = b"\x03"
    return start + length + payload + crc_bytes + end

def pick_port():
    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    print(f"Available ports: {ports}")  # Debug print
    if not ports:
        raise SystemExit("No VESC serial ports found.")
    return ports[0]

def ramp_duty(ser, start, stop, ramp_time):
    steps = max(1, int(ramp_time / 0.05))
    for i in range(steps + 1):
        val = start + (stop - start) * (i / steps)
        try:
            ser.write(encode(SetDutyCycle(int(float(np.clip(val, -1.0, 1.0)) * 100000))))
            ser.flush()
            time.sleep(0.05)
        except serial.SerialException as e:
            print(f"Serial error during ramp: {e}")
            break

def vese_thread(buffer, stop_event):
    try:
        port = pick_port()
        dt = 1 / config['vese_frequency']
        with serial.Serial(port, BAUD, timeout=0.2) as ser:  # Increased timeout
            last_running = False
            last_duty = 0.0

            while not stop_event.is_set():
                with buffer.lock:
                    running = buffer.running
                    duty = float(np.clip(buffer.u_duty, -1.0, 1.0))  # hard clamp

                # Optional soft-start/stop on transitions
                if running and not last_running:
                    # ramp from last to current over configured time
                    ramp_time_s = config.get('duty_ramp_time_s', 0.0)
                    if ramp_time_s > 0:
                        ramp_duty(ser, last_duty, duty, ramp_time_s)
                    else:
                        try:
                            ser.write(encode(SetDutyCycle(int(float(np.clip(duty, -1.0, 1.0)) * 100000))))
                            ser.flush()
                        except serial.SerialException as e:
                            print(f"Serial error: {e}")
                elif not running and last_running:
                    # on stop, ramp to zero if configured
                    ramp_time_s = config.get('stop_ramp_time_s', 0.0)
                    if ramp_time_s > 0:
                        ramp_duty(ser, last_duty, 0.0, ramp_time_s)
                    else:
                        try:
                            ser.write(encode(SetDutyCycle(0)))
                            ser.flush()
                        except serial.SerialException as e:
                            print(f"Serial error: {e}")
                else:
                    # steady state
                    try:
                        ser.write(encode(SetDutyCycle(int(float(np.clip(duty, -1.0, 1.0)) * 100000))))
                        ser.flush()
                    except serial.SerialException as e:
                        print(f"Serial error: {e}")

                # Read a telemetry frame (example parsing—as in your original)
                try:
                    ser.write(pack_comm(b'\x04'))  # hypothetical "get values" cmd
                    time.sleep(0.01)               # small delay
                    buf = ser.read(128)            # read some bytes
                    if buf and len(buf) >= 29:
                        rpm_bytes = buf[25:29]
                        omega_erpm = int.from_bytes(rpm_bytes, 'big', signed=True)
                        with buffer.lock:
                            buffer.omega_erpm = omega_erpm
                            buffer.omega_rpm = omega_erpm / config['wheel_rpm_const']
                    else:
                        print(f"No sufficient data received from VESC (len: {len(buf) if buf else 0})")
                except serial.SerialException as e:
                    print(f"Serial read error: {e}")

                last_running = running
                last_duty = duty
                time.sleep(dt)
    except Exception as e:
        print(f"VESC thread error: {e}")
