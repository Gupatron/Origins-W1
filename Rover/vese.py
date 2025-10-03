#!/usr/bin/env python3
import time
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
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
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
            ser.write(encode(SetDutyCycle(int(val * 100000))))
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
                    duty = buffer.u_duty if running else 0.0
                if running and not last_running:
                    ramp_duty(ser, last_duty, duty, config['ramp_time_s'])
                elif not running and last_running:
                    ramp_duty(ser, last_duty, 0.0, config['ramp_time_s'])
                else:
                    try:
                        ser.write(encode(SetDutyCycle(int(duty * 100000))))
                        ser.flush()
                    except serial.SerialException as e:
                        print(f"Serial error: {e}")
                ser.reset_input_buffer()
                try:
                    ser.write(pack_comm(b"\x04"))
                    ser.flush()
                    time.sleep(0.1)  # Increased sleep for better reliability
                    buf = ser.read(512)
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
                time.sleep(max(0, dt - 0.1))
    except Exception as e:
        print(f"VESC thread error: {e}")
