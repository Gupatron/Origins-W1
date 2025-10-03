import threading
import signal
import sys
import json
import time
import zenoh

from databuffer import DataBuffer
from PythonArduinoServo_copy import servo_thread
try:
    from vese import vese_thread
    from kalmanfilter import kalman_thread
    from pid import pid_thread
    from print import print_thread
except Exception:
    vese_thread = kalman_thread = pid_thread = print_thread = None

COMMAND_KEY = 'f1tenth_pid/command'
TELEMETRY_KEY = 'f1tenth_pid/telemetry'

def main():
    stop_event = threading.Event()
    threads = []
    session = None
    publisher = None
    try:
        config = json.load(open('config.json'))
        buffer = DataBuffer()
        buffer.use_filtered_on = config.get('use_filtered_on', False)
        # NEW: Initialize new buffer fields for bypass mode
        buffer.bypass_pid = False
        buffer.direct_duty = 0.0

        try:
            with open('gain_default.json') as f:
                gains = json.load(f)
            with buffer.lock:
                buffer.kp = float(gains.get('kp', 0.0))
                buffer.ki = float(gains.get('ki', 0.0))
                buffer.kd = float(gains.get('kd', 0.0))
        except FileNotFoundError:
            print("Default gains file not found, using zero gains")

        zenoh_config = zenoh.Config()
        session = zenoh.open(zenoh_config)
        publisher = session.declare_publisher(TELEMETRY_KEY)

        def subscriber_callback(sample):
            try:
                payload = sample.payload.to_bytes().decode('utf-8')
                data = json.loads(payload)

                with buffer.lock:
                    # PID/motor fields
                    if 'kp' in data: buffer.kp = float(data['kp'])
                    if 'ki' in data: buffer.ki = float(data['ki'])
                    if 'kd' in data: buffer.kd = float(data['kd'])
                    if 'omega_d' in data: buffer.omega_d = float(data['omega_d'])
                    if 'running' in data: buffer.running = bool(data['running'])
                    
                    # NEW: Handle PID bypass commands from GUI
                    if 'bypass_pid' in data: buffer.bypass_pid = bool(data['bypass_pid'])
                    if 'direct_duty' in data: buffer.direct_duty = float(data['direct_duty'])

                    if data.get('timed_run'):
                        buffer.running = True
                        def stop_motor():
                            with buffer.lock:
                                buffer.running = False
                        timer = threading.Timer(float(data.get('run_time', 0.0)), stop_motor)
                        timer.start()

                    # Servo controls
                    if 'servo_angle' in data:
                        buffer.servo_angle = int(data['servo_angle'])
                    if 'servo_offset' in data:
                        buffer.servo_offset = int(data['servo_offset'])
                    if 'servo_jog' in data:
                        try: jog = int(data['servo_jog'])
                        except Exception: jog = 0
                        buffer.servo_angle = int(buffer.servo_angle) + jog

            except json.JSONDecodeError as e:
                print(f"Error decoding JSON: {e}")
            except Exception as e:
                print(f"Error processing subscriber data: {e}")

        session.declare_subscriber(COMMAND_KEY, subscriber_callback)

        if vese_thread is not None:
            threads.append(threading.Thread(target=vese_thread, args=(buffer, stop_event), daemon=True))
        if kalman_thread is not None:
            threads.append(threading.Thread(target=kalman_thread, args=(buffer, stop_event), daemon=True))
        if pid_thread is not None:
            threads.append(threading.Thread(target=pid_thread, args=(buffer, stop_event), daemon=True))
        if print_thread is not None:
            threads.append(threading.Thread(target=print_thread, args=(buffer, stop_event, publisher), daemon=True))

        threads.append(threading.Thread(target=servo_thread, args=(buffer, stop_event), daemon=True))

        for t in threads:
            t.start()

        def sigint_handler(sig, frame):
            stop_event.set()
            for t in threads: t.join(timeout=2.0)
            if session is not None:
                try: session.close()
                except Exception: pass
            sys.exit(0)

        signal.signal(signal.SIGINT, sigint_handler)

        while not stop_event.is_set():
            time.sleep(0.5)

    except Exception as e:
        print(f"Rover error: {e}")
    finally:
        stop_event.set()
        for t in threads:
            try: t.join(timeout=2.0)
            except Exception: pass
        if session is not None:
            try: session.close()
            except Exception: pass

if __name__ == "__main__":
    main()
