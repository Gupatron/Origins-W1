import threading
import signal
import sys
import json
import zenoh
from databuffer import DataBuffer
from vese import vese_thread
from kalmanfilter import kalman_thread
from pid import pid_thread
from print import print_thread

COMMAND_KEY = 'f1tenth_pid/command'
TELEMETRY_KEY = 'f1tenth_pid/telemetry'

def main():
    try:
        config = json.load(open('config.json'))
        buffer = DataBuffer()
        buffer.use_filtered_on = config['use_filtered_on']
        # Load default PID gains
        try:
            with open('gain_default.json') as f:
                gains = json.load(f)
                with buffer.lock:
                    buffer.kp = float(gains['kp'])
                    buffer.ki = float(gains['ki'])
                    buffer.kd = float(gains['kd'])
        except FileNotFoundError:
            print("Default gains file not found, using zero gains")
        
        # Initialize Zenoh session
        zenoh_config = zenoh.Config()
        session = zenoh.open(zenoh_config)
        publisher = session.declare_publisher(TELEMETRY_KEY)

        def subscriber_callback(sample):
            try:
                # Decode and parse JSON
                payload = sample.payload.to_bytes().decode('utf-8')
                data = json.loads(payload)
                # Validate required keys
                required_keys = ['kp', 'ki', 'kd', 'omega_d', 'running', 'timed_run', 'run_time']
                if not all(key in data for key in required_keys):
                    missing = [key for key in required_keys if key not in data]
                    print(f"Missing keys in received JSON: {missing}")
                    return
                with buffer.lock:
                    buffer.kp = float(data['kp'])
                    buffer.ki = float(data['ki'])
                    buffer.kd = float(data['kd'])
                    buffer.omega_d = float(data['omega_d'])
                    buffer.running = bool(data['running'])
                    if data['timed_run']:
                        buffer.running = True
                        # Schedule stop after run_time
                        def stop_motor():
                            with buffer.lock:
                                buffer.running = False
                        timer = threading.Timer(float(data['run_time']), stop_motor)
                        timer.start()
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON: {e}")
            except Exception as e:
                print(f"Error processing subscriber data: {e}")

        # Subscribe to GUI commands
        session.declare_subscriber(COMMAND_KEY, subscriber_callback)

        stop_event = threading.Event()
        threads = [
            threading.Thread(target=vese_thread, args=(buffer, stop_event)),
            threading.Thread(target=kalman_thread, args=(buffer, stop_event)),
            threading.Thread(target=pid_thread, args=(buffer, stop_event)),
            threading.Thread(target=print_thread, args=(buffer, stop_event, publisher))
        ]
        for t in threads:
            t.start()

        def sigint_handler(sig, frame):
            stop_event.set()
            for t in threads:
                t.join()
            session.close()
            sys.exit(0)

        signal.signal(signal.SIGINT, sigint_handler)
        
        # Keep the main thread alive
        while not stop_event.is_set():
            threading.Event().wait(1)
    except Exception as e:
        print(f"Rover error: {e}")
    finally:
        stop_event.set()
        for t in threads:
            t.join()
        session.close()

if __name__ == "__main__":
    main()