import threading
import signal
import sys
import json
from PyQt6.QtWidgets import QApplication
from databuffer import DataBuffer
from vese import vese_thread
from kalmanfilter import kalman_thread
from pid import pid_thread
from print import print_thread
from gui import GUI

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
        stop_event = threading.Event()
        threads = [
            threading.Thread(target=vese_thread, args=(buffer, stop_event)),
            threading.Thread(target=kalman_thread, args=(buffer, stop_event)),
            threading.Thread(target=pid_thread, args=(buffer, stop_event)),
            threading.Thread(target=print_thread, args=(buffer, stop_event))
        ]
        for t in threads:
            t.start()

        def sigint_handler(sig, frame):
            stop_event.set()
            for t in threads:
                t.join()
            sys.exit(0)

        signal.signal(signal.SIGINT, sigint_handler)
        app = QApplication(sys.argv)
        gui = GUI(buffer)
        gui.show()
        sys.exit(app.exec())
    except Exception as e:
        print(f"Main error: {e}")
    finally:
        stop_event.set()
        for t in threads:
            t.join()

if __name__ == "__main__":
    main()