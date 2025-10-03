import sys
import json
import zenoh
from PyQt6.QtWidgets import QApplication
from databuffer import DataBuffer
from gui import GUI

def main():
    try:
        # Initialize Zenoh session
        config = zenoh.Config()
        session = zenoh.open(config)
        buffer = DataBuffer()

        def subscriber_callback(sample):
            try:
                data = json.loads(sample.payload.decode('utf-8'))
                with buffer.lock:
                    buffer.times.append(data['time'])
                    buffer.omega_ds.append(data['omega_d'])
                    buffer.omega_rpms.append(data['omega_rpm'])
                    buffer.omega_filteredrpms.append(data['omega_filteredrpm'])
                    buffer.errors.append(data['error'])
                    buffer.duties.append(data['u_duty'])
                    buffer.omega_erpm = data['omega_erpm']
                    # Trim buffers to plot_window_time
                    dt = 0.1
                    max_len = int(json.load(open('config.json'))['plot_window_time'] / dt) + 1
                    if len(buffer.times) > max_len:
                        buffer.times = buffer.times[-max_len:]
                        buffer.omega_ds = buffer.omega_ds[-max_len:]
                        buffer.omega_rpms = buffer.omega_rpms[-max_len:]
                        buffer.omega_filteredrpms = buffer.omega_filteredrpms[-max_len:]
                        buffer.errors = buffer.errors[-max_len:]
                        buffer.duties = buffer.duties[-max_len:]
            except Exception as e:
                print(f"Error processing subscriber data: {e}")

        # Subscribe to telemetry data
        session.declare_subscriber('f1tenth_pid', subscriber_callback)

        # Start GUI
        app = QApplication(sys.argv)
        gui = GUI(buffer, session)
        gui.show()
        sys.exit(app.exec())
    except Exception as e:
        print(f"Base error: {e}")
    finally:
        session.close()

if __name__ == "__main__":
    main()