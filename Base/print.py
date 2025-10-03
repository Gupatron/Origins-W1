import time
import json

config = json.load(open('config.json'))

def print_thread(buffer, stop_event, publisher):
    dt = 0.1  # Fixed logging frequency
    current_time = 0.0
    max_len = int(config['plot_window_time'] / dt) + 1
    while not stop_event.is_set():
        with buffer.lock:
            buffer.times.append(current_time)
            buffer.omega_ds.append(buffer.omega_d)
            buffer.omega_rpms.append(buffer.omega_rpm)
            buffer.omega_filteredrpms.append(buffer.omega_filteredrpm)
            buffer.errors.append(buffer.error)
            buffer.duties.append(buffer.u_duty)
            if len(buffer.times) > max_len:
                buffer.times = buffer.times[-max_len:]
                buffer.omega_ds = buffer.omega_ds[-max_len:]
                buffer.omega_rpms = buffer.omega_rpms[-max_len:]
                buffer.omega_filteredrpms = buffer.omega_filteredrpms[-max_len:]
                buffer.errors = buffer.errors[-max_len:]
                buffer.duties = buffer.duties[-max_len:]
            print(f"Time: {current_time:.1f}, Desired: {buffer.omega_d:.1f}, RPM: {buffer.omega_rpm:.1f}, Filtered: {buffer.omega_filteredrpm:.1f}, Error: {buffer.error:.1f}, Duty: {buffer.u_duty:.4f}")
            telemetry = {
                'time': current_time,
                'omega_d': buffer.omega_d,
                'omega_rpm': buffer.omega_rpm,
                'omega_filteredrpm': buffer.omega_filteredrpm,
                'error': buffer.error,
                'u_duty': buffer.u_duty,
                'omega_erpm': buffer.omega_erpm
            }
            publisher.put(json.dumps(telemetry))
        current_time += dt
        time.sleep(dt)