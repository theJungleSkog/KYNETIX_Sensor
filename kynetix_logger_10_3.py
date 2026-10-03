"""
KYNETIX sensor logger: records the ESP32's output to a CSV file, shows a live graph,
and saves a picture of the WHOLE run when you stop.

SETUP (once, in a Windows terminal / Command Prompt):
    py -m pip install pyserial matplotlib

ON THE ESP32: set PLOT_MODE = false and CSV_MODE = true, then upload.
(It also understands PLOT_MODE = true output, but CSV mode records more columns.)

RUN:
    1. CLOSE the Arduino Serial Monitor and Serial Plotter (only one program can use the port).
    2. In a terminal, in the folder with this file:
           py kynetix_logger.py COM3
    3. Press Enter in the terminal at any time to re-zero (same as typing b).
    4. Stop with Ctrl+C or by closing the graph window.

YOU GET (in the same folder, named with the date and time):
    kynetix_YYYYMMDD_HHMMSS.csv   every reading, opens in Excel
    kynetix_YYYYMMDD_HHMMSS.png   graph of the whole run, re-zero moments marked
"""

import csv
import sys
import threading
import time
from datetime import datetime

import serial  # from the pyserial package
import matplotlib.pyplot as plt

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
BAUD = 115200
LIVE_WINDOW_S = 30          # live graph shows the last 30 seconds
REDRAW_EVERY_S = 0.25       # how often the live graph refreshes


def parse_line(line):
    """Turn one line from the ESP32 into a dict, or None if it isn't a reading."""
    line = line.strip()
    if not line:
        return None
    # PLOT_MODE line: change_pF:0.0123,top:0.20,bottom:-0.20
    if line.startswith("change_pF:"):
        try:
            return {"delta_pF": float(line.split(",")[0].split(":")[1])}
        except (ValueError, IndexError):
            return None
    # CSV_MODE line: time_s,ticks,delta_ticks,cap_pF,delta_pF,timeouts
    parts = line.split(",")
    if len(parts) == 6 and parts[1] != "TIMEOUT":
        try:
            device_t, ticks, dticks, cap, dpf, tmo = parts
            return {"device_time_s": float(device_t), "ticks": float(ticks),
                    "delta_ticks": float(dticks), "cap_pF": float(cap),
                    "delta_pF": float(dpf), "timeouts": int(tmo)}
        except ValueError:
            return None
    return None


def main():
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path, png_path = f"kynetix_{stamp}.csv", f"kynetix_{stamp}.png"
    columns = ["time_s", "delta_pF", "cap_pF", "ticks", "delta_ticks", "timeouts", "event"]

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)
    except serial.SerialException as e:
        print(f"Could not open {PORT}: {e}")
        print("Close the Arduino Serial Monitor/Plotter, check the COM number, and try again.")
        return

    print(f"Logging {PORT} to {csv_path}")
    print("Press Enter to re-zero. Ctrl+C or close the graph window to stop.\n")

    t_start = time.monotonic()
    times, values, rezero_times = [], [], []
    running = True
    lock = threading.Lock()
    pending_events = []

    def keyboard():
        # Enter in the terminal -> send 'b' to the ESP32 (re-zero)
        while running:
            try:
                input()
            except EOFError:
                return
            if not running:
                return
            ser.write(b"b\n")
            with lock:
                pending_events.append(time.monotonic() - t_start)
            print("  re-zero sent")

    threading.Thread(target=keyboard, daemon=True).start()

    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 4.5))
    (live_line,) = ax.plot([], [], color="#2a78d6", lw=1.5)
    ax.axhline(0, color="#999999", lw=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Change in capacitance (pF)")
    ax.set_title("KYNETIX sensor (live)")
    ax.grid(alpha=0.3)
    last_draw = 0.0

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        try:
            while plt.fignum_exists(fig.number):
                raw = ser.readline().decode(errors="ignore")
                now = time.monotonic() - t_start

                with lock:
                    events, pending_events[:] = list(pending_events), []
                for ev_t in events:
                    rezero_times.append(ev_t)
                    writer.writerow({"time_s": f"{ev_t:.3f}", "event": "re-zero"})

                if raw.startswith("Baseline:"):
                    print("  " + raw.strip())
                reading = parse_line(raw)
                if reading:
                    times.append(now)
                    values.append(reading["delta_pF"])
                    row = {"time_s": f"{now:.3f}"}
                    row.update({k: reading.get(k, "") for k in columns[1:6]})
                    writer.writerow(row)

                if now - last_draw > REDRAW_EVERY_S and times:
                    live_line.set_data(times, values)
                    ax.set_xlim(max(0, now - LIVE_WINDOW_S), max(LIVE_WINDOW_S, now))
                    shown = [v for t, v in zip(times, values) if t >= now - LIVE_WINDOW_S]
                    lo, hi = min(shown + [-0.05]), max(shown + [0.05])
                    pad = 0.1 * (hi - lo)
                    ax.set_ylim(lo - pad, hi + pad)
                    plt.pause(0.001)
                    f.flush()
                    last_draw = now
        except KeyboardInterrupt:
            pass
        finally:
            running = False
            ser.close()

    plt.ioff()
    plt.close("all")
    if not times:
        print("No readings were recorded. Is the ESP32 in CSV_MODE or PLOT_MODE?")
        return

    # Graph of the whole run
    fig, ax = plt.subplots(figsize=(12, 4.5))
    ax.plot(times, values, color="#2a78d6", lw=1.2)
    ax.axhline(0, color="#999999", lw=0.8)
    for i, rt in enumerate(rezero_times):
        ax.axvline(rt, color="#999999", ls="--", lw=0.8,
                   label="re-zero" if i == 0 else None)
    if rezero_times:
        ax.legend(loc="upper left", frameon=False)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Change in capacitance (pF)")
    ax.set_title(f"KYNETIX sensor run {stamp} ({len(times)} readings)", loc="left")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    print(f"\nSaved {len(times)} readings to {csv_path}")
    print(f"Saved graph of the whole run to {png_path}")


if __name__ == "__main__":
    main()
