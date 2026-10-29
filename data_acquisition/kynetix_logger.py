"""KYNETIX logger companion script.

Keys (no Enter needed):
  b = zero sensors + start logging
  e = end logging
  m = save the log to a CSV file
  q = quit

Setup:  pip install pyserial
Run:    python kynetix_logger.py            (auto-detects the XIAO)
        python kynetix_logger.py COM5       (or give the port explicitly)
Close the Arduino Serial Monitor first, only one program can use the port.
"""
import sys
import threading
import time
from datetime import datetime

import serial
import serial.tools.list_ports

if sys.platform == "win32":
    import msvcrt

    def getch():
        return msvcrt.getwch()
else:
    import termios
    import tty

    def getch():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def find_port():
    if len(sys.argv) > 1:
        return sys.argv[1]
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid in (0x2886, 0x239A):  # Seeed / Adafruit USB vendor IDs
            return p.device
    if ports:
        print("Could not identify the XIAO, using", ports[0].device)
        return ports[0].device
    sys.exit("No serial ports found. Is the board plugged in?")


port = find_port()
ser = serial.Serial(port, 115200, timeout=0.1)
print(f"Connected on {port}.  b = start, e = end, m = save, q = quit")

rows = []          # data lines (header + samples)
lock = threading.Lock()
running = True
last_count_print = 0.0


def reader():
    global last_count_print
    buf = b""
    while running:
        try:
            buf += ser.read(4096)
        except serial.SerialException:
            print("\nSerial connection lost.")
            return
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            if line.startswith("#"):
                print(line)
            else:
                with lock:
                    rows.append(line)
                    n = len(rows) - 1  # minus header
                now = time.time()
                if now - last_count_print > 1.0:
                    last_count_print = now
                    print(f"  ...{n} samples received", end="\r")


threading.Thread(target=reader, daemon=True).start()

try:
    while True:
        k = getch().lower()
        if k == "b":
            with lock:
                rows.clear()
            ser.write(b"b")
        elif k in ("e", "n"):
            ser.write(b"e")
            time.sleep(0.3)  # let the last lines arrive
        elif k == "m":
            with lock:
                data = list(rows)
            n = max(len(data) - 1, 0)
            if n == 0:
                print("Nothing to save yet.")
                continue
            name = datetime.now().strftime("kynetix_log_%Y%m%d_%H%M%S.csv")
            with open(name, "w") as f:
                f.write("\n".join(data) + "\n")
            print(f"Saved {n} samples to {name}")
        elif k == "q":
            break
finally:
    running = False
    ser.close()
