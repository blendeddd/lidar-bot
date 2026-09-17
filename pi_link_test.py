#!/usr/bin/env python3
"""
Verify the Pi <-> ESP32 link through the 40-pin hat.

  python3 pi_link_test.py              # listen and decode the feedback stream
  python3 pi_link_test.py --status     # ask for the full status dump (T:710)
  python3 pi_link_test.py --config     # read the saved robot geometry (T:701)
  python3 pi_link_test.py --drive      # nudge both wheels for 1s, then stop

Prereqs on the Pi (once):
  /boot/firmware/config.txt   -> enable_uart=1   and   dtoverlay=disable-bt
  /boot/firmware/cmdline.txt  -> remove console=serial0,115200 (keep the rest on one line)
  sudo usermod -aG dialout $USER   then reboot
"""
import sys, json, time, serial

PORT = "/dev/serial0"
BAUD = 115200

def send(ser, obj):
    ser.write((json.dumps(obj) + "\n").encode())
    ser.flush()

def read_json(ser, timeout=3.0):
    """Return the next decodable JSON line, ignoring noise."""
    end = time.time() + timeout
    while time.time() < end:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        try:
            return json.loads(line)
        except ValueError:
            continue
    return None

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(0.3)
    ser.reset_input_buffer()

    if "--status" in sys.argv:
        send(ser, {"T": 710})
        r = read_json(ser)
        if not r:
            print("No reply. Check wiring, baud, and that the console is off the UART.")
            return
        print(json.dumps(r, indent=2))
        return

    if "--config" in sys.argv:
        send(ser, {"T": 701})
        r = read_json(ser)
        print(json.dumps(r, indent=2) if r else "No reply.")
        return

    if "--drive" in sys.argv:
        print("Driving 0.1 m/s for 1s...")
        send(ser, {"T": 1, "L": 0.1, "R": 0.1})
        time.sleep(1.0)
        send(ser, {"T": 1, "L": 0, "R": 0})
        print("Stopped.")
        return

    # default: watch the stream
    print(f"Listening on {PORT} at {BAUD}. Ctrl-C to stop.\n")
    good = bad = 0
    t0 = time.time()
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        try:
            d = json.loads(line)
            good += 1
        except ValueError:
            bad += 1
            continue

        if good % 10 == 0:
            hz = good / max(0.001, time.time() - t0)
            print(f"[{hz:5.1f} Hz  ok={good} bad={bad}]  "
                  f"odl={d.get('odl'):>8}  odr={d.get('odr'):>8}  "
                  f"L={d.get('L', 0):6.3f}  R={d.get('R', 0):6.3f}  "
                  f"yaw={d.get('y', 0):7.2f}  "
                  f"pack={d.get('pv', 0):5.2f}V  low={d.get('lowBat')}")

if __name__ == "__main__":
    try:
        main()
    except serial.SerialException as e:
        print(f"Serial error: {e}\nIs /dev/serial0 present? Is the console still using it?")
    except KeyboardInterrupt:
        pass
