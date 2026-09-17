#!/usr/bin/env python3
"""
Drive lidar-bot from a Bluetooth gamepad. No ROS needed.

Usage:
    sudo apt install -y python3-evdev python3-serial
    python3 gamepad_drive.py                # auto-detect the pad
    python3 gamepad_drive.py --list         # show input devices
    python3 gamepad_drive.py --event 3      # force /dev/input/event3
"""
import sys, time, json, glob, math, serial
from evdev import InputDevice, ecodes, list_devices

PORT       = "/dev/ttyAMA0"
BAUD       = 115200
RATE_HZ    = 20

MAX_LIN    = 0.25      # m/s at full stick (boost)
MAX_ANG    = 1.6       # rad/s at full stick (boost)
NORMAL     = 0.5       # scale when not boosting
TRACK_W    = 0.21
DEADZONE   = 0.12

DEADMAN    = ecodes.BTN_TR      # right bumper
BOOST      = ecodes.BTN_TL      # left bumper
QUIT_BTN   = ecodes.BTN_START

def find_pad():
    for path in list_devices():
        d = InputDevice(path)
        caps = d.capabilities()
        if ecodes.EV_ABS in caps and ecodes.EV_KEY in caps:
            keys = [k for k, _ in caps[ecodes.EV_ABS]]
            if ecodes.ABS_X in keys and ecodes.ABS_Y in keys:
                return d
    return None

def norm(val, info):
    """scale a raw axis to -1..1 with a deadzone"""
    mid = (info.max + info.min) / 2.0
    half = (info.max - info.min) / 2.0 or 1
    v = (val - mid) / half
    if abs(v) < DEADZONE:
        return 0.0
    return (abs(v) - DEADZONE) / (1 - DEADZONE) * (1 if v > 0 else -1)

def main():
    if "--list" in sys.argv:
        for p in list_devices():
            d = InputDevice(p)
            print(f"{p}  {d.name}")
        return

    if "--event" in sys.argv:
        dev = InputDevice("/dev/input/event" + sys.argv[sys.argv.index("--event") + 1])
    else:
        dev = find_pad()

    if dev is None:
        print("No gamepad found. Pair it first, then try --list.")
        return

    print(f"Gamepad: {dev.name}")
    abs_info = {c: i for c, i in dev.capabilities()[ecodes.EV_ABS]}

    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    time.sleep(0.2)

    axes = {ecodes.ABS_Y: 0.0, ecodes.ABS_RX: 0.0}
    held = set()
    last_send = 0.0
    period = 1.0 / RATE_HZ

    def stop():
        ser.write((json.dumps({"T": 1, "L": 0, "R": 0}) + "\n").encode())
        ser.flush()

    print("Hold the RIGHT BUMPER to drive. START quits.\n")
    dev.grab()

    try:
        while True:
            try:
                for ev in dev.read():
                    if ev.type == ecodes.EV_ABS and ev.code in axes:
                        axes[ev.code] = norm(ev.value, abs_info[ev.code])
                    elif ev.type == ecodes.EV_KEY:
                        if ev.value:
                            held.add(ev.code)
                        else:
                            held.discard(ev.code)
                        if ev.code == QUIT_BTN and ev.value:
                            raise KeyboardInterrupt
            except BlockingIOError:
                pass

            now = time.time()
            if now - last_send < period:
                time.sleep(0.002)
                continue
            last_send = now

            if DEADMAN in held:
                scale = 1.0 if BOOST in held else NORMAL
                v = -axes[ecodes.ABS_Y] * MAX_LIN * scale     # stick up = forward
                w = -axes[ecodes.ABS_RX] * MAX_ANG * scale    # stick right = clockwise
                l = v - (w * TRACK_W / 2.0)
                r = v + (w * TRACK_W / 2.0)
            else:
                v = w = l = r = 0.0

            ser.write((json.dumps({"T": 1, "L": round(l, 3), "R": round(r, 3)}) + "\n").encode())
            ser.flush()

            bar = "#" * int(abs(v) / MAX_LIN * 20)
            print(f"\r{'DRIVE' if DEADMAN in held else ' idle'}  "
                  f"v={v:+.2f}  w={w:+.2f}  L={l:+.2f} R={r:+.2f}  {bar:<20}", end="")

    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping.")
        for _ in range(3):
            stop()
            time.sleep(0.05)
        try:
            dev.ungrab()
        except Exception:
            pass

if __name__ == "__main__":
    main()
