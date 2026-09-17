#!/usr/bin/env python3
"""
Car-style driving for lidar-bot, with live PID tuning from the pad.

  RT / LT ............... forward / reverse throttle
  Left stick X .......... steering
  A ..................... arm / disarm
  B ..................... emergency stop + disarm
  X ..................... flip steering direction
  Y ..................... reset settings to defaults
  LB (hold) ............. boost
  D-pad up/down ......... max speed  -/+
  D-pad left/right ...... turn sharpness  -/+
  RB (hold) + D-pad ..... PID:  up/down = P,  left/right = I
  SELECT ................ save PID gains to the board's flash
  START ................. quit (settings saved locally)
"""
import sys, os, time, json, serial
from evdev import InputDevice, ecodes, list_devices

PORT, BAUD, RATE_HZ = "/dev/ttyAMA0", 115200, 25
SETTINGS = os.path.expanduser("~/.lidarbot_drive.json")

DEFAULTS = {
    "max_lin":   0.45,   # m/s at full trigger, boosted
    "turn_gain": 0.60,   # 0..1 - how much the inner wheel slows in a turn
    "pivot":     0.30,   # m/s wheel speed when spinning on the spot
    "normal":    0.60,   # scale when not holding boost
    "steer_inv": True,
}
STEP = {"max_lin": 0.05, "turn_gain": 0.05}
LIMITS = {"max_lin": (0.05, 1.00), "turn_gain": (0.05, 0.95)}
P_STEP, I_STEP = 25.0, 50.0

PIVOT_BELOW = 0.06       # |v| under this = pivot instead of arc
DEADZONE, TRIG_DEAD = 0.12, 0.06

ARM, STOP, FLIP, RESET = ecodes.BTN_SOUTH, ecodes.BTN_EAST, ecodes.BTN_WEST, ecodes.BTN_NORTH
BOOST, PIDMOD, SAVE, QUIT = ecodes.BTN_TL, ecodes.BTN_TR, ecodes.BTN_SELECT, ecodes.BTN_START
TRIG_R, TRIG_L = [ecodes.ABS_GAS, ecodes.ABS_RZ], [ecodes.ABS_BRAKE, ecodes.ABS_Z]
HAT_X, HAT_Y = ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y


def find_pad():
    for p in list_devices():
        d = InputDevice(p)
        c = d.capabilities()
        if ecodes.EV_ABS in c and ecodes.EV_KEY in c and \
           ecodes.ABS_X in [x for x, _ in c[ecodes.EV_ABS]]:
            return d
    return None


def pick(dev, wanted):
    have = [c for c, _ in dev.capabilities()[ecodes.EV_ABS]]
    return next((w for w in wanted if w in have), None)


def load_cfg():
    cfg = dict(DEFAULTS)
    try:
        cfg.update(json.load(open(SETTINGS)))
    except Exception:
        pass
    return cfg


def query(ser, cmd, key, timeout=1.5):
    """send a json cmd and return the first reply containing key"""
    ser.write((json.dumps(cmd) + "\n").encode())
    ser.flush()
    end = time.time() + timeout
    while time.time() < end:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            d = json.loads(line)
            if key in d:
                return d
        except Exception:
            continue
    return None


def main():
    if "--list" in sys.argv:
        for p in list_devices():
            print(f"{p}  {InputDevice(p).name}")
        return

    dev = find_pad()
    if dev is None:
        print("No gamepad found.")
        return

    info = {c: i for c, i in dev.capabilities()[ecodes.EV_ABS]}
    ax_s, ax_rt, ax_lt = ecodes.ABS_X, pick(dev, TRIG_R), pick(dev, TRIG_L)
    print(f"Gamepad: {dev.name}   RT={ecodes.ABS[ax_rt]}  LT={ecodes.ABS[ax_lt]}")

    if "--axes" in sys.argv:
        for ev in dev.read_loop():
            if ev.type == ecodes.EV_ABS:
                print(f"AXIS {ecodes.ABS.get(ev.code, ev.code):<14} {ev.value}")
            elif ev.type == ecodes.EV_KEY and ev.value == 1:
                print(f"BUTTON {ecodes.BTN.get(ev.code, ev.code)}")
        return

    cfg = load_cfg()
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    time.sleep(0.3)

    gains = {"kp": 200.0, "ki": 400.0}
    r = query(ser, {"T": 701}, "kp")
    if r:
        gains["kp"], gains["ki"] = float(r["kp"]), float(r["ki"])
        print(f"PID read from board: P={gains['kp']:.0f} I={gains['ki']:.0f}")
    else:
        print("Could not read PID from board; using local defaults.")

    def stick(raw):
        i = info[ax_s]
        mid, half = (i.max + i.min) / 2.0, ((i.max - i.min) / 2.0 or 1)
        v = (raw - mid) / half
        if abs(v) < DEADZONE:
            return 0.0
        v = (abs(v) - DEADZONE) / (1 - DEADZONE) * (1 if v > 0 else -1)
        return -v if cfg["steer_inv"] else v

    def trig(code, raw):
        if code is None:
            return 0.0
        i = info[code]
        v = (raw - i.min) / ((i.max - i.min) or 1)
        return 0.0 if v < TRIG_DEAD else (v - TRIG_DEAD) / (1 - TRIG_DEAD)

    def send(l, r_):
        ser.write((json.dumps({"T": 1, "L": round(l, 3), "R": round(r_, 3)}) + "\n").encode())
        ser.flush()

    def push_pid(save=0):
        ser.write((json.dumps({"T": 700, "kp": round(gains["kp"]),
                               "ki": round(gains["ki"]), "kd": 0,
                               "lim": 255, "save": save}) + "\n").encode())
        ser.flush()

    def status():
        return (f"spd {cfg['max_lin']:.2f}  turn {cfg['turn_gain']:.2f}  "
                f"P {gains['kp']:.0f}  I {gains['ki']:.0f}"
                f"{'  [INV]' if cfg['steer_inv'] else ''}")

    vals = {ax_s: 0, ax_rt: 0, ax_lt: 0}
    held, armed, last, note = set(), False, 0.0, ""
    period = 1.0 / RATE_HZ

    print("\nA=arm  B=stop  X=flip steering  LB=boost  RB+dpad=PID  SELECT=save PID  START=quit")
    print(status() + "\n")
    dev.grab()

    try:
        while True:
            try:
                for ev in dev.read():
                    if ev.type == ecodes.EV_ABS:
                        if ev.code in vals:
                            vals[ev.code] = ev.value
                        elif ev.code in (HAT_X, HAT_Y) and ev.value != 0:
                            pidmod = PIDMOD in held
                            if ev.code == HAT_Y:
                                if pidmod:
                                    gains["kp"] = max(0, gains["kp"] + (P_STEP if ev.value < 0 else -P_STEP))
                                    push_pid()
                                else:
                                    lo, hi = LIMITS["max_lin"]
                                    d = STEP["max_lin"] if ev.value < 0 else -STEP["max_lin"]
                                    cfg["max_lin"] = min(hi, max(lo, cfg["max_lin"] + d))
                            else:
                                if pidmod:
                                    gains["ki"] = max(0, gains["ki"] + (I_STEP if ev.value > 0 else -I_STEP))
                                    push_pid()
                                else:
                                    lo, hi = LIMITS["turn_gain"]
                                    d = STEP["turn_gain"] if ev.value > 0 else -STEP["turn_gain"]
                                    cfg["turn_gain"] = min(hi, max(lo, cfg["turn_gain"] + d))
                            note = status()
                    elif ev.type == ecodes.EV_KEY:
                        if ev.value == 1:
                            held.add(ev.code)
                            if ev.code == ARM:
                                armed = not armed
                                note = "ARMED" if armed else "disarmed"
                            elif ev.code == STOP:
                                armed = False
                                send(0, 0)
                                note = "STOP"
                            elif ev.code == FLIP:
                                cfg["steer_inv"] = not cfg["steer_inv"]
                                note = "steering flipped" + ("  [INV]" if cfg["steer_inv"] else "")
                            elif ev.code == RESET:
                                cfg.update(DEFAULTS)
                                note = "reset: " + status()
                            elif ev.code == SAVE:
                                push_pid(save=1)
                                note = f"saved PID to flash: P={gains['kp']:.0f} I={gains['ki']:.0f}"
                            elif ev.code == QUIT:
                                raise KeyboardInterrupt
                        else:
                            held.discard(ev.code)
            except BlockingIOError:
                pass

            now = time.time()
            if now - last < period:
                time.sleep(0.002)
                continue
            last = now

            scale = 1.0 if BOOST in held else cfg["normal"]
            throttle = trig(ax_rt, vals[ax_rt]) - trig(ax_lt, vals[ax_lt])
            steer = stick(vals[ax_s])

            if not armed:
                l = r_ = v = 0.0
            else:
                v = throttle * cfg["max_lin"] * scale

                if abs(v) < PIVOT_BELOW:
                    # stationary: spin on the spot
                    t = steer * cfg["pivot"] * scale
                    l, r_ = t, -t
                else:
                    # moving: proportional arc - the inner wheel slows, the
                    # outer keeps speed, so radius is constant with speed
                    k = abs(steer) * cfg["turn_gain"]
                    if steer > 0:          # right: slow the right wheel
                        l, r_ = v, v * (1 - 2 * k)
                    else:                  # left: slow the left wheel
                        l, r_ = v * (1 - 2 * k), v

            send(l, r_)

            st = "ARMED " if armed else "safe  "
            thr = "#" * int(abs(throttle) * 10)
            # derive the indicator from the wheels themselves so it can never
            # disagree with what the robot is actually doing
            bias = (l - r_) / max(0.01, cfg["max_lin"])          # >0 = turning right
            sb = ("<" * int(max(0, -bias) * 6)).rjust(6) + "|" + ("*" * int(max(0, bias) * 6)).ljust(6)
            print(f"\r{st}{'F' if throttle>0 else ('R' if throttle<0 else ' ')} {thr:<10} "
                  f"[{sb}] L={l:+.2f} R={r_:+.2f}  {note:<52}", end="")

    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping.")
        for _ in range(3):
            send(0, 0)
            time.sleep(0.05)
        try:
            json.dump(cfg, open(SETTINGS, "w"), indent=2)
        except Exception:
            pass
        print(f"Saved locally: {status()}")
        try:
            dev.ungrab()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        main()
    except serial.SerialException as e:
        print(f"Serial error: {e}")
