#!/usr/bin/env python3
"""
lidar-bot all-in-one: telemetry + lidar + gamepad + web dashboard.

Runs ONE process that owns both serial ports (the ESP32 can only be opened by
one program at a time), so don't run gamepad_car.py alongside this.

  sudo apt install -y python3-serial python3-evdev
  python3 robot_server.py
  then open  http://<pi-ip>:8080

  --no-pad      telemetry + lidar only
  --lidar PORT  force the lidar port (default: first /dev/ttyUSB*)
"""
import sys, os, glob, json, math, time, struct, threading, socketserver
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import serial

ESP_PORT, ESP_BAUD = "/dev/ttyAMA0", 115200
LIDAR_BAUD = 230400
HTTP_PORT = 8080
SETTINGS = os.path.expanduser("~/.lidarbot_drive.json")

# lidar-on-camera overlay, drawn in the browser from the scan /data already
# carries, so it costs the Pi nothing.
OVL_DEFAULTS = {
    "ovl_on":      True,
    "ovl_hfov":    68.0,    # camera horizontal field of view, degrees
    "ovl_yaw":     0.0,     # lidar zero angle vs camera centre, degrees
    "ovl_pitch":   0.0,     # camera pitch, + = nose down
    "ovl_dz":     -0.06,    # lidar height MINUS camera height, metres
    "ovl_dx":      0.0,     # lidar ahead of the camera, metres
    "ovl_flip":    False,   # lidar angles run the other way
    "ovl_max":     6.0,     # ignore returns beyond this, metres
    "ovl_gap":     0.25,    # break the line if neighbours are further apart
    "ovl_wide":    3.0,
    "ovl_alpha":   0.85,
    "ovl_near":    0.3,     # heat map hot end, metres
    "ovl_far":     4.0,     # heat map cold end, metres
    "ovl_glow":    True,
    "ovl_labels":  False,
    "ovl_horizon": False,
}

CFG = {
    # --- open-loop drive feel --------------------------------------------
    "max_lin":    0.45,   # m/s at full trigger
    "turn_gain":  0.60,   # how hard the inner wheel is slowed when steering
    "pivot":      0.30,   # wheel m/s for a stationary pivot
    "normal":     0.60,   # throttle scale when the boost button is not held
    "steer_inv":  True,

    # --- gyro "compass" heading -----------------------------------------
    "hold_on":    False,  # hold heading while the stick is centred
    "rate_on":    False,  # closed-loop turn RATE while the stick is deflected
    "sign_ok":    False,  # set by the sign check - until then, do not trust hold
    # The magnetometer compass is NOT needed for mapping or navigation - SLAM
    # gets absolute heading by matching lidar scans, and indoors a magnetometer
    # sitting next to motors and steel is usually worse than nothing. Gyro only
    # is the right default; the other two stay available as diagnostics.
    "hdg_src":    "gyro", # "gyro" = integrated gz | "yaw" = board compass
                          # | "fused" = gyro, pulled slowly onto the compass
    # how the IMU is bolted in - "rp" = firmware roll is robot roll,
    # "pr" = they are swapped (roll the robot and the pitch number moves)
    # axle spacing measured with a ruler, metres. Scrub can only make the
    # EFFECTIVE track width bigger than this, and not by much with a caster, so
    # it is the yardstick for spotting a nonsense calibration.
    "tw_geom":    0.175,

    "imu_map":    "rp",
    "roll_inv":   False,
    "pitch_inv":  False,
    "roll_off":   0.0,    # level offsets, degrees
    "pitch_off":  0.0,

    # kp was 0.5, a two-second pull onto the compass - that is not a
    # complementary filter, it is the compass with a little smoothing, and it
    # dragged the heading back to north seconds after every turn. The gyro has
    # to own the short term; the compass only trims it over tens of seconds.
    "fuse_kp":    0.10,   # pull onto the compass, 1/s (~10 s)
    "fuse_ki":    0.005,  # learns the gyro's leftover drift (~15 s)
    "fuse_tol":   20.0,   # deg the compass may disagree with the gyro over one
                          # second before it is treated as disturbed
    "hdg_inv":    False,  # flip if heading counts the wrong way round
    "gyro_bias":  0.0,    # deg/s read while dead still
    "gyro_scale": 1.0,    # deg reported per deg actually turned

    # heading hold (straight-line). The controller commands a YAW RATE in
    # deg/s and only converts to a wheel differential at the very end, so the
    # gains stay meaningful if the track width changes.
    "hold_kp":    2.00,   # deg/s of correction per degree off line
    "hold_ki":    1.20,   # deg/s per degree-second
    "hold_kd":    0.25,   # damping, fed from the measured yaw rate directly
    "hold_max":   70.0,   # deg/s clamp on the correction

    # turn-rate tracking (while steering)
    "turn_boost": 1.00,   # >1 = snappier than the open-loop command asks for
    "turn_rate":  220.0,  # deg/s ceiling
    "rate_kp":    0.60,   # deg/s of trim per deg/s of rate error
    "rate_ki":    1.50,
    "rate_max":   90.0,   # deg/s clamp on the trim
}
CFG.update(OVL_DEFAULTS)
# Bump CFG_VER whenever a stored gain changes meaning or units, so an old
# settings file cannot quietly reintroduce numbers from a different scheme.
CFG_VER = 4
_FEEL = ("max_lin", "turn_gain", "pivot", "normal", "steer_inv")
# Settings whose tuning changed meaning at a given version. On upgrade these
# are reset to the new defaults while everything else - the calibration you
# spent real time on - is kept.
_RETUNE = {4: ("fuse_kp", "fuse_ki")}
try:
    _saved = json.load(open(SETTINGS))
    _v = _saved.get("_ver")
    if _v == CFG_VER:
        CFG.update({k: v for k, v in _saved.items() if k in CFG})
    elif isinstance(_v, int) and 0 < _v < CFG_VER:
        _keep = {k: v for k, v in _saved.items() if k in CFG}
        _reset = []
        for _n in range(_v + 1, CFG_VER + 1):
            for _k in _RETUNE.get(_n, ()):
                if _keep.pop(_k, None) is not None:
                    _reset.append(_k)
        CFG.update(_keep)
        print("settings upgraded to v%d, calibration kept" % CFG_VER
              + (", re-tuned: " + ", ".join(_reset) if _reset else ""))
    else:
        # different scheme - keep only the drive feel, take fresh gains
        CFG.update({k: v for k, v in _saved.items() if k in _FEEL})
        print("settings file predates the heading controller - "
              "using fresh default gains")
except Exception:
    pass

SAVE_ERR = {"msg": "", "n": 0}


def save_cfg():
    """Persist the settings. This used to swallow every error, so a file that
    could not be written (owned by root after one sudo run, full disk) meant
    calibration silently vanished on the next restart with nothing to see."""
    try:
        out = dict(CFG)
        out["_ver"] = CFG_VER
        tmp = SETTINGS + ".tmp"
        with open(tmp, "w") as f:
            json.dump(out, f, indent=2)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, SETTINGS)          # atomic: never a half-written file
        if SAVE_ERR["msg"]:
            print("settings are saving again: %s" % SETTINGS)
        SAVE_ERR["msg"] = ""
        return True
    except Exception as e:
        SAVE_ERR["msg"] = ("cannot write %s (%s) - calibration will be LOST on "
                           "restart" % (SETTINGS, e))
        SAVE_ERR["n"] += 1
        print("SETTINGS SAVE FAILED: %s" % SAVE_ERR["msg"])
        return False

CAM = {"on": True, "index": 0, "width": 640, "height": 480,
       "fps": 15, "quality": 60, "flip": False}


# raw JPEG bytes live OUTSIDE `state` - `state` gets JSON-encoded for /data
cam_frame = {"jpeg": None}
cam_state = {"kbps": 0.0, "fps": 0.0, "err": "", "size": "-", "dev": "-"}


def _try_open(cv2, idx):
    """Open a V4L2 capture node and prove it actually yields a frame."""
    for src in (idx, f"/dev/video{idx}"):
        try:
            cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
        except Exception:
            continue
        if cap.isOpened():
            return cap, f"/dev/video{idx}"
        cap.release()
    return None, None

def camera_thread():
    try:
        import cv2
    except ImportError:
        cam_state["err"] = "python3-opencv not installed"
        return

    cap = None
    cur = None
    bytes_win, frames_win, t_win = 0, 0, time.time()

    while True:
        if not CAM["on"]:
            if cap is not None:
                cap.release(); cap = None; cur = None
            cam_frame["jpeg"] = None
            time.sleep(0.3)
            continue

        want = (CAM["index"], CAM["width"], CAM["height"], CAM["fps"])
        if cap is None or cur != want:
            if cap is not None:
                cap.release()
            # index -1 = auto: probe every node until one opens and reads
            order = ([CAM["index"]] if CAM["index"] >= 0 else []) + list(range(0, 10))
            dev = None
            for idx in order:
                cap, dev = _try_open(cv2, idx)
                if cap is None:
                    continue
                # ask the camera for MJPEG - far less USB bandwidth than raw YUV
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM["width"])
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM["height"])
                cap.set(cv2.CAP_PROP_FPS, CAM["fps"])
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                ok, _f = cap.read()        # metadata nodes open but never read
                if ok:
                    break
                cap.release(); cap = None; dev = None
            cur = want
            if cap is None:
                perm = "" if os.access("/dev/video0", os.R_OK) else \
                       "  (no read permission - run: sudo usermod -aG video $USER, then log out/in)"
                cam_state["err"] = f"no working camera node{perm}"
                cam_state["dev"] = "-"
                time.sleep(2)
                continue
            cam_state["err"] = ""
            cam_state["dev"] = dev

        ok, frame = cap.read()
        if not ok:
            cam_state["err"] = "read failed"
            cap.release(); cap = None
            time.sleep(0.5)
            continue

        if CAM["flip"]:
            frame = cv2.flip(frame, -1)

        ok, buf = cv2.imencode(".jpg", frame,
                               [int(cv2.IMWRITE_JPEG_QUALITY), int(CAM["quality"])])
        if ok:
            data = buf.tobytes()
            cam_frame["jpeg"] = data
            cam_state["size"] = f"{frame.shape[1]}x{frame.shape[0]}"
            bytes_win += len(data); frames_win += 1

        now = time.time()
        if now - t_win >= 1.0:
            cam_state["kbps"] = bytes_win / 1024.0 / (now - t_win)
            cam_state["fps"] = frames_win / (now - t_win)
            bytes_win, frames_win, t_win = 0, 0, now

        time.sleep(max(0.0, 1.0 / max(1, CAM["fps"]) - 0.005))


state = {
    "cfg": CFG,
    "mode": 0,                  # 1 = open loop, 3 = closed loop (from the board)
    "telem": {},
    "scan": [],                 # [[angle_deg, dist_mm], ...]
    "lidar_rpm": 0.0,
    "drive": {"armed": False, "throttle": 0.0, "steer": 0.0, "l": 0.0, "r": 0.0},
    "pad": "none",
    "cam": CAM,
    "cam_stat": cam_state,
}
lock = threading.Lock()
ESP = None
TEL = {"seq": 0}          # bumped for every frame that carries odometry

# ------------------------------------------------- geometry + gyro heading
# wd/ppr/tw are the board's own numbers, fetched with T:701 and written back
# with T:700 so the ESP32 and the Pi never disagree about the drivetrain.
GEO = {"wd": 0.0800, "ppr": 1650, "tw": 0.1720, "swap": 0, "ok": False}


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


HDG = {
    "gyro": 0.0,    # integrated gz, degrees, continuous (slow drift)
    "ymag": 0.0,    # board AHRS/compass yaw, unwrapped to continuous
    "yaw":  0.0,    # board AHRS/compass yaw, raw -180..180
    "fuse": 0.0,    # gyro, slowly pulled onto the compass (absolute, smooth)
    "bhat": 0.0,    # leftover gyro drift the fused filter has learnt, deg/s
    "magdiff": 0.0, # how far the compass and the gyro disagree, degrees
    "mrate": 0.0,   # turn rate worked out from the compass, deg/s (filtered)
    "srate": 0.0,   # turn rate belonging to the SELECTED source - what the
                    # controller and the on-screen arrow both use
    "yok":  False,  # is the board compass arriving?
    "ready": False, # is the selected source usable?
    "slow": False,  # gz is arriving, but far too slowly to integrate
    "polled": False,# we are polling the board to get gz at all
    "gaps": 0,      # telemetry dropouts seen - each one costs heading accuracy
    "mrej": 0,      # compass jumps rejected as physically impossible
    "mtrust": False,# is the compass currently being believed at all
    "_wt": 0.0,     # trust window: start time, and the two headings at its start
    "_wm": 0.0,
    "_wg": 0.0,
    "_dtn": 0.0,    # the normal gap between gz samples, learnt
    "_over": 0,     # consecutive late samples - 3 means the cadence changed
    "deg":  0.0,    # whichever source the controller is using
    "rate": 0.0,    # deg/s after bias + scale correction
    "raw":  0.0,    # deg/s straight off the board
    "hz":   0.0,    # telemetry frames per second feeding the estimate
    "ok":   False,
    "_ly":  None,
    "_t":   0.0,
    "_n":   0,
    "_ht":  0.0,
    "warn": "",     # loud message for the UI when something is clearly wrong
    "_seq": 0,      # frame counter, so samplers can tell frames apart
    "_lym": 0.0,    # previous unwrapped compass angle
    "_lyt": 0.0,    # when we last saw a compass angle
}


def hdg_update(d):
    """Fold one telemetry frame into the heading estimate."""
    now = time.time()
    sgn = -1.0 if CFG["hdg_inv"] else 1.0

    if "y" in d:
        try:
            HDG["yaw"] = float(d["y"])
            y = HDG["yaw"] * sgn
            if HDG["_ly"] is None:
                HDG["_ly"] = y
                HDG["_lym"] = HDG["ymag"]
            # Cap how much the compass may move in one sample. During a spin
            # the magnetometer swings all over the place next to the motors, and
            # unwrapping raw noise accumulated thousands of bogus degrees.
            dy = wrap180(y - HDG["_ly"])
            tyd0 = (now - HDG["_lyt"]) if HDG["_lyt"] else 0.04
            lim = max(20.0, abs(HDG["rate"]) * max(tyd0, 0.01) * 2.5)
            if abs(dy) > lim:
                HDG["mrej"] += 1
                dy = lim if dy > 0 else -lim
            HDG["ymag"] += dy
            HDG["_ly"] = y
            # the compass can also give us a turn rate - noisy, so filter it,
            # because with the gyro missing this is the only rate we have
            tyd = (now - HDG["_lyt"]) if HDG["_lyt"] else 0.0
            HDG["_lyt"] = now
            if 0.0 < tyd < 0.5:
                mr = (HDG["ymag"] - HDG["_lym"]) / tyd
                HDG["mrate"] += (mr - HDG["mrate"]) * min(1.0, tyd / 0.15)
            HDG["_lym"] = HDG["ymag"]
            HDG["yok"] = True
        except (TypeError, ValueError):
            pass

    if "gz" in d:
        try:
            raw = float(d["gz"])
        except (TypeError, ValueError):
            raw = None
        if raw is not None:
            HDG["raw"] = raw
            rate = (raw - CFG["gyro_bias"]) * CFG["gyro_scale"] * sgn
            dt = (now - HDG["_t"]) if HDG["_t"] else 0.0
            HDG["_t"] = now
            HDG["rate"] = rate
            # A board that only sends gz in its once-a-second status reply used
            # to fail this dt test silently, leaving the gyro frozen at zero
            # with no warning anywhere. So integrate whatever cadence we get -
            # but learn what the normal gap looks like, and never integrate far
            # more than that, or one dropped second of telemetry writes a
            # rate-times-gap lie straight into the heading.
            typ = HDG["_dtn"] if HDG["_dtn"] > 0 else dt
            cap = max(0.08, typ * 2.5)
            if dt > cap:
                # One late sample is a dropout and gets clamped. Several in a
                # row is not a dropout, it is the real cadence having changed -
                # learn it, or we permanently integrate a fraction of every
                # sample and the heading reads a fraction of the true rotation.
                HDG["_over"] += 1
                if HDG["_over"] >= 3:
                    HDG["_dtn"] = dt
                    HDG["_over"] = 0
                    use = dt
                else:
                    HDG["gaps"] += 1
                    use = min(dt, cap)
            else:
                HDG["_over"] = 0
                use = dt
                HDG["_dtn"] = typ + (dt - typ) * 0.05
            HDG["slow"] = typ > 0.25
            if 0.0 < use < 1.5:
                dt = use
                HDG["gyro"] += rate * dt
                # Fused: integrate the gyro, then lean gently towards the
                # compass. The compass is absolute but the magnetometer sits
                # next to the motor wiring, so while the robot is actually
                # turning we ignore it completely and only trust it when
                # things are calm.
                # Two states: heading, and an estimate of the drift still left
                # in the gyro. Leaning on the compass alone cannot beat a gyro
                # bias - it just settles at bias/gain degrees off - so the
                # filter learns the bias and subtracts it instead.
                HDG["fuse"] += (rate - HDG["bhat"]) * dt
                # Only trust the compass when it is calm AND its own turn rate
                # agrees with the gyro's. A magnetometer being swung about by
                # motor current disagrees violently, and leaning on it then is
                # what makes the heading snap back after a turn.
                # Compare how far each source moved over a WHOLE SECOND, not
                # their instantaneous rates: a compass rate is differentiated
                # noise and is never quiet enough to agree with anything.
                if HDG["_wt"] == 0.0:
                    HDG["_wt"], HDG["_wm"], HDG["_wg"] = now, HDG["ymag"], 0.0
                HDG["_wg"] += rate * dt
                if now - HDG["_wt"] >= 1.0:
                    dm = HDG["ymag"] - HDG["_wm"]
                    HDG["mtrust"] = abs(wrap180(dm - HDG["_wg"])) < CFG["fuse_tol"]
                    HDG["_wt"], HDG["_wm"], HDG["_wg"] = now, HDG["ymag"], 0.0
                if abs(rate) < 25.0 and HDG["mtrust"]:
                    e = wrap180(HDG["ymag"] - HDG["fuse"])
                    HDG["fuse"] += CFG["fuse_kp"] * e * dt
                    HDG["bhat"] -= CFG["fuse_ki"] * e * dt
                    HDG["bhat"] = max(-20.0, min(20.0, HDG["bhat"]))
            HDG["ok"] = True
            HDG["_n"] += 1
            HDG["_seq"] += 1
            if now - HDG["_ht"] >= 1.0:
                HDG["hz"] = HDG["_n"] / max(1e-3, now - HDG["_ht"])
                HDG["_n"], HDG["_ht"] = 0, now

    HDG["magdiff"] = wrap180(HDG["ymag"] - HDG["gyro"])
    src = CFG["hdg_src"]
    if src == "yaw":
        HDG["deg"] = HDG["ymag"]
        HDG["srate"] = HDG["mrate"]
        HDG["ready"] = HDG["yok"]
    elif src == "fused":
        HDG["deg"] = HDG["fuse"]
        HDG["srate"] = HDG["rate"] - HDG["bhat"]
        HDG["ready"] = HDG["ok"] and HDG["yok"] and not HDG["slow"]
    else:
        HDG["deg"] = HDG["gyro"]
        HDG["srate"] = HDG["rate"]
        HDG["ready"] = HDG["ok"] and not HDG["slow"]


ATT = {"roll": 0.0, "pitch": 0.0, "rawr": 0.0, "rawp": 0.0, "seq": 0}


def att_update(d):
    """Map the board's roll/pitch onto the robot's own axes."""
    try:
        if "r" in d:
            ATT["rawr"] = float(d["r"])
        if "p" in d:
            ATT["rawp"] = float(d["p"])
    except (TypeError, ValueError):
        return
    if "r" in d or "p" in d:
        ATT["seq"] += 1
    r, pi = ATT["rawr"], ATT["rawp"]
    if CFG["imu_map"] == "pr":
        r, pi = pi, r
    if CFG["roll_inv"]:
        r = -r
    if CFG["pitch_inv"]:
        pi = -pi
    ATT["roll"] = r - CFG["roll_off"]
    ATT["pitch"] = pi - CFG["pitch_off"]


def att_level():
    """Call with the robot standing as it normally stands: this becomes zero."""
    r, pi = ATT["rawr"], ATT["rawp"]
    if CFG["imu_map"] == "pr":
        r, pi = pi, r
    if CFG["roll_inv"]:
        r = -r
    if CFG["pitch_inv"]:
        pi = -pi
    CFG["roll_off"] = round(r, 3)
    CFG["pitch_off"] = round(pi, 3)
    att_update({})
    save_cfg()
    return {"roll_off": CFG["roll_off"], "pitch_off": CFG["pitch_off"]}


def hdg_zero():
    HDG["_wt"] = 0.0
    HDG["gyro"] = 0.0
    HDG["ymag"] = 0.0
    HDG["fuse"] = 0.0
    HDG["deg"] = 0.0
    HDG["_ly"] = None
    HOLD.reset()


def hdg_health():
    """Data going quiet must show up as NO DATA, not as a stale number that
    never changes - so age each source out if nothing has carried it lately."""
    now = time.time()
    age = now - HDG["_t"] if HDG["_t"] else 999.0
    if age > 2.0:
        HDG["ok"] = False
        HDG["hz"] = 0.0
        HDG["rate"] = 0.0
    yage = now - HDG["_lyt"] if HDG["_lyt"] else 999.0
    if yage > 2.0:
        HDG["yok"] = False
        HDG["mrate"] = 0.0
    src = CFG["hdg_src"]
    HDG["ready"] = (HDG["yok"] if src == "yaw"
                    else (HDG["ok"] and HDG["yok"] and not HDG["slow"])
                    if src == "fused"
                    else (HDG["ok"] and not HDG["slow"]))
    return age


class Heading:
    """
    Closed-loop heading trim layered on top of the open-loop mixer - the
    open-loop wheel command is left exactly as it was, and this only adds a
    differential on top of it, so the drive feel does not change.

    stick centred   -> hold the heading held when the stick centred
                       (kills the caster's 'shopping trolley' pull)
    stick deflected -> actually achieve the turn rate the open-loop wheel
                       command implies, so turns repeat regardless of floor
                       grip, battery state or which way the caster is pointing
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.target = None
        self.i = 0.0
        self.ri = 0.0
        self.last_err = 0.0
        self._t = 0.0
        self.mode = "idle"
        self.err = 0.0
        self.corr = 0.0
        self.want = 0.0
        self.wcmd = 0.0
        self.bad_t = 0.0

    @staticmethod
    def _diff(w_deg):
        """Yaw rate command (deg/s) -> wheel differential (m/s per side)."""
        tw = GEO["tw"] if GEO["tw"] > 0.02 else 0.172
        return w_deg / 57.29578 * tw / 2.0

    def trim(self, v, steer, armed, l_ff=0.0, r_ff=0.0):
        """Returns wheel differential in m/s. Positive turns the robot LEFT."""
        now = time.time()
        dt = (now - self._t) if self._t else 0.02
        self._t = now
        dt = min(max(dt, 0.005), 0.2)
        h = HDG["deg"]

        if not armed or not HDG["ready"]:
            self.target = h
            self.i = self.ri = self.last_err = 0.0
            self.mode = "no gyro" if armed else "idle"
            self.err = self.corr = self.want = self.wcmd = 0.0
            return 0.0

        turning = abs(steer) > 0.08
        moving = abs(v) > 0.05

        if turning:
            # ---- turn-rate tracking ------------------------------------
            self.i = self.last_err = 0.0
            self.target = h                 # hold resumes from wherever we stop
            tw = GEO["tw"] if GEO["tw"] > 0.02 else 0.172
            want = (r_ff - l_ff) / tw * 57.29578 * CFG["turn_boost"]
            want = max(-CFG["turn_rate"], min(CFG["turn_rate"], want))
            self.want = want
            if not CFG["rate_on"]:
                self.ri = 0.0
                self.mode = "rate off"
                self.err = self.corr = self.wcmd = 0.0
                return 0.0
            # A gyro that counts backwards makes the trim fight the turn and
            # the robot lurches the other way. Catch it instead of letting it
            # run: commanded one way, measurably going the other.
            if abs(want) > 25.0 and abs(HDG["srate"]) > 15.0 \
                    and want * HDG["srate"] < 0:
                self.bad_t += dt
            else:
                self.bad_t = max(0.0, self.bad_t - dt * 0.5)
            if self.bad_t > 0.5:
                CFG["rate_on"] = CFG["hold_on"] = False
                CFG["sign_ok"] = False
                HDG["warn"] = ("gyro is reading backwards - closed loop switched "
                               "off. Run STEP 1 (gyro direction).")
                self.ri = 0.0
                self.mode = "sign fault"
                self.err = self.corr = self.wcmd = 0.0
                return 0.0
            err = want - HDG["srate"]
            self.ri += err * dt
            lim = CFG["rate_max"] / max(CFG["rate_ki"], 1e-6)
            self.ri = max(-lim, min(lim, self.ri))
            w = CFG["rate_kp"] * err + CFG["rate_ki"] * self.ri
            w = max(-CFG["rate_max"], min(CFG["rate_max"], w))
            self.mode = "rate"
            self.err = err
            self.wcmd = w
            self.corr = self._diff(w)
            return self.corr

        self.want = 0.0
        self.ri = 0.0

        if not moving or not CFG["hold_on"]:
            self.target = h
            self.i = self.last_err = 0.0
            self.mode = "hold off" if moving else "idle"
            self.err = self.corr = self.wcmd = 0.0
            return 0.0

        # ---- heading hold ----------------------------------------------
        if self.target is None:
            self.target = h
        err = wrap180(self.target - h)
        if abs(err) > 60.0:
            # hold should never be allowed to get this far off line
            CFG["hold_on"] = False
            CFG["sign_ok"] = False
            HDG["warn"] = ("heading hold lost the line (%.0f deg off) - switched "
                           "off. Run STEP 1 (gyro direction)." % err)
            self.i = 0.0
            self.mode = "hold fault"
            self.err = self.corr = self.wcmd = 0.0
            return 0.0
        self.i += err * dt
        lim = CFG["hold_max"] / max(CFG["hold_ki"], 1e-6)
        self.i = max(-lim, min(lim, self.i))
        self.last_err = err
        # d(err)/dt is just -(yaw rate), which the gyro already gives us
        # cleanly - no need to differentiate a noisy integrated angle
        w = (CFG["hold_kp"] * err + CFG["hold_ki"] * self.i
             - CFG["hold_kd"] * HDG["srate"])
        w = max(-CFG["hold_max"], min(CFG["hold_max"], w))
        self.mode = "hold"
        self.err = err
        self.wcmd = w
        self.corr = self._diff(w)
        return self.corr


HOLD = Heading()


def mix_v(v, steer, armed, pivot=None):
    """v in m/s, steer -1..1  ->  (left, right) wheel speed in m/s."""
    if not armed:
        HOLD.trim(0.0, 0.0, False)
        return 0.0, 0.0
    if pivot is None:
        pivot = CFG["pivot"]
    if abs(v) < 0.06:
        t = steer * pivot
        l_ff, r_ff = t, -t
    else:
        k = abs(steer) * CFG["turn_gain"]
        l_ff, r_ff = (v, v * (1 - 2 * k)) if steer > 0 else (v * (1 - 2 * k), v)
    corr = HOLD.trim(v, steer, True, l_ff, r_ff)
    cap = CFG["max_lin"] * 1.4
    return (max(-cap, min(cap, l_ff - corr)),
            max(-cap, min(cap, r_ff + corr)))


def mix(throttle, steer, scale, armed):
    v = throttle * CFG["max_lin"] * scale
    l, r = mix_v(v, steer, armed, pivot=CFG["pivot"] * scale)
    return l, r, v


def odo_counts():
    t = state["telem"]
    try:
        return float(t.get("odl", 0.0)), float(t.get("odr", 0.0))
    except (TypeError, ValueError):
        return 0.0, 0.0


def counts_to_m():
    return math.pi * GEO["wd"] / max(1.0, float(GEO["ppr"]))


def odo_m():
    k = counts_to_m()
    cl, cr = odo_counts()
    return cl * k, cr * k


# --------------------------------------------------------------- ESP32 link
class Esp:
    def __init__(self):
        self.ser = serial.Serial(ESP_PORT, ESP_BAUD, timeout=0.3)
        self.wlock = threading.Lock()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        while True:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                d = json.loads(line)
                if "wd" in d and "tw" in d:          # T:701 / T:710 reply
                    try:
                        GEO.update({"wd": float(d["wd"]),
                                    "ppr": float(d["ppr"]),
                                    "tw": float(d["tw"]),
                                    "swap": 1 if d.get("swap") else 0,
                                    "ok": True})
                    except (TypeError, ValueError, KeyError):
                        pass
                if "odl" in d or "v" in d:
                    hdg_update(d)
                    att_update(d)
                    if "odl" in d:
                        TEL["seq"] += 1
                    with lock:
                        state["telem"].update(d)
                        if "mt" in d:
                            state["mode"] = d["mt"]
            except Exception:
                continue

    def send(self, obj):
        try:
            with self.wlock:
                self.ser.write((json.dumps(obj) + "\n").encode())
                self.ser.flush()
        except Exception:
            pass


# --------------------------------------------------------------- lidar
def lidar_thread(port):
    while True:
        try:
            ser = serial.Serial(port, LIDAR_BAUD, timeout=1)
            buf = bytearray()
            pts = {}
            while True:
                buf += ser.read(max(1, ser.in_waiting))
                while True:
                    i = buf.find(b'\x54')
                    if i < 0 or len(buf) < i + 2:
                        break
                    n = buf[i + 1] & 0x1F
                    plen = 11 + 3 * n
                    if len(buf) < i + plen:
                        break
                    pkt = bytes(buf[i:i + plen])
                    del buf[:i + plen]

                    speed = struct.unpack_from('<H', pkt, 2)[0]
                    a0 = struct.unpack_from('<H', pkt, 4)[0] / 100.0
                    a1 = struct.unpack_from('<H', pkt, 6 + 3 * n)[0] / 100.0
                    span = (a1 - a0) % 360.0
                    step = span / (n - 1) if n > 1 else 0
                    for k in range(n):
                        dist, _q = struct.unpack_from('<HB', pkt, 6 + 3 * k)
                        ang = (a0 + step * k) % 360.0
                        if 0 < dist < 20000:
                            pts[int(ang)] = dist
                    if a1 < a0:          # completed a revolution
                        with lock:
                            state["scan"] = sorted(pts.items())
                            state["lidar_rpm"] = speed / 6.0
                        pts = {}
        except Exception:
            with lock:
                state["scan"] = []
            time.sleep(2)


# --------------------------------------------------------------- gamepad
def pad_thread(esp):
    from evdev import InputDevice, ecodes, list_devices

    cfg = CFG

    dev = None
    for p in list_devices():
        d = InputDevice(p)
        c = d.capabilities()
        if ecodes.EV_ABS in c and ecodes.EV_KEY in c and \
           ecodes.ABS_X in [x for x, _ in c[ecodes.EV_ABS]]:
            dev = d
            break
    if dev is None:
        with lock:
            state["pad"] = "not found"
        return

    info = {c: i for c, i in dev.capabilities()[ecodes.EV_ABS]}
    have = list(info)
    ax_rt = ecodes.ABS_GAS if ecodes.ABS_GAS in have else ecodes.ABS_RZ
    ax_lt = ecodes.ABS_BRAKE if ecodes.ABS_BRAKE in have else ecodes.ABS_Z
    with lock:
        state["pad"] = dev.name

    vals = {ecodes.ABS_X: 0, ax_rt: 0, ax_lt: 0}
    held, armed = set(), False

    def stick(raw):
        i = info[ecodes.ABS_X]
        mid, half = (i.max + i.min) / 2.0, ((i.max - i.min) / 2.0 or 1)
        v = (raw - mid) / half
        if abs(v) < 0.12:
            return 0.0
        v = (abs(v) - 0.12) / 0.88 * (1 if v > 0 else -1)
        return -v if cfg["steer_inv"] else v

    def trig(code, raw):
        i = info[code]
        v = (raw - i.min) / ((i.max - i.min) or 1)
        return 0.0 if v < 0.06 else (v - 0.06) / 0.94

    dev.grab()
    last = 0.0
    while True:
        try:
            for ev in dev.read():
                if ev.type == ecodes.EV_ABS and ev.code in vals:
                    vals[ev.code] = ev.value
                elif ev.type == ecodes.EV_KEY:
                    if ev.value == 1:
                        held.add(ev.code)
                        if ev.code == ecodes.BTN_SOUTH:
                            armed = not armed
                        elif ev.code == ecodes.BTN_EAST:
                            armed = False
                        elif ev.code == ecodes.BTN_WEST:
                            cfg["steer_inv"] = not cfg["steer_inv"]
                            save_cfg()
                    else:
                        held.discard(ev.code)
        except BlockingIOError:
            pass
        except OSError:
            with lock:
                state["pad"] = "disconnected"
            return

        now = time.time()
        if now - last < 0.04:
            time.sleep(0.002)
            continue
        last = now

        scale = 1.0 if ecodes.BTN_TL in held else cfg["normal"]
        throttle = trig(ax_rt, vals[ax_rt]) - trig(ax_lt, vals[ax_lt])
        steer = stick(vals[ecodes.ABS_X])

        # a calibration routine owns the motors while it runs; any real stick
        # input aborts it so the pad is always the emergency override
        if AUTO["busy"]:
            if abs(throttle) > 0.15 or abs(steer) > 0.35:
                AUTO["abort"] = True
            continue

        l, r, v = mix(throttle, steer, scale, armed)

        esp.send({"T": 1, "L": round(l, 3), "R": round(r, 3)})
        with lock:
            state["drive"] = {"armed": armed, "throttle": throttle,
                              "steer": steer, "l": l, "r": r}



# ------------------------------------------------------- calibration runner
AUTO = {"busy": False, "name": "", "step": "", "msg": "",
        "res": {}, "abort": False, "ok": True, "last": {}}


def auto_stop():
    """Emergency stop: cut the motors, abort any routine, and always land back
    in a clean idle state so the page never sits on 'stopping' forever."""
    _halt()
    if AUTO["busy"]:
        AUTO["abort"] = True          # _auto_body clears busy and sets 'aborted'
        AUTO["msg"] = "stopped"

        def _watchdog():
            # a routine normally clears itself within a few ms; if anything ever
            # leaves the flag set, do not let the page sit on RUNNING forever
            time.sleep(2.0)
            if AUTO["busy"] and AUTO["abort"]:
                AUTO.update({"busy": False, "abort": False, "step": "idle",
                             "msg": "stopped (forced)", "ok": True})
                _halt()
        threading.Thread(target=_watchdog, daemon=True).start()
    else:
        AUTO.update({"abort": False, "step": "idle", "ok": True,
                     "msg": "motors cut"})
    CFG["hold_on"] = CFG["rate_on"] = False
    HOLD.reset()


def wait_fresh(secs=2.0):
    """Block until the board has sent a new odometry frame. A routine that
    snapshots the encoders before the first frame lands measures from a stale
    number and gets the travel wrong."""
    seq = TEL["seq"]
    t0 = time.time()
    while TEL["seq"] == seq and time.time() - t0 < secs and not AUTO["abort"]:
        AUTO["step"] = "waiting for odometry from the board"
        time.sleep(0.02)
    return TEL["seq"] != seq


def _f(a, k, d):
    try:
        return float(a.get(k, [d])[0])
    except (TypeError, ValueError, IndexError):
        return d


def _drive(v, steer):
    l, r = mix_v(v, steer, True)
    if ESP is not None:
        ESP.send({"T": 1, "L": round(l, 3), "R": round(r, 3)})
    with lock:
        state["drive"] = {"armed": True, "throttle": v / max(CFG["max_lin"], .01),
                          "steer": steer, "l": l, "r": r}


def _spin(l, r):
    if ESP is not None:
        ESP.send({"T": 1, "L": round(l, 3), "R": round(r, 3)})
    with lock:
        state["drive"] = {"armed": True, "throttle": 0.0, "steer": 0.0,
                          "l": l, "r": r}


def _halt():
    if ESP is not None:
        ESP.send({"T": 1, "L": 0, "R": 0})
    with lock:
        state["drive"] = {"armed": False, "throttle": 0.0, "steer": 0.0,
                          "l": 0.0, "r": 0.0}


# ---------------------------------------------------------------- routines
def r_bias(a):
    """Sit dead still and average raw gz - that average IS the bias."""
    secs = _f(a, "secs", 12.0)
    _halt()
    acc, n, t0, seq = 0.0, 0, time.time(), -1
    while time.time() - t0 < secs and not AUTO["abort"]:
        if HDG["_seq"] != seq:               # one sample per telemetry frame
            seq = HDG["_seq"]
            try:
                acc += float(state["telem"].get("gz", 0.0))
                n += 1
            except (TypeError, ValueError):
                pass
        AUTO["step"] = "do not touch it - %.0fs left" % max(0.0, secs - (time.time() - t0))
        time.sleep(0.02)
    b = acc / max(1, n)
    AUTO["res"] = {"bias": round(b, 4), "n": n}
    AUTO["msg"] = ("raw gyro sits at %+.3f deg/s with the robot still (%d samples). "
                   "Press APPLY to subtract it." % (b, n))


def r_straight(a):
    """Drive a commanded distance; the user measures the real one."""
    dist = _f(a, "dist", 2.0)
    speed = _f(a, "speed", 0.25)
    hold = _f(a, "hold", 1.0) > 0.5
    keep = CFG["hold_on"]
    CFG["hold_on"] = hold
    try:
        HOLD.reset()
        wait_fresh()
        time.sleep(0.15)
        wait_fresh()
        c0l, c0r = odo_counts()
        h0 = HDG["deg"]
        g0, m0 = HDG["gyro"], HDG["ymag"]
        k = counts_to_m()
        t0 = time.time()
        cap = dist / max(speed, 0.05) * 3.0 + 8.0
        dev_max = 0.0
        trav = 0.0
        while not AUTO["abort"]:
            cl, cr = odo_counts()
            trav = ((cl - c0l) + (cr - c0r)) / 2.0 * k
            dev = wrap180(HDG["deg"] - h0)
            if abs(dev) > abs(dev_max):
                dev_max = dev
            AUTO["step"] = "%.2f / %.2f m   drift %+.1f deg" % (trav, dist, dev)
            if trav >= dist or time.time() - t0 > cap:
                break
            _drive(speed, 0.0)
            time.sleep(0.02)
        _halt()
        AUTO["step"] = "coasting"
        time.sleep(1.0)
        cl, cr = odo_counts()
        trav = ((cl - c0l) + (cr - c0r)) / 2.0 * k
        dev = wrap180(HDG["deg"] - h0)
        AUTO["res"] = {"kind": "straight", "cmd_m": dist, "enc_m": round(trav, 4),
                       "encl_m": round((cl - c0l) * k, 4),
                       "encr_m": round((cr - c0r) * k, 4),
                       "dev": round(dev, 2), "dev_max": round(dev_max, 2),
                       "gyro_dev": round(HDG["gyro"] - g0, 2),
                       "mag_dev": round(HDG["ymag"] - m0, 2),
                       "hold": 1 if hold else 0}
        AUTO["msg"] = ("encoders say %.3f m. Heading moved %+.1f deg (peak %+.1f). "
                       "Tape-measure the real distance and enter it below."
                       % (trav, dev, dev_max))
    finally:
        CFG["hold_on"] = keep


def r_spin(a):
    """Pivot for a FIXED TIME, then let the user say how far it really went.

    This used to spin until the gyro said it had reached the target angle, which
    made the gyro reading always come out as the target plus a bit of coast - so
    it measured nothing, and applying a scale correction never converged. The
    stop condition has to be independent of the thing being measured.
    """
    secs = _f(a, "secs", 45.0)
    rate = _f(a, "rate", 0.12)
    ccw = _f(a, "ccw", 1.0) > 0.5
    keep_r, keep_h = CFG["rate_on"], CFG["hold_on"]
    CFG["rate_on"] = False          # pure open loop - we are measuring, not driving
    CFG["hold_on"] = False
    try:
        HOLD.reset()
        wait_fresh()
        time.sleep(0.15)
        wait_fresh()
        c0l, c0r = odo_counts()
        h0 = HDG["deg"]
        g0, m0 = HDG["gyro"], HDG["ymag"]
        s = 1.0 if ccw else -1.0
        t0 = time.time()
        while not AUTO["abort"]:
            left = secs - (time.time() - t0)
            AUTO["step"] = ("spinning %s - press PASSED THE MARK each time it "
                            "comes round (%d so far, %.0fs left)"
                            % ("LEFT" if ccw else "RIGHT", LAP["n"],
                               max(0.0, left)))
            if left <= 0:
                break
            _spin(-s * rate, s * rate)
            time.sleep(0.02)
        _halt()
        AUTO["step"] = "settling"
        time.sleep(1.4)
        k = counts_to_m()
        cl, cr = odo_counts()
        dl, dr = (cl - c0l) * k, (cr - c0r) * k
        gyro = HDG["deg"] - h0
        asym = (abs(abs(dr) - abs(dl)) / max(abs(dl), abs(dr), 1e-6)) * 100.0
        AUTO["res"] = {"kind": "spin", "secs": secs,
                       "gyro_deg": round(gyro, 2),
                       "mag_deg": round(HDG["ymag"] - m0, 2),
                       "encl_m": round(dl, 4), "encr_m": round(dr, 4),
                       "diff_m": round(dr - dl, 4),
                       "asym": round(asym, 1),
                       "slow": 1 if HDG["slow"] else 0}
        bits = ["spun for %.1f s. Gyro says %+.0f deg, compass says %+.0f deg, "
                "and the wheels differed by %.3f m."
                % (secs, gyro, HDG["ymag"] - m0, dr - dl)]
        if asym > 15:
            bits.append("The wheels travelled %.0f%% different amounts (%.3f vs "
                        "%.3f m), so this was not a clean pivot - one wheel is "
                        "slipping or is weaker. Track width from this will be "
                        "rough." % (asym, abs(dl), abs(dr)))
        if HDG["slow"]:
            bits.append("The gyro is still arriving too slowly to measure a fast "
                        "spin, so do not trust a gyro scale taken from this.")
        bits.append("Now read the floor mark and enter how far it ACTUALLY turned.")
        AUTO["msg"] = " ".join(bits)
    finally:
        CFG["rate_on"], CFG["hold_on"] = keep_r, keep_h


def r_drift(a):
    """Reproduce the symptom: pivot, then go straight and measure the pull."""
    turn = _f(a, "turn", 90.0)      # + = to the right
    dist = _f(a, "dist", 1.5)
    speed = _f(a, "speed", 0.25)
    hold = _f(a, "hold", 1.0) > 0.5
    keep_h, keep_r = CFG["hold_on"], CFG["rate_on"]
    try:
        # 1 - open-loop pivot, exactly like the stick would do it
        CFG["hold_on"] = False
        CFG["rate_on"] = False
        HOLD.reset()
        time.sleep(0.15)
        h0 = HDG["deg"]
        s = -1.0 if turn > 0 else 1.0
        t0 = time.time()
        while not AUTO["abort"] and abs(HDG["deg"] - h0) < abs(turn) \
                and time.time() - t0 < 15:
            AUTO["step"] = "turning %.0f / %.0f deg" % (abs(HDG["deg"] - h0), abs(turn))
            _spin(-s * 0.26, s * 0.26)
            time.sleep(0.02)
        _halt()
        AUTO["step"] = "settling"
        time.sleep(1.0)

        # 2 - straight leg, with heading hold on or off
        CFG["hold_on"] = hold
        CFG["rate_on"] = keep_r
        HOLD.reset()
        time.sleep(0.15)
        h1 = HDG["deg"]
        c0l, c0r = odo_counts()
        k = counts_to_m()
        dev_max = 0.0
        t0 = time.time()
        cap = dist / max(speed, 0.05) * 3.0 + 8.0
        while not AUTO["abort"]:
            cl, cr = odo_counts()
            trav = ((cl - c0l) + (cr - c0r)) / 2.0 * k
            dev = wrap180(HDG["deg"] - h1)
            if abs(dev) > abs(dev_max):
                dev_max = dev
            AUTO["step"] = "straight %.2f / %.2f m   drift %+.1f deg" % (trav, dist, dev)
            if trav >= dist or time.time() - t0 > cap:
                break
            _drive(speed, 0.0)
            time.sleep(0.02)
        _halt()
        time.sleep(0.9)
        dev = wrap180(HDG["deg"] - h1)
        res = {"kind": "drift", "hold": 1 if hold else 0, "dev": round(dev, 2),
               "dev_max": round(dev_max, 2),
               "turned": round(HDG["deg"] - h0, 1), "dist": dist}
        AUTO["res"] = res
        AUTO["last"]["drift_on" if hold else "drift_off"] = res
        AUTO["msg"] = ("hold %s: pulled %+.1f deg over %.1f m (peak %+.1f). "
                       "Positive = drifted left, negative = drifted right."
                       % ("ON" if hold else "OFF", dev, dist, dev_max))
    finally:
        CFG["hold_on"], CFG["rate_on"] = keep_h, keep_r


def r_sign(a):
    """Pivot LEFT briefly and see which way the gyro says we went. If it
    disagrees, flip the sign automatically - this is the single most common
    reason heading hold drives the robot into a spin."""
    rate = _f(a, "rate", 0.22)
    secs = _f(a, "secs", 1.2)
    keep_r, keep_h = CFG["rate_on"], CFG["hold_on"]
    CFG["rate_on"] = CFG["hold_on"] = False
    try:
        HOLD.reset()
        _halt()
        AUTO["step"] = "settling"
        time.sleep(0.5)
        h0 = HDG["deg"]
        t0 = time.time()
        while time.time() - t0 < secs and not AUTO["abort"]:
            AUTO["step"] = "pivoting LEFT - it should turn anticlockwise"
            _spin(-rate, rate)          # right wheel forward = left turn
            time.sleep(0.02)
        _halt()
        AUTO["step"] = "settling"
        time.sleep(0.9)
        moved = HDG["deg"] - h0
        if abs(moved) < 8.0:
            AUTO["ok"] = False
            AUTO["msg"] = ("the gyro barely moved (%.1f deg). If the robot did "
                           "physically turn then gz is not reaching the Pi. If it "
                           "did not turn, put it on the floor or raise the pivot "
                           "speed." % moved)
            return
        if moved < 0:
            CFG["hdg_inv"] = not CFG["hdg_inv"]
            HDG["gyro"] = HDG["ymag"] = HDG["fuse"] = HDG["deg"] = 0.0
            HDG["bhat"] = 0.0          # learnt in the old sign, now meaningless
            HDG["_ly"] = None
            AUTO["msg"] = ("the gyro was counting backwards (%.0f deg for a LEFT "
                           "turn) - sign flipped for you. Heading now goes UP when "
                           "the robot turns left." % moved)
        else:
            AUTO["msg"] = ("gyro direction is correct: a LEFT turn read %+.0f deg."
                           % moved)
        CFG["sign_ok"] = True
        HDG["warn"] = ""
        save_cfg()
    finally:
        CFG["rate_on"], CFG["hold_on"] = keep_r, keep_h


def r_axes(a):
    """Motors stay off. You tilt the robot nose-up and hold it; whichever of the
    board's two tilt numbers moved is the robot's pitch, and its direction tells
    us the sign. Fixes a visualiser that rolls when you pitch it."""
    secs = _f(a, "secs", 8.0)
    _halt()
    # Wait for one genuinely fresh attitude frame first: starting from a stale
    # reading makes the very first sample look like a huge movement and picks
    # the wrong axis.
    seq0 = ATT["seq"]
    twait = time.time()
    while ATT["seq"] == seq0 and time.time() - twait < 2.0 and not AUTO["abort"]:
        AUTO["step"] = "waiting for tilt data from the board"
        time.sleep(0.02)
    if ATT["seq"] == seq0:
        AUTO["ok"] = False
        AUTO["msg"] = ("no roll/pitch data is arriving from the board, so the "
                       "axes cannot be worked out.")
        return
    # sample from the first instant after that - if we waited to settle, somebody
    # who lifts the nose straight away would show no movement at all
    r0, p0 = ATT["rawr"], ATT["rawp"]
    rmin = rmax = r0
    pmin = pmax = p0
    t0 = time.time()
    while time.time() - t0 < secs and not AUTO["abort"]:
        r, pi = ATT["rawr"], ATT["rawp"]
        rmin, rmax = min(rmin, r), max(rmax, r)
        pmin, pmax = min(pmin, pi), max(pmax, pi)
        AUTO["step"] = ("START FLAT, then LIFT THE NOSE - %.0fs left  "
                        "(roll moved %.0f, pitch moved %.0f)"
                        % (max(0.0, secs - (time.time() - t0)),
                           rmax - rmin, pmax - pmin))
        time.sleep(0.05)
    # take the extreme furthest from where we started, so it still works if the
    # nose gets put back down before the countdown finishes
    dr = (rmax - r0) if abs(rmax - r0) >= abs(rmin - r0) else (rmin - r0)
    dp = (pmax - p0) if abs(pmax - p0) >= abs(pmin - p0) else (pmin - p0)
    spanr, spanp = rmax - rmin, pmax - pmin
    if max(spanr, spanp) < 6.0:
        AUTO["ok"] = False
        AUTO["msg"] = ("neither tilt number moved more than %.1f deg - did you "
                       "lift the nose? Hold it up until the countdown ends."
                       % max(spanr, spanp))
        return
    if spanp >= spanr:
        CFG["imu_map"] = "rp"          # firmware pitch really is robot pitch
        CFG["pitch_inv"] = dp < 0      # nose up must read positive
        which, moved = "pitch", dp
    else:
        CFG["imu_map"] = "pr"          # they are swapped
        CFG["pitch_inv"] = dr < 0
        which, moved = "roll", dr
    save_cfg()
    AUTO["msg"] = ("lifting the nose moved the board's %s by %+.1f deg "
                   "(the other moved %.1f), so mapping is now '%s'%s. Put the "
                   "robot back down flat and press SET LEVEL."
                   % (which, moved, min(spanr, spanp), CFG["imu_map"],
                      " with pitch inverted" if CFG["pitch_inv"] else ""))


def _pulse(l, r, secs, label):
    """Short open-loop pulse with everything closed-loop switched off, and the
    encoder / gyro deltas it produced."""
    keep_h, keep_r = CFG["hold_on"], CFG["rate_on"]
    CFG["hold_on"] = CFG["rate_on"] = False
    try:
        HOLD.reset()
        _halt()
        time.sleep(0.4)
        wait_fresh()
        c0l, c0r = odo_counts()
        g0, m0 = HDG["gyro"], HDG["ymag"]
        t0 = time.time()
        while time.time() - t0 < secs and not AUTO["abort"]:
            AUTO["step"] = label
            _spin(l, r)
            time.sleep(0.02)
        _halt()
        time.sleep(0.8)
        cl, cr = odo_counts()
        k = counts_to_m()
        return {"dl": (cl - c0l) * k, "dr": (cr - c0r) * k,
                "dg": HDG["gyro"] - g0, "dm": HDG["ymag"] - m0}
    finally:
        CFG["hold_on"], CFG["rate_on"] = keep_h, keep_r


def r_check(a):
    """The whole drivetrain sanity sweep: is the gyro arriving, do the motors
    drive the way they are told, do the encoders count the way the wheels turn,
    are left and right the right way round, and does the gyro agree."""
    v = _f(a, "v", 0.22)
    secs = _f(a, "secs", 0.7)
    fix = []
    rep = []

    # --- gyro present and fast enough -----------------------------------
    hdg_health()
    if not HDG["ok"]:
        rep.append("GYRO: no gz arriving at all - nothing below can be judged")
    elif HDG["slow"]:
        rep.append("GYRO: arriving at only %.1f Hz - reflash the firmware so gz "
                   "goes in the fast stream" % HDG["hz"])
    else:
        rep.append("GYRO: ok, %.0f Hz" % HDG["hz"])

    # --- forwards ---------------------------------------------------------
    f = _pulse(v, v, secs, "driving FORWARD a little")
    if AUTO["abort"]:
        return
    rep.append("FORWARD: left wheel %+.3f m, right %+.3f m, turned %+.1f deg"
               % (f["dl"], f["dr"], f["dg"]))
    if f["dl"] < -0.005 and f["dr"] < -0.005:
        rep.append("  -> both encoders count backwards when driving forward")
        fix.append(("encL", 1)); fix.append(("encR", 1))
    elif f["dl"] < -0.005:
        rep.append("  -> the LEFT encoder counts backwards")
        fix.append(("encL", 1))
    elif f["dr"] < -0.005:
        rep.append("  -> the RIGHT encoder counts backwards")
        fix.append(("encR", 1))
    elif abs(f["dl"]) < 0.005 and abs(f["dr"]) < 0.005:
        rep.append("  -> neither encoder moved: is it on the floor, is it powered?")
    else:
        rep.append("  -> encoders agree with forward")

    # --- pivot left -------------------------------------------------------
    p = _pulse(-v, v, secs, "pivoting LEFT a little")
    if AUTO["abort"]:
        return
    rep.append("PIVOT LEFT: left wheel %+.3f m, right %+.3f m, gyro %+.1f deg, "
               "compass %+.1f deg" % (p["dl"], p["dr"], p["dg"], p["dm"]))
    swapped = p["dl"] > 0.005 and p["dr"] < -0.005
    if swapped:
        rep.append("  -> left and right are the other way round on the board")
        fix.append(("swap", 1 if not GEO["swap"] else 0))
    elif p["dr"] > 0.005 and p["dl"] < -0.005:
        rep.append("  -> left/right wiring is correct")
    else:
        rep.append("  -> the wheels did not counter-rotate; check the motors")

    truth = -1.0 if swapped else 1.0        # which way it PHYSICALLY went
    if HDG["ok"] and abs(p["dg"]) > 6.0:
        if (p["dg"] > 0) != (truth > 0):
            rep.append("  -> the gyro disagrees with the direction it really "
                       "turned: gyro sign is inverted")
            fix.append(("hdg_inv", 0 if CFG["hdg_inv"] else 1))
        else:
            rep.append("  -> gyro direction agrees")
        if abs(p["dm"]) > 6.0:
            ratio = abs(p["dg"]) / max(abs(p["dm"]), 1e-6)
            rep.append("  -> gyro read %.2fx what the compass read over the same "
                       "turn%s" % (ratio,
                       " - one of them is badly scaled" if ratio < 0.6 or ratio > 1.7
                       else ""))
    elif HDG["ok"]:
        rep.append("  -> the gyro barely registered a pivot (%.1f deg) - it is "
                   "not keeping up" % p["dg"])

    AUTO["res"] = {"kind": "check", "text": chr(10).join(rep),
                   "fix": [list(x) for x in fix]}
    AUTO["msg"] = ("check finished - %d thing%s to fix. Read the list, then press "
                   "APPLY FIXES." % (len(fix), "" if len(fix) == 1 else "s")) \
        if fix else "check finished - everything agrees."


ROUTINES = {"sign": r_sign, "bias": r_bias, "straight": r_straight,
            "spin": r_spin, "drift": r_drift, "axes": r_axes, "check": r_check}


def apply_fixes():
    """Push whatever r_check found: board fields over serial, gyro sign locally."""
    fix = AUTO["res"].get("fix") or []
    if not fix:
        return {"ok": True, "msg": "nothing to apply"}
    board, done = {}, []
    for k, val in fix:
        if k == "hdg_inv":
            CFG["hdg_inv"] = bool(val)
            hdg_zero()
            done.append("gyro sign flipped")
        else:
            board[k] = int(val)
            done.append("%s=%d on the board" % (k, val))
    if board and ESP is not None:
        board["T"] = 700
        board["save"] = 1
        ESP.send(board)
        ESP.send({"T": 701})
        if "swap" in board:
            # the stick inversion was compensating for the old wiring
            CFG["steer_inv"] = not CFG["steer_inv"]
            done.append("stick inversion undone to match")
    save_cfg()
    AUTO["res"]["fix"] = []
    return {"ok": True, "msg": "applied: " + ", ".join(done), "cfg": CFG}


# ---- lap counting ---------------------------------------------------------
# Nobody can eyeball 1280 degrees. But anybody can see the robot arrive back at
# a strip of tape. So: press once when it is on the mark, press again every time
# it comes back, and the true rotation is laps x 360 exactly - no counting, no
# estimating how far past the mark it stopped. A consistent reaction delay on
# the presses cancels out, because both ends of the interval are delayed alike.
LAP = {"n": 0, "first": None, "last": None}


def _snap():
    cl, cr = odo_counts()
    return {"cl": cl, "cr": cr, "g": HDG["gyro"], "m": HDG["ymag"],
            "t": time.time()}


def lap_reset():
    LAP.update({"n": 0, "first": None, "last": None})
    return {"ok": True, "laps": 0, "msg": "lap count cleared"}


def lap_mark():
    wait_fresh()
    snap = _snap()
    if LAP["first"] is None:
        LAP.update({"n": 0, "first": snap, "last": snap})
        msg = ("start point recorded. Now press this again every time the robot "
               "comes back round to the mark.")
    else:
        LAP["n"] += 1
        LAP["last"] = snap
        msg = ("%d full turn%s recorded (%d deg). Keep going, or press FINISH."
               % (LAP["n"], "" if LAP["n"] == 1 else "s", LAP["n"] * 360))
    return {"ok": True, "laps": LAP["n"], "msg": msg}


def lap_finish():
    if LAP["first"] is None or LAP["n"] < 1:
        return {"ok": False, "msg": "mark the start, then mark at least one full "
                                    "turn before finishing"}
    a, b = LAP["first"], LAP["last"]
    deg = LAP["n"] * 360.0
    k = counts_to_m()
    dl, dr = (b["cl"] - a["cl"]) * k, (b["cr"] - a["cr"]) * k
    gyro = b["g"] - a["g"]
    mag = b["m"] - a["m"]
    asym = (abs(abs(dr) - abs(dl)) / max(abs(dl), abs(dr), 1e-6)) * 100.0
    AUTO["res"] = {"kind": "spin", "laps": LAP["n"], "auto_deg": deg,
                   "secs": round(b["t"] - a["t"], 1),
                   "gyro_deg": round(gyro, 2), "mag_deg": round(mag, 2),
                   "encl_m": round(dl, 4), "encr_m": round(dr, 4),
                   "diff_m": round(dr - dl, 4), "asym": round(asym, 1),
                   "slow": 1 if HDG["slow"] else 0}
    geom = CFG["tw_geom"] if CFG["tw_geom"] > 0.02 else 0.175
    tw = abs(dr - dl) / math.radians(deg)
    bits = ["%d full turn%s = %.0f deg in %.1f s. The gyro reported %+.0f deg, "
            "so it is reading %.2fx the truth. The wheel difference of %.3f m "
            "puts the effective track width at %.0f mm (%.2fx your measured %.0f)."
            % (LAP["n"], "" if LAP["n"] == 1 else "s", deg,
               b["t"] - a["t"], gyro, abs(gyro) / deg, dr - dl,
               tw * 1000, tw / geom, geom * 1000)]
    if asym > 15:
        bits.append("The wheels travelled %.0f%% different amounts, so this was "
                    "not a clean pivot - one is slipping." % asym)
    if HDG["slow"]:
        bits.append("The gyro is still arriving too slowly to trust a scale from "
                    "a spin; reflash first.")
    bits.append("Press the APPLY buttons - the rotation is already known, you do "
                "not need to type anything.")
    AUTO["msg"] = " ".join(bits)
    LAP.update({"first": None, "last": None, "n": 0})
    return {"ok": True, "msg": AUTO["msg"], "res": AUTO["res"]}


# ---- measure from a manoeuvre you drive yourself -------------------------
MARK = {"set": False}


def mark_now():
    wait_fresh()
    cl, cr = odo_counts()
    MARK.update({"set": True, "cl": cl, "cr": cr, "g": HDG["gyro"],
                 "m": HDG["ymag"], "h": HDG["deg"], "t": time.time()})
    return {"ok": True, "msg": "marked - now drive it yourself, then come back"}


def measure_now(deg, metres):
    if not MARK["set"]:
        return {"ok": False, "msg": "press MARK START first"}
    wait_fresh()
    cl, cr = odo_counts()
    k = counts_to_m()
    dl, dr = (cl - MARK["cl"]) * k, (cr - MARK["cr"]) * k
    res = {"kind": "manual",
           "enc_m": round((dl + dr) / 2.0, 4),
           "encl_m": round(dl, 4), "encr_m": round(dr, 4),
           "diff_m": round(dr - dl, 4),
           "gyro_deg": round(HDG["gyro"] - MARK["g"], 2),
           "mag_deg": round(HDG["ymag"] - MARK["m"], 2),
           "secs": round(time.time() - MARK["t"], 1)}
    AUTO["res"] = res
    bits = ["over %.1f s the encoders saw %+.3f m of travel (L %+.3f / R %+.3f), "
            "the wheels differed by %.3f m, the gyro turned %+.1f deg and the "
            "compass %+.1f deg"
            % (res["secs"], res["enc_m"], dl, dr, res["diff_m"],
               res["gyro_deg"], res["mag_deg"])]
    if abs(deg) > 20:
        g = abs(res["gyro_deg"])
        if g > 5:
            bits.append("gyro is reading %.2fx the truth" % (g / abs(deg)))
        tw = abs(res["diff_m"]) / math.radians(abs(deg))
        bits.append("encoders put the effective track width at %.1f mm "
                    "(board has %.1f)" % (tw * 1000, GEO["tw"] * 1000))
    if metres > 0.05:
        bits.append("encoders are reading %.2fx the real distance"
                    % (res["enc_m"] / metres))
    AUTO["msg"] = ". ".join(bits)
    MARK["set"] = False
    return {"ok": True, "msg": AUTO["msg"], "res": res}


def _auto_body(name, a):
    try:
        ROUTINES[name](a)
    except Exception as e:
        AUTO["msg"] = "failed: %s" % e
        AUTO["ok"] = False
    finally:
        _halt()
        if AUTO["abort"]:
            # a half-finished measurement must not be appliable
            AUTO["res"] = {}
            AUTO["msg"] = "stopped - nothing measured, nothing applied"
        AUTO["busy"] = False
        AUTO["step"] = "aborted" if AUTO["abort"] else "done"
        AUTO["abort"] = False


def run_auto(name, a):
    if name not in ROUTINES:
        return {"ok": False, "msg": "no such routine"}
    if AUTO["busy"]:
        return {"ok": False, "msg": "%s already running" % AUTO["name"]}
    if ESP is None:
        return {"ok": False, "msg": "no link to the board"}
    AUTO.update({"busy": True, "name": name, "step": "starting", "msg": "",
                 "res": {}, "abort": False, "ok": True})
    threading.Thread(target=_auto_body, args=(name, a), daemon=True).start()
    return {"ok": True, "name": name}


def cal_apply(what, val):
    """Turn a measured truth into a stored constant."""
    msg = "nothing to apply"
    res = AUTO["res"]
    # a lap-counted spin already knows the true angle exactly
    if what in ("tw", "scale") and abs(val) < 20 and res.get("auto_deg"):
        val = float(res["auto_deg"])
    if what == "bias":
        if "bias" in res:
            CFG["gyro_bias"] = res["bias"]
            msg = "gyro bias set to %+.4f deg/s" % CFG["gyro_bias"]
    elif what == "drift":
        # bhat is in corrected deg/s; undo scale and sign to get raw units
        b = HDG["bhat"]
        if abs(b) > 0.005:
            sgn = -1.0 if CFG["hdg_inv"] else 1.0
            CFG["gyro_bias"] = round(
                CFG["gyro_bias"] + b / (CFG["gyro_scale"] * sgn), 5)
            HDG["bhat"] = 0.0
            msg = ("folded %+.3f deg/s of learnt drift into the gyro bias "
                   "(now %+.4f)" % (b, CFG["gyro_bias"]))
        else:
            msg = "the filter has not learnt any drift worth applying yet"
    elif what == "wd":
        enc = float(res.get("enc_m", 0.0))
        if enc > 0.05 and val > 0.05:
            old = GEO["wd"]
            GEO["wd"] = round(old * val / enc, 5)
            if ESP is not None:
                ESP.send({"T": 700, "wd": GEO["wd"], "save": 1})
            msg = ("wheel diameter %.1f -> %.1f mm (encoders were %+.1f%% out)"
                   % (old * 1000, GEO["wd"] * 1000, (enc / val - 1) * 100))
    elif what == "scale":
        g = abs(float(res.get("gyro_deg", 0.0)))
        if g > 20 and abs(val) > 20:
            old = CFG["gyro_scale"]
            CFG["gyro_scale"] = round(old * abs(val) / g, 5)
            msg = "gyro scale %.4f -> %.4f" % (old, CFG["gyro_scale"])
            if res.get("slow"):
                msg += ("  - but the gyro was arriving too slowly during that "
                        "spin, so this will be wrong once the firmware is fixed. "
                        "Redo it afterwards.")
    elif what == "tw":
        d = abs(float(res.get("diff_m", 0.0)))
        if d > 0.01 and abs(val) > 20:
            old = GEO["tw"]
            new = d / math.radians(abs(val))
            geom = CFG["tw_geom"] if CFG["tw_geom"] > 0.02 else 0.175
            ratio = new / geom
            if not 0.90 <= ratio <= 1.50:
                msg = ("that works out to %.0f mm, which is %.2fx your measured "
                       "axle spacing of %.0f mm - not believable, so nothing was "
                       "changed. Scrub can only make the effective track width a "
                       "little BIGGER than the real one - with a caster, within "
                       "about a third. %.4f m of wheel difference needs about %.0f "
                       "deg of rotation to land on %.0f mm, so check the rotation "
                       "you entered (and that the wheels were not slipping)."
                       % (new * 1000, ratio, geom * 1000, d,
                          math.degrees(d / geom), geom * 1000))
            else:
                GEO["tw"] = round(new, 5)
                if ESP is not None:
                    ESP.send({"T": 700, "tw": GEO["tw"], "save": 1})
                msg = ("effective track width %.1f -> %.1f mm"
                       % (old * 1000, GEO["tw"] * 1000))
    save_cfg()
    if ESP is not None:
        ESP.send({"T": 701})
    return {"ok": True, "msg": msg, "cfg": CFG, "geo": GEO}


# --------------------------------------------------------------- web
PAGE = """<!DOCTYPE html><html><head><meta charset="utf-8">
<title>lidar-bot</title><meta name="viewport" content="width=device-width,initial-scale=1">
<style>
*{box-sizing:border-box}
body{margin:0;background:#12141c;color:#c9d1e0;font-family:system-ui,sans-serif}
.wrap{display:flex;flex-wrap:wrap;gap:14px;padding:14px;max-width:1100px;margin:0 auto}
.card{background:#1b1f2b;border-radius:12px;padding:14px;flex:1 1 260px}
.big{font-size:30px;color:#fff;line-height:1.1}
.lab{font-size:11px;opacity:.6;letter-spacing:.05em}
.row{display:flex;gap:18px;flex-wrap:wrap}
canvas{width:100%;background:#0c0e14;border-radius:10px;display:block}
#ovl{background:transparent}   /* the overlay must not hide the video */
.pill{display:inline-block;padding:3px 10px;border-radius:20px;font-size:12px;font-weight:600}
.on{background:#2a7;color:#041} .off{background:#444;color:#bbb} .warn{background:#a33;color:#fff}
.bar{height:10px;background:#0c0e14;border-radius:5px;overflow:hidden;margin-top:6px}
.fill{height:100%;background:#4a8;width:0}
h2{font-size:13px;margin:0 0 10px;opacity:.75;font-weight:600;letter-spacing:.06em}
</style></head><body>
<div class="wrap">
  <div class="card" style="flex:1 1 100%;display:flex;align-items:center;gap:14px">
    <div style="flex:1 1 auto"><b>lidar-bot</b></div>
    <span class="lab" id="hdgTop" style="margin:0"></span>
    <a href="/calib" style="color:#6ad">calibration &rarr;</a>
  </div>
  <div class="card" style="flex:1 1 420px">
    <h2>LIDAR</h2>
    <canvas id="lidar" width="520" height="520"></canvas>
    <div class="lab" style="margin-top:8px"><span id="lrpm">--</span> rpm &middot; <span id="lpts">0</span> points &middot; range <span id="lrange">6</span> m
      <button onclick="zoom(0.5)">+</button><button onclick="zoom(2)">-</button></div>
  </div>

  <div class="card" style="flex:1 1 420px">
    <h2>CAMERA</h2>
    <div style="position:relative;line-height:0">
      <img id="cam" src="/stream.mjpg" style="width:100%;border-radius:10px;background:#0c0e14;display:block">
      <canvas id="ovl" style="position:absolute;left:0;top:0;width:100%;height:100%;
        pointer-events:none;border-radius:10px;background:transparent!important"></canvas>
    </div>
    <div class="lab" style="margin-top:6px">
      <span id="camStat">--</span>
      <span id="camErr" style="color:#f88"></span>
    </div>

    <div class="row" style="margin-top:10px;align-items:center;gap:10px">
      <label class="lab">RES
        <select id="camRes" onchange="camRes()" style="margin-left:4px">
          <option value="320x240">320x240</option>
          <option value="640x480" selected>640x480</option>
          <option value="800x600">800x600</option>
          <option value="1280x720">1280x720</option>
          <option value="1920x1080">1920x1080</option>
        </select>
      </label>
      <label class="lab">CAM # <span style="opacity:.6">(-1 = auto)</span>
        <input type="number" id="camIdx" value="0" min="-1" max="9" style="width:52px;margin-left:4px"
               onchange="setCam('index', this.value)">
      </label>
      <button class="pill off" style="border:0;cursor:pointer"
              onclick="document.getElementById('camIdx').value=-1;setCam('index',-1)">AUTO</button>
      <button class="pill off" style="border:0;cursor:pointer" onclick="camToggle()" id="camOn">ON</button>
      <button class="pill off" style="border:0;cursor:pointer" onclick="camFlip()">FLIP</button>
    </div>

    <label class="lab">FPS <b id="vFps">15</b></label>
    <input type="range" id="sFps" min="1" max="30" step="1" style="width:100%"
           oninput="setCam('fps', this.value)">

    <label class="lab">JPEG QUALITY <b id="vQual">60</b> &mdash; higher = sharper &amp; more bandwidth</label>
    <input type="range" id="sQual" min="10" max="95" step="5" style="width:100%"
           oninput="setCam('quality', this.value)">

    <div class="row" style="margin-top:10px;align-items:center">
      <button id="ovlBtn" onclick="S('ovl_on', D.cfg.ovl_on?0:1)">LIDAR OVERLAY</button>
      <button onclick="S('ovl_glow', D.cfg.ovl_glow?0:1)">GLOW</button>
      <button onclick="S('ovl_labels', D.cfg.ovl_labels?0:1)">RANGE LABELS</button>
      <button onclick="S('ovl_horizon', D.cfg.ovl_horizon?0:1)">AIM GUIDES</button>
    </div>
    <details><summary style="margin-top:10px;cursor:pointer;font-size:12px;opacity:.7">OVERLAY CALIBRATION</summary>
      <div class="lab" style="margin:8px 0;line-height:1.6;opacity:.8">
        Point the robot square at a wall with a doorway or a box on the floor, turn
        on AIM GUIDES, then in order:
        <b>1</b> FIELD OF VIEW to whatever your webcam claims (most USB cams are
        65-78&deg;). <b>2</b> LIDAR YAW until the contour lines up left-to-right with
        the real edges. <b>3</b> HEIGHT DIFFERENCE = lidar height minus camera
        height, off a ruler. <b>4</b> PITCH until the contour sits on the floor line.
      </div>
      <div id="ovlSliders"></div>
      <div class="row" style="margin-top:6px">
        <button onclick="S('ovl_flip', D.cfg.ovl_flip?0:1)">FLIP LIDAR DIRECTION</button>
        <span class="lab" id="ovlFlipTxt"></span>
      </div>
    </details>

    <div class="row" style="margin-top:8px">
      <button onclick="camPreset(320,240,10,40)" class="pill off" style="border:0;cursor:pointer">LOW BW</button>
      <button onclick="camPreset(640,480,15,60)" class="pill off" style="border:0;cursor:pointer">BALANCED</button>
      <button onclick="camPreset(1280,720,20,80)" class="pill warn" style="border:0;cursor:pointer">HIGH</button>
    </div>
  </div>

  <div style="flex:1 1 300px;display:flex;flex-direction:column;gap:14px">
    <div class="card">
      <h2>POWER</h2>
      <div class="row">
        <div><div class="big" id="pv">--</div><div class="lab">PACK VOLTS</div></div>
        <div><div class="big" id="pct">--</div><div class="lab">CHARGE</div></div>
      </div>
      <div class="row" style="margin-top:12px">
        <div><div class="big" id="amp">--</div><div class="lab">AMPS</div></div>
        <div><div class="big" id="rail">--</div><div class="lab">12V RAIL</div></div>
      </div>
      <div class="bar"><div class="fill" id="batbar"></div></div>
    </div>

    <div class="card">
      <h2>DRIVE</h2>
      <div class="row" style="align-items:center">
        <span class="pill off" id="arm">DISARMED</span>
        <span class="pill off" id="dir">IDLE</span>
      </div>
      <canvas id="wheel" width="300" height="150" style="margin-top:10px"></canvas>
      <div class="row" style="margin-top:8px">
        <div><div class="big" id="steer">0&deg;</div><div class="lab">STEERING</div></div>
        <div><div class="big" id="spd">0.00</div><div class="lab">m/s CMD</div></div>
      </div>
      <div class="lab" style="margin-top:8px">L <span id="wl">0.00</span> &nbsp; R <span id="wr">0.00</span>
        &nbsp;&middot;&nbsp; pad: <span id="pad">--</span></div>
    </div>

    <div class="card">
      <h2>TUNING</h2>
      <div class="row" style="margin-bottom:10px">
        <button id="mOpen"  onclick="setMode(1)" class="pill off" style="border:0;cursor:pointer">OPEN LOOP</button>
        <button id="mClosed" onclick="setMode(3)" class="pill off" style="border:0;cursor:pointer">CLOSED LOOP (PID)</button>
      </div>
      <div class="lab" id="modeHint" style="margin-bottom:12px">&nbsp;</div>

      <label class="lab">MAX SPEED <b id="vMax">0.45</b></label>
      <input type="range" id="sMax" min="0.05" max="1.5" step="0.05" style="width:100%"
             oninput="setCfg('max_lin', this.value)">

      <label class="lab">TURN SHARPNESS <b id="vTurn">0.60</b></label>
      <input type="range" id="sTurn" min="0.05" max="2" step="0.05" style="width:100%"
             oninput="setCfg('turn_gain', this.value)">

      <label class="lab">PIVOT SPEED (on the spot) <b id="vPiv">0.30</b></label>
      <input type="range" id="sPiv" min="0.05" max="1.5" step="0.05" style="width:100%"
             oninput="setCfg('pivot', this.value)">

      <label class="lab">NORMAL SCALE (no boost) <b id="vNorm">0.60</b></label>
      <input type="range" id="sNorm" min="0.1" max="1" step="0.05" style="width:100%"
             oninput="setCfg('normal', this.value)">

      <div class="row" style="margin-top:10px">
        <button onclick="preset(0.35,0.5,0.25)" class="pill off" style="border:0;cursor:pointer">CALM</button>
        <button onclick="preset(0.5,0.7,0.4)"  class="pill off" style="border:0;cursor:pointer">NORMAL</button>
        <button onclick="preset(1.0,1.2,0.8)"  class="pill warn" style="border:0;cursor:pointer">AGGRESSIVE</button>
      </div>
    </div>

    <div class="card">
      <h2>IMU</h2>
      <div class="row">
        <div><div class="big" id="yaw">--</div><div class="lab">YAW</div></div>
        <div><div class="big" id="temp">--</div><div class="lab">ESP &deg;C</div></div>
      </div>
    </div>
  </div>
</div>

<script>
var RANGE = 6000;                       // mm shown across the radius
function zoom(f){ RANGE = Math.max(1000, Math.min(20000, RANGE*f));
  document.getElementById('lrange').textContent = (RANGE/1000).toFixed(0); }

function drawLidar(scan){
  var c = document.getElementById('lidar'), g = c.getContext('2d');
  var W = c.width, H = c.height, cx = W/2, cy = H/2, R = Math.min(cx,cy) - 8;
  g.clearRect(0,0,W,H);

  g.strokeStyle = '#1e2432'; g.lineWidth = 1;
  for (var m = 1; m <= 4; m++){
    g.beginPath(); g.arc(cx, cy, R*m/4, 0, 6.284); g.stroke();
  }
  g.beginPath(); g.moveTo(cx,cy-R); g.lineTo(cx,cy+R);
  g.moveTo(cx-R,cy); g.lineTo(cx+R,cy); g.stroke();

  g.fillStyle = '#556'; g.font = '10px sans-serif';
  for (var m2 = 1; m2 <= 4; m2++) g.fillText((RANGE*m2/4000).toFixed(1)+'m', cx+4, cy - R*m2/4 + 12);

  g.fillStyle = '#5cf';
  for (var i=0;i<scan.length;i++){
    var a = scan[i][0], d = scan[i][1];
    if (d > RANGE) continue;
    var rad = (a - 90) * Math.PI / 180;      // 0 deg = forward = up
    var x = cx + Math.cos(rad) * (d/RANGE) * R;
    var y = cy + Math.sin(rad) * (d/RANGE) * R;
    g.fillRect(x-1.5, y-1.5, 3, 3);
  }
  // robot
  g.fillStyle = '#fc6';
  g.beginPath(); g.moveTo(cx, cy-9); g.lineTo(cx-6, cy+7); g.lineTo(cx+6, cy+7);
  g.closePath(); g.fill();
}

function drawWheels(l, r, max){
  var c = document.getElementById('wheel'), g = c.getContext('2d');
  g.clearRect(0,0,c.width,c.height);
  var mid = c.height/2;
  function bar(x, v, label){
    var h = Math.max(-1, Math.min(1, v/max)) * (mid-14);
    g.fillStyle = '#0c0e14'; g.fillRect(x, 10, 54, c.height-20);
    g.fillStyle = v >= 0 ? '#4a8' : '#a55';
    g.fillRect(x, mid, 54, -h);
    g.strokeStyle='#2a3040'; g.beginPath(); g.moveTo(x,mid); g.lineTo(x+54,mid); g.stroke();
    g.fillStyle='#889'; g.font='11px sans-serif'; g.fillText(label, x+18, c.height-1);
  }
  bar(60, l, 'L'); bar(180, r, 'R');
}

var camTouched = 0;
function setCam(k, v){
  camTouched = Date.now();
  if (k === 'fps')     document.getElementById('vFps').textContent = v;
  if (k === 'quality') document.getElementById('vQual').textContent = v;
  fetch('/cam?'+k+'='+v);
}
function camRes(){
  var p = document.getElementById('camRes').value.split('x');
  camTouched = Date.now();
  fetch('/cam?width='+p[0]+'&height='+p[1]);
}
function camToggle(){
  var b = document.getElementById('camOn');
  var on = b.textContent === 'ON' ? 0 : 1;
  fetch('/cam?on='+on).then(()=>{
    if (on) document.getElementById('cam').src = '/stream.mjpg?'+Date.now();
  });
}
function camFlip(){ fetch('/cam?flip=' + (window._flip ? 0 : 1)); window._flip = !window._flip; }
function camPreset(w,h,f,q){
  camTouched = Date.now();
  document.getElementById('camRes').value = w+'x'+h;
  document.getElementById('sFps').value = f;  document.getElementById('vFps').textContent = f;
  document.getElementById('sQual').value = q; document.getElementById('vQual').textContent = q;
  fetch('/cam?width='+w+'&height='+h+'&fps='+f+'&quality='+q);
}

var D = {cfg:{}, scan:[]};
function g(id){ return document.getElementById(id); }
function S(k, v){ window._ovlTouch = Date.now(); return fetch('/set?'+k+'='+v); }

// ------------------------------------------- lidar overlay on the camera
// Each lidar return is a point on a horizontal plane at the lidar's height.
// Project it through a pinhole camera and you get a contour that traces the
// bottom of whatever the beam hit - walls, chair legs, boxes. Joining
// neighbouring returns turns the dots into edges that sit on the objects.
var OVL_STOPS = [[0.00,255,247,214],[0.15,255,210,74],[0.35,255,122,26],
                 [0.55,240,50,31],[0.75,160,26,122],[1.00,43,63,208]];
function heat(t){
  t = t<0?0:(t>1?1:t);
  for (var i=1;i<OVL_STOPS.length;i++){
    var b=OVL_STOPS[i], a=OVL_STOPS[i-1];
    if (t<=b[0]){
      var k=(t-a[0])/((b[0]-a[0])||1);
      return [Math.round(a[1]+(b[1]-a[1])*k),
              Math.round(a[2]+(b[2]-a[2])*k),
              Math.round(a[3]+(b[3]-a[3])*k)];
    }
  }
  var z=OVL_STOPS[OVL_STOPS.length-1]; return [z[1],z[2],z[3]];
}

function project(angDeg, rMetres, c, W, H){
  var yaw=(c.ovl_yaw||0), flip=c.ovl_flip?-1:1;
  var a=(angDeg*flip + yaw)*Math.PI/180;
  var x=rMetres*Math.cos(a), y=rMetres*Math.sin(a);   // forward, left
  var Xc=x-(c.ovl_dx||0), Yc=y, Zc=(c.ovl_dz||0);
  var p=(c.ovl_pitch||0)*Math.PI/180, cp=Math.cos(p), sp=Math.sin(p);
  var X= Xc*cp + Zc*sp;
  var Z=-Xc*sp + Zc*cp;
  if (X < 0.15) return null;                          // behind or too close
  var f=(W/2)/Math.tan((c.ovl_hfov||68)*Math.PI/360);
  return {u: W/2 - f*(Yc/X), v: H/2 - f*(Z/X), d: rMetres, x:x, y:y};
}

function drawOverlay(){
  var img=g('cam'), cv=g('ovl');
  if (!img||!cv) return;
  var W=img.clientWidth|0, H=img.clientHeight|0;
  if (W<10||H<10) return;
  if (cv.width!==W||cv.height!==H){ cv.width=W; cv.height=H; }
  var x=cv.getContext('2d');
  x.clearRect(0,0,W,H);
  var c=D.cfg||{};
  if (!c.ovl_on) return;

  var scan=D.scan||[], maxr=c.ovl_max||6, near=c.ovl_near||0.3, far=c.ovl_far||4;
  var gap=c.ovl_gap||0.25, lw=c.ovl_wide||3, al=(c.ovl_alpha===undefined?0.85:c.ovl_alpha);

  if (c.ovl_horizon){
    var hz=project(0, 1000, c, W, H);
    if (hz){ x.strokeStyle='rgba(120,140,180,.35)'; x.lineWidth=1;
      x.beginPath(); x.moveTo(0,hz.v); x.lineTo(W,hz.v); x.stroke();
      x.beginPath(); x.moveTo(W/2,0); x.lineTo(W/2,H); x.stroke(); }
  }

  var pts=[], i;
  for (i=0;i<scan.length;i++){
    var r=scan[i][1]/1000;
    if (r<0.05||r>maxr) { pts.push(null); continue; }
    var p=project(scan[i][0], r, c, W, H);
    if (p && p.u>-W && p.u<2*W) pts.push(p); else pts.push(null);
  }

  x.lineCap='round'; x.lineJoin='round';
  x.globalAlpha=al;
  for (var pass=0; pass<2; pass++){
    if (pass===0 && !c.ovl_glow) continue;
    x.lineWidth = pass===0 ? lw*3 : lw;
    x.globalAlpha = pass===0 ? al*0.22 : al;
    for (i=1;i<pts.length;i++){
      var a=pts[i-1], b=pts[i];
      if (!a||!b) continue;
      var sep=Math.hypot(b.x-a.x, b.y-a.y);
      if (sep>gap) continue;                    // different object - do not join
      var d=Math.min(a.d,b.d);
      var col=heat((d-near)/((far-near)||1));
      x.strokeStyle='rgb('+col[0]+','+col[1]+','+col[2]+')';
      x.beginPath(); x.moveTo(a.u,a.v); x.lineTo(b.u,b.v); x.stroke();
    }
  }
  x.globalAlpha=1;

  if (c.ovl_labels){
    x.font='11px system-ui'; x.textAlign='center';
    for (i=0;i<pts.length;i+=18){
      var p2=pts[i]; if(!p2) continue;
      var col2=heat((p2.d-near)/((far-near)||1));
      x.fillStyle='rgba(0,0,0,.55)'; x.fillRect(p2.u-15,p2.v-16,30,13);
      x.fillStyle='rgb('+col2[0]+','+col2[1]+','+col2[2]+')';
      x.fillText(p2.d.toFixed(1)+'m', p2.u, p2.v-6);
    }
  }

  // closest thing in view, bottom left
  var best=null;
  for (i=0;i<pts.length;i++){
    var q=pts[i];
    if (q && q.u>0 && q.u<W && (!best||q.d<best.d)) best=q;
  }
  x.font='bold 13px system-ui'; x.textAlign='left';
  x.fillStyle='rgba(0,0,0,.5)'; x.fillRect(8,H-30,150,22);
  if (best){
    var cb=heat((best.d-near)/((far-near)||1));
    x.fillStyle='rgb('+cb[0]+','+cb[1]+','+cb[2]+')';
    x.fillText('nearest in view  '+best.d.toFixed(2)+' m', 14, H-14);
  } else {
    x.fillStyle='#667'; x.fillText('no lidar returns in view', 14, H-14);
  }
}

var OVL_TUNE = [
  ['ovl_hfov',   30, 140,  0.5,  'FIELD OF VIEW, degrees'],
  ['ovl_yaw',  -180, 180,  0.5,  'LIDAR YAW vs camera centre, degrees'],
  ['ovl_pitch', -40,  40,  0.25, 'CAMERA PITCH, + is nose down'],
  ['ovl_dz',    -0.6, 0.6, 0.005,'HEIGHT DIFFERENCE, lidar minus camera, m'],
  ['ovl_dx',    -0.4, 0.4, 0.005,'LIDAR AHEAD OF CAMERA, m'],
  ['ovl_max',    0.5,  12, 0.25, 'DRAW OUT TO, m'],
  ['ovl_gap',   0.05, 1.5, 0.01, 'BREAK THE LINE IF POINTS ARE FURTHER APART, m'],
  ['ovl_near',     0,   3, 0.05, 'HEAT MAP HOT END, m'],
  ['ovl_far',    0.5,  12, 0.25, 'HEAT MAP COLD END, m'],
  ['ovl_wide',     1,  10, 0.5,  'LINE WIDTH'],
  ['ovl_alpha',  0.1,   1, 0.05, 'OPACITY']
];
(function(){
  var h='', t, i;
  for (i=0;i<OVL_TUNE.length;i++){
    t=OVL_TUNE[i];
    h += '<label class="lab" style="margin-top:8px;display:block">'+t[4]
      +  ' <b id="ov_'+t[0]+'">--</b></label>'
      +  '<input type="range" id="os_'+t[0]+'" min="'+t[1]+'" max="'+t[2]
      +  '" step="'+t[3]+'" style="width:100%" oninput="S(\\''+t[0]+'\\', this.value);'
      +  'g(\\'ov_'+t[0]+'\\').textContent=this.value">';
  }
  var e=g('ovlSliders'); if(e) e.innerHTML=h;
})();

var cfgTouched = 0;
function setCfg(k, v){
  cfgTouched = Date.now();
  document.getElementById({max_lin:'vMax', turn_gain:'vTurn', pivot:'vPiv', normal:'vNorm'}[k])
          .textContent = Number(v).toFixed(2);
  fetch('/set?'+k+'='+v);
}
function preset(a,b,c){
  document.getElementById('sMax').value=a; setCfg('max_lin',a);
  document.getElementById('sTurn').value=b; setCfg('turn_gain',b);
  document.getElementById('sPiv').value=c; setCfg('pivot',c);
}
function setMode(m){ fetch('/mode?v='+m); }

function showCfg(cfg, mode){
  if (Date.now() - cfgTouched < 2000) return;      // don't fight the user
  var map = {max_lin:['sMax','vMax'], turn_gain:['sTurn','vTurn'],
             pivot:['sPiv','vPiv'], normal:['sNorm','vNorm']};
  for (var k in map){
    if (cfg[k] == null) continue;
    document.getElementById(map[k][0]).value = cfg[k];
    document.getElementById(map[k][1]).textContent = Number(cfg[k]).toFixed(2);
  }
  document.getElementById('mOpen').className   = 'pill ' + (mode==1 ? 'on':'off');
  document.getElementById('mClosed').className = 'pill ' + (mode==3 ? 'on':'off');
  document.getElementById('modeHint').textContent = mode==1
    ? 'open loop: values are throttle, 0.50 \u2248 full power'
    : (mode==3 ? 'closed loop: values are m/s targets' : 'mode unknown');
}

function poll(){
  fetch('/data').then(r=>r.json()).then(d=>{
    D = d;
    var t = d.telem || {}, dr = d.drive || {};

    document.getElementById('pv').textContent   = (t.pv!=null? t.pv.toFixed(2):'--');
    document.getElementById('pct').textContent  = (t.pct!=null? t.pct+'%':'--');
    document.getElementById('amp').textContent  = (t.pa!=null? (Math.abs(t.pa)/1000).toFixed(2):'--');
    document.getElementById('rail').textContent = (t.v!=null? t.v.toFixed(1):'--');
    document.getElementById('yaw').textContent  = (t.y!=null? t.y.toFixed(0)+'\\u00b0':'--');
    document.getElementById('temp').textContent = (t.espTemp!=null? t.espTemp.toFixed(0):'--');
    if (t.pct!=null) document.getElementById('batbar').style.width = t.pct+'%';

    var arm = document.getElementById('arm');
    arm.textContent = dr.armed ? 'ARMED' : 'DISARMED';
    arm.className = 'pill ' + (dr.armed ? 'on' : 'off');
    if (t.lowBat){ arm.className='pill warn'; arm.textContent='LOW BATTERY'; }

    var dir = document.getElementById('dir');
    var th = dr.throttle || 0;
    dir.textContent = th > 0.05 ? 'FORWARD' : (th < -0.05 ? 'REVERSE' : 'IDLE');
    dir.className = 'pill ' + (Math.abs(th) > 0.05 ? 'on' : 'off');

    var l = dr.l||0, r = dr.r||0;
    document.getElementById('wl').textContent = l.toFixed(2);
    document.getElementById('wr').textContent = r.toFixed(2);
    document.getElementById('spd').textContent = ((l+r)/2).toFixed(2);
    document.getElementById('steer').textContent = Math.round((dr.steer||0)*45) + '\\u00b0';
    document.getElementById('pad').textContent = d.pad;
    drawWheels(l, r, 0.6);

    showCfg(d.cfg||{}, d.mode);

    var cs = d.cam_stat || {}, cm = d.cam || {};
    document.getElementById('camStat').textContent =
      (cs.dev||'-') + '  ' + (cs.size||'-') + '  ' + (cs.fps||0).toFixed(1) + ' fps  ' +
      (cs.kbps||0).toFixed(0) + ' kB/s  (' + ((cs.kbps||0)*8/1000).toFixed(1) + ' Mbit/s)';
    document.getElementById('camErr').textContent = cs.err ? '  ' + cs.err : '';
    var cb = document.getElementById('camOn');
    cb.textContent = cm.on ? 'ON' : 'OFF';
    cb.className = 'pill ' + (cm.on ? 'on' : 'off');
    if (Date.now() - camTouched > 2000 && cm.fps != null){
      document.getElementById('sFps').value = cm.fps;
      document.getElementById('vFps').textContent = cm.fps;
      document.getElementById('sQual').value = cm.quality;
      document.getElementById('vQual').textContent = cm.quality;
    }

    var hh = d.hdg||{}, hc = d.hold||{};
    document.getElementById('hdgTop').textContent =
      'heading ' + (hh.deg||0).toFixed(1) + ' deg  -  ' + (hc.mode||'') +
      (hc.err ? '  err ' + hc.err.toFixed(1) : '');

    var ob=g('ovlBtn');
    if (ob){
      ob.textContent = 'LIDAR OVERLAY: ' + (d.cfg.ovl_on?'ON':'OFF');
      ob.style.background = d.cfg.ovl_on ? '#2a7' : '';
      ob.style.color = d.cfg.ovl_on ? '#042' : '';
      g('ovlFlipTxt').textContent = d.cfg.ovl_flip ? 'reversed' : 'normal';
      if (!window._ovlTouch || Date.now()-window._ovlTouch>2500){
        for (var oi=0;oi<OVL_TUNE.length;oi++){
          var ok2=OVL_TUNE[oi][0];
          if (d.cfg[ok2]===undefined || !g('os_'+ok2)) continue;
          g('os_'+ok2).value=d.cfg[ok2];
          g('ov_'+ok2).textContent=d.cfg[ok2];
        }
      }
    }
    drawOverlay();

    document.getElementById('lrpm').textContent = (d.lidar_rpm||0).toFixed(0);
    document.getElementById('lpts').textContent = (d.scan||[]).length;
    drawLidar(d.scan||[]);
  }).catch(()=>{}).finally(()=> setTimeout(poll, 120));
}
document.getElementById('lrange').textContent = (RANGE/1000).toFixed(0);
poll();
</script></body></html>"""


CALIB_PAGE = """<!DOCTYPE html><html><head><meta charset="utf-8">
<title>lidar-bot calibration</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
*{box-sizing:border-box}
body{margin:0;background:#12141c;color:#c9d1e0;font-family:system-ui,sans-serif;font-size:14px}
.wrap{max-width:1040px;margin:0 auto;padding:14px;display:flex;flex-wrap:wrap;gap:14px}
.card{background:#1b1f2b;border-radius:12px;padding:16px;flex:1 1 100%}
.half{flex:1 1 460px}
h2{font-size:13px;margin:0 0 4px;font-weight:700;letter-spacing:.03em;color:#fff}
h3{font-size:11px;margin:14px 0 4px;opacity:.6;letter-spacing:.08em;font-weight:600}
.do{font-size:13px;color:#9fb4d0;margin:2px 0 12px;line-height:1.5}
.lab{font-size:11px;opacity:.6;letter-spacing:.05em;display:block}
.num{font-size:28px;color:#fff;line-height:1.1}
.row{display:flex;gap:14px;flex-wrap:wrap;align-items:flex-end}
.grid{display:flex;gap:22px;flex-wrap:wrap}
input[type=number],select{background:#0c0e14;border:1px solid #2a3040;color:#dfe6f2;
  border-radius:7px;padding:7px 8px;font-size:14px;width:96px;margin-top:3px}
input:focus{outline:2px solid #4a8;border-color:#4a8}
button{background:#2a3040;border:0;color:#dfe6f2;border-radius:7px;padding:9px 14px;
  font-size:13px;font-weight:600;cursor:pointer}
button:hover{background:#39415a}
button.go{background:#2a7;color:#042}
button:disabled{background:#242a36;color:#5a6376;cursor:not-allowed}
button.stop{background:#c33;color:#fff;padding:11px 22px;font-size:15px}
.banner{border-radius:10px;padding:12px 14px;font-size:13px;line-height:1.5}
.amber{background:#3a2f14;color:#f0c869;border-left:4px solid #d4a022}
.red{background:#3a1a1c;color:#ffb0b0;border-left:4px solid #d33}
.green{background:#16301f;color:#9fe0b8;border-left:4px solid #2a7}
canvas{width:100%;background:#0c0e14;border-radius:8px;display:block;margin-top:10px}
.msg{background:#0c0e14;border-radius:8px;padding:11px;margin-top:12px;font-size:13px;
  min-height:19px;line-height:1.5}
.hint{font-size:12px;opacity:.5;line-height:1.5;margin-top:10px}
table{width:100%;border-collapse:collapse;font-size:13px;margin-top:8px}
td{padding:4px 0;border-bottom:1px solid #232936}
td:last-child{text-align:right;color:#fff;font-variant-numeric:tabular-nums}
a{color:#6ad}
.pill{display:inline-block;padding:3px 11px;border-radius:20px;font-size:12px;font-weight:600}
.on{background:#2a7;color:#041}.off{background:#3a4150;color:#9aa4b8}.warn{background:#a33;color:#fff}
.arrow{font-size:34px;line-height:1;color:#4a8;min-width:44px;text-align:center}
details{margin-top:6px}
summary{cursor:pointer;font-size:12px;opacity:.7;letter-spacing:.06em;font-weight:600}
.sw{display:flex;align-items:center;gap:10px;cursor:pointer;font-size:14px;font-weight:600}
.sw input{width:20px;height:20px;margin:0}
</style></head><body>
<div class="wrap">

  <div class="card" style="display:flex;align-items:center;gap:16px;flex-wrap:wrap">
    <div style="flex:1 1 auto"><b>DRIVETRAIN CALIBRATION</b>
      &nbsp;<a href="/">&larr; back to dashboard</a></div>
    <button class="stop" onclick="api('/cal/stop')">STOP</button>
  </div>

  <div class="card" id="armCard" style="padding:12px 16px">
    <label class="sw"><input type="checkbox" id="safe" onchange="gate()">
      <span id="armTxt">Tick this when the robot is on the floor with clear space
      &mdash; the RUN buttons stay greyed out until you do</span></label>
  </div>

  <div id="gyroBox" class="card banner red" style="display:none"></div>
  <div id="warnBox" class="card banner red" style="display:none"></div>
  <div id="signBox" class="card banner amber" style="display:none"></div>
  <div id="statusBox" class="card banner green">Ready. Work down the steps in order
    &mdash; step 1 first.</div>

  <div class="card half">
    <h2>LIVE</h2>
    <div class="grid" style="align-items:center">
      <div class="arrow" id="arrow">&mdash;</div>
      <div><div class="num" id="hcomp">--</div><div class="lab">COMPASS 0-360</div></div>
      <div><div class="num" id="hrate">--</div><div class="lab">TURN RATE deg/s</div></div>
      <div><div class="num" id="herr">--</div><div class="lab">OFF LINE deg</div></div>
    </div>
    <table>
      <tr><td>controller</td><td id="hmode">--</td></tr>
      <tr><td>heading, counted continuously</td><td id="hdeg">--</td></tr>
      <tr><td>correction being applied</td><td id="hcorr">--</td></tr>
      <tr><td>turn rate asked for</td><td id="hwant">--</td></tr>
      <tr><td>gyro updates</td><td id="ghz">--</td></tr>
      <tr><td>telemetry dropouts</td><td id="hgaps">--</td></tr>
      <tr><td>impossible compass jumps ignored</td><td id="hmrej">--</td></tr>
      <tr><td>board compass (raw)</td><td id="hyaw">--</td></tr>
      <tr><td>gyro vs compass disagreement</td><td id="hdiff">--</td></tr>
      <tr><td>drift the filter has learnt</td><td id="hbhat">--</td></tr>
      <tr><td>compass being believed?</td><td id="hmtrust">--</td></tr>
      <tr><td>bias / scale in use</td><td id="hbs">--</td></tr>
    </table>
    <div class="row" style="margin-top:12px">
      <label class="lab">HEADING SOURCE<br>
        <select id="hsrc" style="width:auto" onchange="S('hdg_src', this.value)">
          <option value="gyro">gyro only &mdash; smooth, drifts slowly</option>
          <option value="fused">fused &mdash; gyro, nudged onto the compass</option>
          <option value="yaw">board compass only &mdash; absolute, but noisy</option>
        </select></label>
      <button onclick="S('hdg_src','gyro')">USE GYRO ONLY</button>
      <button onclick="api('/hdg?zero=1')">ZERO HEADING</button>
      <button onclick="S('hdg_inv', D.cfg.hdg_inv?0:1)">FLIP SIGN</button>
      <button onclick="api('/cal/apply?what=drift')">USE LEARNT DRIFT AS BIAS</button>
    </div>
    <div class="hint"><b>Which one?</b> The gyro is smooth and reacts instantly but
      has no absolute reference, so it creeps over minutes. The board compass is
      absolute but its magnetometer is bolted next to your motor wiring, so it
      swings around when current flows. <b>Fused</b> integrates the gyro and leans
      it gently back onto the compass only while the robot is not turning &mdash;
      absolute in the long run, steady in corners. Try fused; watch the
      disagreement row. If the two never settle within about 10&deg; of each other
      when the robot is parked, the compass is being pulled by the motors and you
      should stay on gyro only.
      <br><br>Fused also learns how much drift is left in the gyro and shows it
      above. Let it run on <b>fused</b> for a minute or two while the robot sits
      still, then press <b>USE LEARNT DRIFT AS BIAS</b> &mdash; that improves the
      gyro-only source too, and is a shortcut for step 2.</div>
    <canvas id="view3d" width="620" height="300"></canvas>
    <canvas id="plot" width="620" height="120"></canvas>
    <div class="hint">The continuous heading deliberately keeps counting past 360
      &mdash; 1080 means three full left turns. Hold needs a number that does not
      jump from 359 back to 0 mid-corner, so it never wraps. The compass reading
      above is the same angle folded back into 0-360 if you prefer to read that.</div>
  </div>

  <div class="card half">
    <h2>WHAT THE ROBOT CURRENTLY BELIEVES</h2>
    <div class="do">These three numbers live on the ESP32. Steps 2 and 3 below
      measure them properly; you only need to type here if you want to set one by hand.</div>
    <table>
      <tr><td>wheel diameter</td><td id="v_wd">--</td></tr>
      <tr><td>encoder pulses per wheel turn</td><td id="v_ppr">--</td></tr>
      <tr><td>effective track width</td><td id="v_tw">--</td></tr>
      <tr><td>axle spacing you measured</td><td id="v_twg">--</td></tr>
      <tr><td>distance travelled, left / right</td><td id="v_od">--</td></tr>
    </table>
    <div class="row" style="margin-top:4px">
      <div style="flex:1 1 auto"><b>Left / right motors</b>
        <span id="swapPill" class="pill off">--</span>
        <div class="hint" style="margin-top:4px">If the robot physically turns the
          opposite way to what this page commands &mdash; but your gamepad feels
          right &mdash; the wheels are wired the other way round and the stick
          inversion has been hiding it. This fixes it at the source, on the board,
          so the gamepad, the calibration routines and the odometry all agree.
          It flips the stick inversion at the same time so your controller keeps
          working; check both afterwards.</div></div>
      <button onclick="api('/board?fix=1')">SWAP MOTORS ON BOARD</button>
    </div>
    <details><summary>SET BY HAND</summary>
      <div class="row" style="margin-top:10px">
        <label class="lab">WHEEL DIA mm<br><input type="number" id="gwd" step="0.1" onfocus="tag(this)" oninput="tag(this)"></label>
        <label class="lab">PULSES / REV<br><input type="number" id="gppr" step="1" onfocus="tag(this)" oninput="tag(this)"></label>
        <label class="lab">TRACK WIDTH mm<br><input type="number" id="gtw" step="0.1" onfocus="tag(this)" oninput="tag(this)"></label>
        <label class="lab">AXLE SPACING mm<br><input type="number" id="twg" step="0.5"
          onfocus="tag(this)" oninput="tag(this)"
          onchange="S('tw_geom', this.value/1000)"></label>
        <button onclick="writeGeo()">WRITE TO BOARD</button>
      </div>
    </details>
    <div class="hint">Put a ruler across the two drive wheels, centre to centre,
      and type that into AXLE SPACING under SET BY HAND. It is the yardstick:
      scrub can only make the <i>effective</i> track width a little bigger than the
      real one, never smaller, so anything wildly off that is a bad measurement
      rather than a discovery, and the APPLY button will say so instead of storing
      it.</div>
  </div>

  <div class="card">
    <h2>STEP 0 &nbsp;&mdash;&nbsp; FULL DRIVETRAIN CHECK</h2>
    <div class="do"><b>Start here.</b> The robot drives forward a few centimetres,
      then pivots left a little, and from the encoder and gyro readings it works
      out: whether the gyro is arriving fast enough, whether each encoder counts
      the way its wheel turns, whether left and right are the right way round on
      the board, and whether the gyro agrees with the direction it really turned.
      It changes nothing until you press APPLY FIXES.</div>
    <div class="row">
      <label class="lab">PULSE SPEED m/s<br><input type="number" id="kv" value="0.22" step="0.02"></label>
      <label class="lab">PULSE LENGTH s<br><input type="number" id="ksec" value="0.7" step="0.1"></label>
      <button class="go run" onclick="run('check',{v:v('kv'),secs:v('ksec')})">RUN FULL CHECK</button>
      <button id="fixBtn" onclick="api('/cal/fixes')" disabled>APPLY FIXES</button>
    </div>
    <pre id="chk" style="background:#0c0e14;border-radius:8px;padding:11px;
      margin-top:12px;font-size:12px;line-height:1.6;white-space:pre-wrap;
      font-family:ui-monospace,monospace;color:#bcd">run it to see the report</pre>
  </div>

  <div class="card">
    <h2>STEP 0b &nbsp;&mdash;&nbsp; TILT AXES &amp; LEVEL (motors stay off)</h2>
    <div class="do">If rolling the robot moves <i>pitch</i> on the 3D view, the IMU
      is bolted in rotated and the two are swapped. Press RUN, then <b>lift the
      nose and hold it up</b> until the countdown ends &mdash; the Pi watches which
      number moves and sorts out the mapping and the sign itself. Then stand the
      robot normally and press SET LEVEL so your 6 mm nose-up stance reads as
      zero.</div>
    <div class="row">
      <label class="lab">SECONDS<br><input type="number" id="axsec" value="8" step="1"></label>
      <button class="go" onclick="run('axes',{secs:v('axsec')})">RUN AXIS DETECT</button>
      <button onclick="api('/att?level=1')">SET LEVEL</button>
      <button onclick="api('/att?swapmap=1')">SWAP ROLL/PITCH</button>
      <button onclick="api('/att?roll_inv='+(D.cfg.roll_inv?0:1))">INVERT ROLL</button>
      <button onclick="api('/att?pitch_inv='+(D.cfg.pitch_inv?0:1))">INVERT PITCH</button>
    </div>
    <table>
      <tr><td>board reports roll / pitch</td><td id="a_raw">--</td></tr>
      <tr><td>robot roll / pitch after mapping</td><td id="a_map">--</td></tr>
      <tr><td>mapping / inverts / offsets</td><td id="a_cfg">--</td></tr>
    </table>
  </div>

  <div class="card">
    <h2>STEP 1 &nbsp;&mdash;&nbsp; GYRO DIRECTION</h2>
    <div class="do"><b>Do this first, and do it again after any wiring change.</b>
      The robot pivots left for about a second and checks that the gyro agrees.
      If the gyro counts backwards, heading hold steers the wrong way and the robot
      lurches &mdash; which is exactly what you saw. This fixes it automatically.
      <br>Watch the robot: it should rotate <b>anticlockwise</b>. If it physically
      turns the other way, your motor wiring is swapped, not the gyro.</div>
    <div class="row">
      <label class="lab">PIVOT SPEED m/s<br><input type="number" id="zrate" value="0.22" step="0.02"></label>
      <button class="go run" onclick="run('sign',{rate:v('zrate')})">RUN SIGN CHECK</button>
    </div>
    <div class="hint">You can also just push the robot round by hand and watch the
      big arrow in the LIVE panel &mdash; turning the robot left should show a left
      arrow.</div>
  </div>

  <div class="card">
    <h2>STEP 2 &nbsp;&mdash;&nbsp; GYRO BIAS</h2>
    <div class="do">Leave the robot completely still and untouched. Whatever the
      gyro reads while nothing is moving is pure offset; left in, the heading slowly
      creeps and hold gradually steers you off line. Takes 12 seconds.</div>
    <div class="row">
      <label class="lab">SECONDS<br><input type="number" id="bsecs" value="12" step="1"></label>
      <button class="go run" onclick="run('bias',{secs:v('bsecs')})">RUN</button>
      <button onclick="api('/cal/apply?what=bias')">APPLY BIAS</button>
    </div>
  </div>

  <div class="card">
    <h2>STEP 3 &nbsp;&mdash;&nbsp; STRAIGHT LINE &rarr; WHEEL SIZE</h2>
    <div class="do">Mark where the robot starts. It drives forward, then stops.
      Tape-measure how far it actually went, type that in, press APPLY. Every
      distance the robot reports afterwards will be right.</div>
    <div class="row">
      <label class="lab">DISTANCE m<br><input type="number" id="sdist" value="2" step="0.1"></label>
      <label class="lab">SPEED m/s<br><input type="number" id="sspd" value="0.25" step="0.05"></label>
      <label class="lab">HEADING HOLD<br><select id="shold"><option value="1">on</option><option value="0">off</option></select></label>
      <button class="go run" onclick="run('straight',{dist:v('sdist'),speed:v('sspd'),hold:v('shold')})">RUN</button>
    </div>
    <table>
      <tr><td>commanded</td><td id="s_cmd">--</td></tr>
      <tr><td>encoders reported</td><td id="s_enc">--</td></tr>
      <tr><td>heading drift, final / peak</td><td id="s_dev">--</td></tr>
      <tr><td>drift per source, gyro / compass</td><td id="s_src">--</td></tr>
    </table>
    <div class="row" style="margin-top:12px">
      <label class="lab">WHAT YOU MEASURED, m<br><input type="number" id="s_act" step="0.005" placeholder="e.g. 1.93"></label>
      <button onclick="api('/cal/apply?what=wd&value='+v('s_act'))">APPLY WHEEL DIAMETER</button>
    </div>
  </div>

  <div class="card">
    <h2>STEP 4 &nbsp;&mdash;&nbsp; SPIN &rarr; TRACK WIDTH &amp; GYRO SCALE</h2>
    <div class="do"><b>You never have to measure an angle.</b> Put a strip of tape
      on the floor lined up with something on the robot. Press START SPIN &mdash; it
      turns slowly. Press <b>PASSED THE MARK</b> once when it is sitting on the
      mark, then again every time it comes back round. Press FINISH when you have
      done a few. The true rotation is laps &times; 360 exactly, so there is
      nothing to estimate and nothing to type.
      <br><br>One turn is enough; three is better. Your reaction delay does not
      matter, because it applies equally to the first and last press and cancels out.
      <br><br><b>Do step 3 before this one.</b> Track width is worked out from
      how far the encoders say each wheel rolled, so a wheel diameter that is 5%
      out makes the track width 5% out too.
      <br><br>You can also do this with the robot off: press START SPIN if you want
      it to drive itself, or just turn it by hand and use the mark button.</div>
    <div class="row">
      <label class="lab">PIVOT m/s (keep it slow)<br><input type="number" id="prate" value="0.12" step="0.02"></label>
      <label class="lab">DIRECTION<br><select id="pccw"><option value="1">left</option><option value="0">right</option></select></label>
      <label class="lab">GIVE UP AFTER, s<br><input type="number" id="psec" value="45" step="5"></label>
      <button class="go run" onclick="run('spin',{secs:v('psec'),rate:v('prate'),ccw:v('pccw')})">START SPIN</button>
    </div>
    <div class="row" style="margin-top:14px;align-items:center">
      <button onclick="api('/cal/lap')" style="background:#d4a022;color:#241a00;
        font-size:16px;padding:14px 26px">PASSED THE MARK</button>
      <div><div class="num" id="p_laps">--</div><div class="lab">FULL TURNS COUNTED</div></div>
      <button onclick="api('/cal/lap?finish=1')">FINISH &amp; WORK IT OUT</button>
      <button onclick="api('/cal/lap?reset=1')">CLEAR</button>
      <button onclick="api('/cal/stop')">STOP SPINNING</button>
    </div>
    <table>
      <tr><td>true rotation from the laps</td><td id="p_true">--</td></tr>
      <tr><td>gyro measured</td><td id="p_gyro">--</td></tr>
      <tr><td>board compass measured</td><td id="p_mag">--</td></tr>
      <tr><td>wheel travel, left / right</td><td id="p_lr">--</td></tr>
      <tr><td>difference between the wheels</td><td id="p_diff">--</td></tr>
      <tr><td>wheels travelled evenly?</td><td id="p_asym">--</td></tr>
    </table>
    <div class="row" style="margin-top:12px">
      <label class="lab">OR TYPE IT YOURSELF, deg<br><input type="number" id="p_act" step="5"
        placeholder="left blank = use the laps" oninput="preview()"></label>
      <button onclick="api('/cal/apply?what=scale&value='+v('p_act'))">APPLY GYRO SCALE</button>
      <button onclick="api('/cal/apply?what=tw&value='+v('p_act'))">APPLY TRACK WIDTH</button>
    </div>
    <div class="msg" id="p_prev">Type what it really turned and this will show you
      what the two buttons would set, before you press them.</div>
  </div>

  <div class="card">
    <h2>STEP 4b &nbsp;&mdash;&nbsp; MEASURE A MOVE YOU DRIVE YOURSELF</h2>
    <div class="do">The easiest way to calibrate, and it uses exactly what you have
      been doing by hand. Press <b>MARK START</b>, then drive the robot yourself
      with the controller &mdash; spin it five full turns, or drive it a measured
      distance down a hallway. Come back, type in what it <i>actually</i> did, and
      press MEASURE. Then use whichever APPLY buttons are relevant.</div>
    <div class="row">
      <button onclick="api('/cal/mark')">MARK START</button>
      <label class="lab">IT TURNED, deg (5 spins = 1800)<br><input type="number" id="m_deg" step="5" placeholder="1800"></label>
      <label class="lab">IT TRAVELLED, m<br><input type="number" id="m_m" step="0.01" placeholder="optional"></label>
      <button onclick="api('/cal/measure?deg='+(v('m_deg')||0)+'&m='+(v('m_m')||0))">MEASURE</button>
    </div>
    <table>
      <tr><td>encoder travel, mean (L / R)</td><td id="m_enc">--</td></tr>
      <tr><td>wheel difference</td><td id="m_diff">--</td></tr>
      <tr><td>gyro turned / compass turned</td><td id="m_rot">--</td></tr>
    </table>
    <div class="row" style="margin-top:12px">
      <button onclick="api('/cal/apply?what=tw&value='+v('m_deg'))">APPLY TRACK WIDTH</button>
      <button onclick="api('/cal/apply?what=scale&value='+v('m_deg'))">APPLY GYRO SCALE</button>
      <button onclick="api('/cal/apply?what=wd&value='+v('m_m'))">APPLY WHEEL DIAMETER</button>
    </div>
    <div class="hint">Which number fixes what: <b>track width</b> only changes
      rotation worked out from the <i>encoders</i>, and the turn rate the robot aims
      for. <b>Wheel diameter</b> only changes distance. <b>Gyro scale</b> only
      changes what the gyro reports &mdash; and the gyro does not use your wheel
      sizes at all, so if five real turns read as one on the gyro, that is a gyro
      or telemetry problem, not a track width one.</div>
  </div>

  <div class="card">
    <h2>STEP 5 &nbsp;&mdash;&nbsp; DRIFT TEST (your caster problem, as a number)</h2>
    <div class="do">Pivots, then drives straight, and reports how many degrees it
      wandered on the straight bit. Run it with hold OFF first to get the baseline,
      then with hold ON. Hold on should be a couple of degrees.</div>
    <div class="row">
      <label class="lab">TURN deg, + is right<br><input type="number" id="dturn" value="90" step="15"></label>
      <label class="lab">THEN STRAIGHT m<br><input type="number" id="ddist" value="1.5" step="0.1"></label>
      <label class="lab">SPEED m/s<br><input type="number" id="dspd" value="0.25" step="0.05"></label>
      <button class="go run" onclick="run('drift',{turn:v('dturn'),dist:v('ddist'),speed:v('dspd'),hold:0})">RUN, HOLD OFF</button>
      <button class="go run" onclick="run('drift',{turn:v('dturn'),dist:v('ddist'),speed:v('dspd'),hold:1})">RUN, HOLD ON</button>
    </div>
    <table>
      <tr><td>hold OFF &mdash; pull over the straight leg</td><td id="d_off">--</td></tr>
      <tr><td>hold ON &mdash; pull over the straight leg</td><td id="d_on">--</td></tr>
    </table>
  </div>

  <div class="card">
    <h2>STEP 6 &nbsp;&mdash;&nbsp; SWITCH IT ON FOR DRIVING</h2>
    <div class="do">Only once step 1 has passed. If anything feels wrong, the
      STOP button at the top switches both off instantly.
      <br><br><b>KEEP HEADING</b> is the answer to your drift. Any time the robot is
      moving forward or back with the steering centred, it locks the heading it had
      at that moment and trims the wheels to stay on it &mdash; so the caster
      swinging back to straight gets cancelled instead of curving you off. It picks
      up a new heading the instant you let go of the stick after a turn, so
      turn-then-go-straight holds the new direction, not the old one. It does
      nothing below about 0.05 m/s, and nothing while you are actively steering.
      <br><br><b>ACCURATE TURNS</b> makes the robot turn at the rate the stick is
      asking for rather than whatever the floor allows &mdash; on the bench the open
      loop came out 20% short, closed loop 1%.</div>
    <div class="row">
      <button id="tHold" onclick="S('hold_on', D.cfg.hold_on?0:1)">KEEP HEADING</button>
      <button id="tRate" onclick="S('rate_on', D.cfg.rate_on?0:1)">ACCURATE TURNS</button>
    </div>
    <details><summary>TUNING &mdash; only if it weaves or feels lazy</summary>
      <div id="sliders" style="margin-top:10px"></div>
      <div class="hint">Raise <b>hold kp</b> until the robot snaps back onto line,
        then back off just before it starts weaving. <b>hold ki</b> removes a steady
        lean from one motor being stronger. <b>hold kd</b> damps it. Everything is
        in degrees per second of correction, so the numbers mean something:
        hold kp 2 means a 10&deg; error asks for a 20&deg;/s correction.</div>
    </details>
  </div>

</div>
<script>
var D = {cfg:{}, hdg:{}, geo:{}, auto:{}, hold:{}};
var EH = [], touched = {};

function g(id){return document.getElementById(id)}
function v(id){var e=g(id);return e?parseFloat(e.value):0}
function api(u){return fetch(u).then(function(r){return r.json()}).catch(function(){})}
function S(k,val){touched['_tune']=Date.now();return api('/set?'+k+'='+val)}
function tag(el){touched[el.id]=Date.now()}
function fresh(id){
  if (document.activeElement && document.activeElement.id===id) return false;
  return !touched[id] || Date.now()-touched[id] > 4000;
}
function run(name,args){
  var q=[]; for (var k in args) q.push(k+'='+args[k]);
  api('/cal/run?name='+name+'&'+q.join('&'));
}
function gate(){
  var ok=g('safe').checked, b=document.getElementsByClassName('run');
  for (var i=0;i<b.length;i++) b[i].disabled=!ok;
  g('armCard').className = 'card banner ' + (ok?'green':'amber');
  g('armTxt').textContent = ok
    ? 'Motors enabled - the RUN buttons below will move the robot.'
    : 'Tick this when the robot is on the floor with clear space - the RUN buttons stay greyed out until you do';
}
function writeGeo(){
  api('/geo?wd='+(v('gwd')/1000)+'&ppr='+v('gppr')+'&tw='+(v('gtw')/1000));
}

var TUNE = [
  ['hold_kp',   0,   8,    0.1,  'hold P - deg/s of correction per degree off line'],
  ['hold_ki',   0,   5,    0.1,  'hold I - removes a steady lean'],
  ['hold_kd',   0,   1.5,  0.05, 'hold D - damping, stops weaving'],
  ['hold_max',  10,  200,  5,    'biggest correction hold may ask for, deg/s'],
  ['turn_boost',0.2, 2.5,  0.05, 'turn authority multiplier'],
  ['turn_rate', 20,  400,  10,   'turn rate ceiling, deg/s'],
  ['rate_kp',   0,   3,    0.05, 'turn P - deg/s of trim per deg/s of error'],
  ['rate_ki',   0,   6,    0.1,  'turn I'],
  ['rate_max',  10,  250,  5,    'biggest trim turns may ask for, deg/s'],
  ['max_lin',   0.05,1.5,  0.05, 'top speed, m/s'],
  ['turn_gain', 0.1, 1.5,  0.05, 'open-loop steering strength'],
  ['pivot',     0.05,1.0,  0.05, 'pivot wheel speed, m/s']
];
(function(){
  var h='';
  for (var i=0;i<TUNE.length;i++){
    var t=TUNE[i];
    h += '<label class="lab" style="margin-top:10px">'+t[0].replace("_"," ").toUpperCase()
      +  ' <b id="tv_'+t[0]+'">--</b> <span style="opacity:.7">'+t[4]+'</span></label>'
      +  '<input type="range" id="ts_'+t[0]+'" min="'+t[1]+'" max="'+t[2]+'" step="'+t[3]
      +  '" style="width:100%" oninput="S(\\''+t[0]+'\\', this.value);'
      +  'g(\\'tv_'+t[0]+'\\').textContent=this.value">';
  }
  g('sliders').innerHTML=h;
})();

// ---------------------------------------------------------------- 3D view
// Small painter's-algorithm renderer - no libraries, works offline on the Pi.
// Robot frame: x forward, y left, z up. The robot turns, the world stays put.
var EL = 0.60;                       // camera elevation, radians
function proj(p, sc, cx, cy){
  var se=Math.sin(EL), ce=Math.cos(EL);
  return [cx - p[1]*sc, cy - p[0]*se*sc - p[2]*ce*sc, p[0]*ce - p[2]*se];
}
function rotate(p, yaw, pitch, roll){
  var x=p[0], y=p[1], z=p[2], c, s, t;
  c=Math.cos(roll);  s=Math.sin(roll);  t=y*c-z*s; z=y*s+z*c; y=t;
  c=Math.cos(pitch); s=Math.sin(pitch); t=x*c+z*s; z=-x*s+z*c; x=t;
  c=Math.cos(yaw);   s=Math.sin(yaw);   t=x*c-y*s; y=x*s+y*c; x=t;
  return [x,y,z];
}
function boxFaces(cx,cy,cz,sx,sy,sz,col){
  var v=[], i;
  for (i=0;i<8;i++)
    v.push([cx+(i&1?sx:-sx)/2, cy+(i&2?sy:-sy)/2, cz+(i&4?sz:-sz)/2]);
  var q=[[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]];
  var out=[];
  for (i=0;i<6;i++) out.push({p:[v[q[i][0]],v[q[i][1]],v[q[i][2]],v[q[i][3]]],c:col});
  return out;
}
function robotFaces(){
  var f=[];
  f=f.concat(boxFaces(0,0,0.10, 0.42,0.30,0.10, [58,70,96]));    // chassis
  f=f.concat(boxFaces(-0.04,0.17,0.07, 0.16,0.05,0.16, [26,32,44])); // left wheel
  f=f.concat(boxFaces(-0.04,-0.17,0.07, 0.16,0.05,0.16, [26,32,44])); // right wheel
  f=f.concat(boxFaces(0.18,0,0.03, 0.07,0.07,0.06, [40,48,64]));  // caster
  f=f.concat(boxFaces(0.02,0,0.20, 0.16,0.16,0.09, [70,160,130])); // lidar
  f=f.concat(boxFaces(0.24,0,0.13, 0.10,0.05,0.03, [230,170,60])); // nose marker
  return f;
}
var RFACES = robotFaces();
function shade(col, n){
  var L=[0.4,0.5,0.76], d=n[0]*L[0]+n[1]*L[1]+n[2]*L[2];
  var k=0.45+0.55*Math.max(0,d);
  return 'rgb('+Math.round(col[0]*k)+','+Math.round(col[1]*k)+','+Math.round(col[2]*k)+')';
}
function normal(p){
  var a=[p[1][0]-p[0][0],p[1][1]-p[0][1],p[1][2]-p[0][2]];
  var b=[p[2][0]-p[0][0],p[2][1]-p[0][1],p[2][2]-p[0][2]];
  var n=[a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
  var m=Math.hypot(n[0],n[1],n[2])||1;
  return [n[0]/m,n[1]/m,n[2]/m];
}
function draw3d(hdgDeg, pitchDeg, rollDeg){
  var c=g('view3d'); if(!c) return;
  var x=c.getContext('2d'), W=c.width, H=c.height;
  var sc=Math.min(W,H)*0.85, cx=W/2, cy=H*0.60;
  x.clearRect(0,0,W,H);

  // world-fixed compass ring and cardinal letters
  var i, a, p, q;
  x.strokeStyle='#232b3a'; x.lineWidth=1;
  for (i=0;i<72;i++){
    a=i/72*2*Math.PI;
    p=proj([Math.cos(a)*0.62, Math.sin(a)*0.62, 0], sc, cx, cy);
    if(i===0) { x.beginPath(); x.moveTo(p[0],p[1]); } else x.lineTo(p[0],p[1]);
  }
  x.closePath(); x.stroke();
  x.fillStyle='#4a5570'; x.font='bold 11px system-ui'; x.textAlign='center';
  var card=[['N',0],['E',-90],['S',180],['W',90]];
  for (i=0;i<4;i++){
    a=card[i][1]*Math.PI/180;
    p=proj([Math.cos(a)*0.78, Math.sin(a)*0.78, 0], sc, cx, cy);
    x.fillText(card[i][0], p[0], p[1]+4);
  }
  // heading needle on the ring
  var hr=hdgDeg*Math.PI/180;
  x.strokeStyle='#d4a022'; x.lineWidth=2; x.beginPath();
  p=proj([0,0,0],sc,cx,cy); x.moveTo(p[0],p[1]);
  p=proj([Math.cos(hr)*0.62, Math.sin(hr)*0.62, 0],sc,cx,cy); x.lineTo(p[0],p[1]);
  x.stroke();

  // the robot itself, sorted back to front
  var yaw=hr, pit=pitchDeg*Math.PI/180, rol=rollDeg*Math.PI/180;
  var list=[];
  for (i=0;i<RFACES.length;i++){
    var f=RFACES[i], wp=[], dsum=0, sp=[];
    for (var k=0;k<4;k++){
      var r=rotate(f.p[k], yaw, pit, rol);
      r[2]+=0.02;
      wp.push(r);
      var pr=proj(r, sc, cx, cy);
      sp.push(pr); dsum+=pr[2];
    }
    list.push({sp:sp, d:dsum/4, col:f.c, n:normal(wp)});
  }
  list.sort(function(a,b){return b.d-a.d});
  for (i=0;i<list.length;i++){
    var L=list[i];
    x.beginPath();
    x.moveTo(L.sp[0][0],L.sp[0][1]);
    for (var k=1;k<4;k++) x.lineTo(L.sp[k][0],L.sp[k][1]);
    x.closePath();
    x.fillStyle=shade(L.col, L.n); x.fill();
    x.strokeStyle='rgba(0,0,0,.35)'; x.lineWidth=1; x.stroke();
  }
  x.fillStyle='#66718c'; x.font='10px system-ui'; x.textAlign='left';
  x.fillText('heading '+hdgDeg.toFixed(0)+'  pitch '+pitchDeg.toFixed(1)
             +'  roll '+rollDeg.toFixed(1), 6, 13);
  x.textAlign='center';
}

function preview(){
  var r=(D.auto&&D.auto.res)||{}, act=v('p_act')||r.auto_deg, box=g('p_prev');
  if (!box) return;
  if (!act || !r.gyro_deg){
    box.textContent='Count some laps and press FINISH, and this will show you what '
      + 'the APPLY buttons would set.'; return }
  var scale=(D.cfg.gyro_scale||1)*Math.abs(act)/Math.abs(r.gyro_deg);
  var tw=Math.abs(r.diff_m)/(Math.abs(act)*Math.PI/180)*1000;
  var t='APPLY GYRO SCALE would set '+scale.toFixed(3)+' (now '
      + (D.cfg.gyro_scale||1).toFixed(3) + ') - the gyro is reading '
      + (Math.abs(r.gyro_deg)/Math.abs(act)).toFixed(2) + 'x the truth. '
      + 'APPLY TRACK WIDTH would set ' + tw.toFixed(0) + ' mm (now '
      + ((D.geo.tw||0)*1000).toFixed(0) + ').';
  var geom=(D.cfg.tw_geom||0.175)*1000, ratio=tw/geom;
  t += '  That is ' + ratio.toFixed(2) + 'x your measured axle spacing of '
     + geom.toFixed(0) + ' mm.';
  if (ratio<0.90 || ratio>1.50)
    t += '  REJECTED - scrub cannot do that. The rotation that would land on '
       + geom.toFixed(0) + ' mm is about '
       + (Math.abs(r.diff_m)/(geom/1000)*180/Math.PI).toFixed(0) + ' deg, so '
       + 'either it turned much further than you think or the wheels slipped.';
  box.textContent=t;
}

function plot(){
  var c=g('plot'), x=c.getContext('2d'), W=c.width, H=c.height;
  x.clearRect(0,0,W,H);
  x.strokeStyle='#2a3040'; x.beginPath(); x.moveTo(0,H/2); x.lineTo(W,H/2); x.stroke();
  var span=10;
  for (var i=0;i<EH.length;i++) span=Math.max(span,Math.abs(EH[i])*1.2);
  x.strokeStyle='#4a8'; x.lineWidth=1.5; x.beginPath();
  for (var i=0;i<EH.length;i++){
    var px=W-(EH.length-i)*(W/300), py=H/2-EH[i]/span*(H/2-4);
    if(i===0) x.moveTo(px,py); else x.lineTo(px,py);
  }
  x.stroke();
  x.fillStyle='#556'; x.font='10px system-ui';
  x.fillText('degrees off line, +/-'+span.toFixed(0), 5, 12);
}

function fmt(n,d){return (n===undefined||n===null)?'--':Number(n).toFixed(d===undefined?1:d)}

function poll(){
  fetch('/data').then(function(r){return r.json()}).then(function(d){
    D=d;
    var h=d.hdg||{}, hl=d.hold||{}, a=d.auto||{}, c=d.cfg||{}, geo=d.geo||{};

    var comp=((h.deg||0)%360+360)%360;
    g('hcomp').textContent=fmt(comp,0);
    g('hrate').textContent=fmt(h.srate);
    g('herr').textContent=fmt(hl.err);
    g('hdeg').textContent=fmt(h.deg)+' deg';
    g('hcorr').textContent=fmt(hl.wcmd)+' deg/s  ('+fmt(hl.corr,3)+' m/s)';
    g('hwant').textContent=fmt(hl.want)+' deg/s';
    g('ghz').textContent=fmt(h.hz)+' Hz'+(h.ok?'':'  NO DATA');
    g('hmrej').textContent=(h.mrej||0)+(h.mrej>20?'  (compass is being swamped by the motors)':'');
    g('hmrej').style.color=(h.mrej>20)?'#fc6':'#fff';
    g('hgaps').textContent=(h.gaps||0)+(h.gaps>3?'  (heading accuracy suffers)':'');
    g('hgaps').style.color=(h.gaps>3)?'#fc6':'#fff';
    g('hyaw').textContent=fmt(h.yaw)+' deg  (continuous '+fmt(h.ymag)+')';
    var md=h.magdiff||0;
    g('hdiff').textContent=fmt(md)+' deg';
    g('hdiff').style.color = Math.abs(md)>25 ? '#f77' : (Math.abs(md)>10?'#fc6':'#7d9');
    if (document.activeElement!==g('hsrc')) g('hsrc').value=c.hdg_src||'gyro';
    g('hbhat').textContent=fmt(h.bhat,3)+' deg/s';
    g('hmtrust').textContent = (c.hdg_src==='fused')
      ? (h.mtrust ? 'yes - trimming the gyro' : 'no - ignoring it, gyro only')
      : 'not in use with this source';
    g('hbs').textContent=fmt(c.gyro_bias,3)+' deg/s  /  x'+fmt(c.gyro_scale,4);
    var m=hl.mode||'--';
    g('hmode').textContent=m;
    g('hmode').style.color=(m.indexOf('fault')>=0)?'#f77':'#fff';
    var rr=h.srate||0;
    g('arrow').textContent = rr>8 ? 'LEFT' : (rr<-8 ? 'RIGHT' : 'still');
    g('arrow').style.color = Math.abs(rr)>8 ? '#4a8' : '#3a4150';
    g('arrow').style.fontSize = '20px';

    EH.push(hl.err||0); if(EH.length>300) EH.shift();
    plot();
    var at=d.att||{};
    draw3d(h.deg||0, Number(at.pitch)||0, Number(at.roll)||0);

    if (d.save_err){
      var eb=g('warnBox');
      eb.style.display='block';
      eb.textContent='SETTINGS ARE NOT BEING SAVED - ' + d.save_err
        + '.  Check: ls -l ~/.lidarbot_drive.json  (if it is owned by root, '
        + 'sudo chown $USER ~/.lidarbot_drive.json)';
      eb.className='card banner red';
    }
    var gb=g('gyroBox'), gm='';
    if (h.ok===false){
      gm = 'NO GYRO DATA - the board has not sent gz for ' + fmt(h.age) + ' s. '
         + 'Gyro and fused cannot work; only "board compass only" will. The '
         + 'firmware needs to send gz in its telemetry.';
    } else if (h.slow){
      gm = 'GYRO TOO SLOW - gz is arriving at only ' + fmt(h.hz) + ' Hz, which '
         + 'means your board is sending it in the once-a-second status reply '
         + 'rather than the fast stream. That is why gyro and fused looked dead. '
         + 'The Pi is now polling the board at ' + (h.polled?'6 Hz as a stopgap':'a low rate')
         + ' - usable, but reflash the firmware for the real fix.';
    } else if (!h.yok && (c.hdg_src==='fused'||c.hdg_src==='yaw')){
      gm = 'NO COMPASS DATA - the board is not sending y. Switch the source to '
         + 'gyro only.';
    }
    if (gm){ gb.style.display='block'; gb.textContent=gm } else gb.style.display='none';
    var wb=g('warnBox');
    if (h.warn){ wb.style.display='block'; wb.textContent=h.warn }
    else wb.style.display='none';
    var sb=g('signBox');
    if (c.hdg_src!=='gyro'){
      sb.style.display='block';
      sb.textContent='Heading source is "'+c.hdg_src+'". If the heading creeps '
        + 'back towards where it started a few seconds after every turn, that is '
        + 'the magnetometer pulling it - press USE GYRO ONLY. Mapping does not '
        + 'need a compass; the lidar provides absolute heading through SLAM.';
    }
    else if (!c.sign_ok){ sb.style.display='block';
      sb.textContent='Gyro direction has not been checked yet. Run STEP 1 before '
        +'turning heading hold on - a backwards gyro is what makes the robot lurch '
        +'the wrong way.' }
    else sb.style.display='none';

    g('v_wd').textContent=fmt(geo.wd*1000)+' mm';
    g('v_ppr').textContent=fmt(geo.ppr,0);
    g('v_tw').textContent=fmt(geo.tw*1000)+' mm'
      + (c.tw_geom>0.02 ? '  ('+(geo.tw/c.tw_geom).toFixed(2)+'x measured)' : '');
    g('v_twg').textContent=fmt(c.tw_geom*1000)+' mm';
    if (fresh('twg')) g('twg').value=(c.tw_geom*1000).toFixed(1);
    var sp=g('swapPill');
    sp.textContent = geo.swap ? 'SWAPPED' : 'NORMAL';
    sp.className = 'pill ' + (geo.swap ? 'on' : 'off');
    var lp=(d.laps===undefined?-1:d.laps);
    g('p_laps').textContent = lp<0 ? '--' : lp;
    var od=d.odo||[0,0];
    g('v_od').textContent=fmt(od[0],3)+' / '+fmt(od[1],3)+' m';
    if (fresh('gwd')) g('gwd').value=(geo.wd*1000).toFixed(1);
    if (fresh('gppr')) g('gppr').value=Math.round(geo.ppr);
    if (fresh('gtw')) g('gtw').value=(geo.tw*1000).toFixed(1);

    var busy=!!a.busy, sbx=g('statusBox');
    g('armCard').style.opacity = busy ? .6 : 1;
    sbx.className='card banner '+(a.ok===false?'red':(busy?'amber':'green'));
    sbx.textContent = busy
      ? ('RUNNING ' + a.name.toUpperCase() + ' - ' + (a.step||'') +
         '     (press STOP at any time)')
      : ((a.msg||'') || 'Ready. Work down the steps in order - step 1 first.');

    var r=a.res||{};
    if (r.kind==='straight'){
      g('s_cmd').textContent=fmt(r.cmd_m,3)+' m';
      g('s_enc').textContent=fmt(r.enc_m,3)+' m';
      g('s_dev').textContent=fmt(r.dev)+' / '+fmt(r.dev_max)+' deg';
      g('s_src').textContent=fmt(r.gyro_dev)+' / '+fmt(r.mag_dev)+' deg';
    }
    if (r.kind==='spin'){
      g('p_true').textContent = r.auto_deg
        ? fmt(r.auto_deg,0)+' deg  ('+r.laps+' turns, measured exactly)'
        : 'not lap-counted - type it in below';
      g('p_gyro').textContent=fmt(r.gyro_deg)+' deg'
        + (r.auto_deg ? '   -> reading '+(Math.abs(r.gyro_deg)/r.auto_deg).toFixed(2)+'x the truth' : '');
      g('p_mag').textContent=fmt(r.mag_deg)+' deg';
      g('p_lr').textContent=fmt(r.encl_m,3)+' / '+fmt(r.encr_m,3)+' m';
      g('p_diff').textContent=fmt(r.diff_m,4)+' m';
      g('p_asym').textContent=(r.asym>15?'no - ':'yes - ')+fmt(r.asym,0)+'% apart';
      g('p_asym').style.color=(r.asym>15)?'#fc6':'#7d9';
      preview();
    }
    if (r.kind==='check'){
      g('chk').textContent=r.text||'';
      g('fixBtn').disabled = !(r.fix && r.fix.length);
      g('fixBtn').className = (r.fix && r.fix.length) ? 'go' : '';
    }
    if (r.kind==='manual'){
      g('m_enc').textContent=fmt(r.enc_m,3)+' m  ('+fmt(r.encl_m,3)+' / '+fmt(r.encr_m,3)+')';
      g('m_diff').textContent=fmt(r.diff_m,4)+' m';
      g('m_rot').textContent=fmt(r.gyro_deg)+' / '+fmt(r.mag_deg)+' deg';
    }
    g('a_raw').textContent=fmt(at.rawr)+' / '+fmt(at.rawp)+' deg';
    g('a_map').textContent=fmt(at.roll)+' / '+fmt(at.pitch)+' deg';
    g('a_cfg').textContent=(c.imu_map||'rp')
      +(c.roll_inv?'  roll inv':'')+(c.pitch_inv?'  pitch inv':'')
      +'   off '+fmt(c.roll_off)+' / '+fmt(c.pitch_off);

    var L=a.last||{};
    if (L.drift_off) g('d_off').textContent=fmt(L.drift_off.dev)+' deg (peak '+fmt(L.drift_off.dev_max)+')';
    if (L.drift_on) g('d_on').textContent=fmt(L.drift_on.dev)+' deg (peak '+fmt(L.drift_on.dev_max)+')';

    g('tHold').className=c.hold_on?'go':'';
    g('tRate').className=c.rate_on?'go':'';
    g('tHold').textContent='KEEP HEADING: '+(c.hold_on?'ON':'OFF');
    g('tRate').textContent='ACCURATE TURNS: '+(c.rate_on?'ON':'OFF');
    if (!touched['_tune'] || Date.now()-touched['_tune']>2500){
      for (var i=0;i<TUNE.length;i++){
        var k=TUNE[i][0];
        if (c[k]===undefined) continue;
        g('ts_'+k).value=c[k];
        g('tv_'+k).textContent=c[k];
      }
    }
  }).catch(function(){}).finally(function(){ setTimeout(poll,150) });
}
gate(); poll();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _q(self):
        from urllib.parse import urlparse, parse_qs
        return parse_qs(urlparse(self.path).query)

    def do_GET(self):
        if self.path.startswith("/stream.mjpg"):
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            try:
                last = None
                while True:
                    f = cam_frame["jpeg"]
                    if f is None or f is last:
                        time.sleep(0.02)
                        continue
                    last = f
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(f)))
                    self.end_headers()
                    self.wfile.write(f)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        if self.path.startswith("/cam"):
            from urllib.parse import urlparse, parse_qs
            q = parse_qs(urlparse(self.path).query)
            for k, v in q.items():
                if k in CAM:
                    try:
                        CAM[k] = (v[0] == "1") if k in ("on", "flip") else int(float(v[0]))
                    except ValueError:
                        pass
            self._json({"ok": True, "cam": CAM})
            return

        if self.path.startswith("/set"):
            q = self._q()
            for k, v in q.items():
                if k not in CFG:
                    continue
                cur = CFG[k]
                try:
                    if isinstance(cur, bool):
                        CFG[k] = v[0] in ("1", "true", "on", "True")
                    elif isinstance(cur, str):
                        CFG[k] = v[0]
                    else:
                        CFG[k] = float(v[0])
                except ValueError:
                    pass
            if q.get("hold_on") or q.get("rate_on") or "hdg_inv" in q:
                HDG["warn"] = ""
            save_cfg()
            self._json({"ok": True, "cfg": CFG})
            return

        # ------------------------------------------------ calibration API
        if self.path.startswith("/hdg"):
            q = self._q()
            if q.get("zero"):
                hdg_zero()
            self._json({"ok": True, "hdg": {k: v for k, v in HDG.items()
                                            if not k.startswith("_")}})
            return

        if self.path.startswith("/geo"):
            q = self._q()
            msg = {}
            for k in ("wd", "ppr", "tw"):
                if k in q:
                    try:
                        GEO[k] = float(q[k][0])
                        msg[k] = GEO[k]
                    except ValueError:
                        pass
            if msg and ESP is not None:
                if "ppr" in msg:
                    msg["ppr"] = int(msg["ppr"])
                msg["save"] = 1
                msg["T"] = 700
                ESP.send(msg)
                ESP.send({"T": 701})
            self._json({"ok": True, "geo": GEO})
            return

        if self.path.startswith("/board"):
            q = self._q()
            msg = ""
            if "swap" in q or "fix" in q:
                want = (0 if GEO["swap"] else 1) if "fix" in q \
                    else (1 if q["swap"][0] in ("1", "true", "on") else 0)
                if ESP is not None:
                    ESP.send({"T": 700, "swap": want, "save": 1})
                    ESP.send({"T": 701})
                GEO["swap"] = want
                # the gamepad was compensating for the old mapping, so undo
                # that at the same time or the stick flips instead
                if "fix" in q:
                    CFG["steer_inv"] = not CFG["steer_inv"]
                    save_cfg()
                    msg = ("left/right motors swapped on the board and the stick "
                           "inversion undone to match - re-check both")
                else:
                    msg = "board motor swap set to %d" % want
                hdg_zero()
            self._json({"ok": True, "msg": msg, "geo": GEO, "cfg": CFG})
            return

        if self.path.startswith("/cal/run"):
            q = self._q()
            name = q.get("name", [""])[0]
            self._json(run_auto(name, q))
            return

        if self.path.startswith("/cal/stop"):
            auto_stop()
            _halt()
            self._json({"ok": True})
            return

        if self.path.startswith("/cal/apply"):
            q = self._q()
            what = q.get("what", [""])[0]
            self._json(cal_apply(what, _f(q, "value", 0.0)))
            return

        if self.path.startswith("/cal/fixes"):
            self._json(apply_fixes())
            return

        if self.path.startswith("/cal/lap"):
            q = self._q()
            if q.get("reset"):
                self._json(lap_reset())
            elif q.get("finish"):
                self._json(lap_finish())
            else:
                self._json(lap_mark())
            return

        if self.path.startswith("/cal/mark"):
            self._json(mark_now())
            return

        if self.path.startswith("/cal/measure"):
            q = self._q()
            self._json(measure_now(_f(q, "deg", 0.0), _f(q, "m", 0.0)))
            return

        if self.path.startswith("/att"):
            q = self._q()
            msg = ""
            if q.get("level"):
                r = att_level()
                msg = ("level set: roll offset %+.2f, pitch offset %+.2f"
                       % (r["roll_off"], r["pitch_off"]))
            if q.get("swapmap"):
                CFG["imu_map"] = "pr" if CFG["imu_map"] == "rp" else "rp"
                msg = "tilt axes swapped, mapping is now '%s'" % CFG["imu_map"]
                save_cfg()
            for k in ("roll_inv", "pitch_inv"):
                if k in q:
                    CFG[k] = q[k][0] in ("1", "true", "on")
                    msg = "%s = %s" % (k, CFG[k])
                    save_cfg()
            att_update({})
            self._json({"ok": True, "msg": msg, "att": ATT, "cfg": CFG})
            return

        if self.path.startswith("/calib"):
            body = CALIB_PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path.startswith("/mode"):
            from urllib.parse import urlparse, parse_qs
            q = parse_qs(urlparse(self.path).query)
            mt = int(q.get("v", ["1"])[0])
            ESP.send({"T": 700, "mt": mt, "save": 1})
            with lock:
                state["mode"] = mt
            self._json({"ok": True, "mode": mt})
            return

        if self.path.startswith("/data"):
            with lock:
                snap = dict(state)
            age = hdg_health()
            snap["hdg"] = {k: (round(v, 4) if isinstance(v, float) else v)
                           for k, v in HDG.items() if not k.startswith("_")}
            snap["hdg"]["age"] = round(age, 2)
            snap["save_err"] = SAVE_ERR["msg"]
            snap["att"] = {k: round(v, 2) for k, v in ATT.items()}
            snap["geo"] = dict(GEO)
            snap["auto"] = dict(AUTO)
            snap["hold"] = {"mode": HOLD.mode, "err": round(HOLD.err, 2),
                            "corr": round(HOLD.corr, 4),
                            "wcmd": round(HOLD.wcmd, 1),
                            "want": round(HOLD.want, 1),
                            "target": (None if HOLD.target is None
                                       else round(HOLD.target, 2))}
            lm, rm = odo_m()
            snap["odo"] = [round(lm, 4), round(rm, 4)]
            snap["laps"] = LAP["n"] if LAP["first"] is not None else -1
            body = json.dumps(snap).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)


def main():
    lport = None
    if "--lidar" in sys.argv:
        lport = sys.argv[sys.argv.index("--lidar") + 1]
    else:
        found = sorted(glob.glob("/dev/ttyUSB*"))
        lport = found[0] if found else None

    global ESP
    ESP = Esp()
    esp = ESP
    print(f"ESP32 on {ESP_PORT}")

    if lport:
        threading.Thread(target=lidar_thread, args=(lport,), daemon=True).start()
        print(f"LiDAR on {lport}")
    else:
        print("No /dev/ttyUSB* found - lidar view will be empty")

    threading.Thread(target=camera_thread, daemon=True).start()

    if save_cfg():
        print(f"settings file OK: {SETTINGS}")

    esp.send({"T": 701})          # pull wheel diameter / ppr / track width

    if "--no-pad" not in sys.argv:
        threading.Thread(target=pad_thread, args=(esp,), daemon=True).start()

    # keep telemetry fresh even when the board's stream is quiet
    def poke():
        """Keep telemetry fresh. If the board's fast stream is not carrying gz,
        fall back to polling the full-status reply (which always has it) often
        enough to be usable - a stopgap until the firmware is reflashed."""
        n = 0
        while True:
            esp.send({"T": 710})
            n += 1
            if not GEO["ok"] or n % 30 == 0:
                esp.send({"T": 701})
            if HDG["slow"] or not HDG["ok"]:
                HDG["polled"] = True
                time.sleep(1.0 / 6.0)     # ~6 Hz, the link cannot take much more
            else:
                HDG["polled"] = False
                time.sleep(1.0)
    threading.Thread(target=poke, daemon=True).start()

    print(f"\nDashboard: http://<pi-ip>:{HTTP_PORT}\n")
    ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler).serve_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nbye")
