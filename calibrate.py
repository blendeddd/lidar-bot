#!/usr/bin/env python3
"""
lidar-bot odometry calibration. Talks to the ESP32 directly - no ROS needed.
Stop robot_server.py / ROS first (one owner per serial port).

  python3 calibrate.py live        watch distance + heading as you push it
  python3 calibrate.py straight    wheel-diameter (distance) calibration
  python3 calibrate.py spin        gyro scale + effective track width
  python3 calibrate.py drift       gyro zero-rate (bias) check - robot still

Why these three:
  straight -> fixes the scale of everything. Wheels roll cleanly in a straight
              line, so encoder distance is trustworthy once scaled right.
  spin     -> the wheels scrub when turning, so heading comes from the gyro.
              This checks the gyro's scale and measures the EFFECTIVE track
              width (always wider than the real one on a skid-steer).
  drift    -> a gyro with a bias slowly rotates your map. Measures it.
"""
import sys, json, time, math, serial

PORT, BAUD = "/dev/ttyAMA0", 115200


class Board:
    def __init__(self):
        self.ser = serial.Serial(PORT, BAUD, timeout=0.5)
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.geom = self.get_geometry()

    def send(self, obj):
        self.ser.write((json.dumps(obj) + "\n").encode())
        self.ser.flush()

    def frame(self, need="odl", timeout=2.0):
        end = time.time() + timeout
        while time.time() < end:
            try:
                d = json.loads(self.ser.readline().decode(errors="ignore").strip())
                if need in d:
                    return d
            except Exception:
                continue
        return None

    def get_geometry(self):
        self.send({"T": 701})
        d = self.frame("ppr", 2.0)
        if not d:
            print("! could not read geometry from the board")
            return {"wd": 0.070, "ppr": 414, "tw": 0.21}
        return {"wd": float(d["wd"]), "ppr": float(d["ppr"]), "tw": float(d["tw"])}

    def m_per_pulse(self):
        return math.pi * self.geom["wd"] / self.geom["ppr"]

    def set_geometry(self, **kw):
        cmd = {"T": 700, "save": 1}
        cmd.update(kw)
        self.send(cmd)
        time.sleep(0.3)
        self.geom = self.get_geometry()


def track(b, duration=None, stop_key=True):
    """integrate distance and gyro heading until Ctrl-C (or duration)"""
    mpp = b.m_per_pulse()
    d0 = b.frame()
    if d0 is None:
        print("no telemetry - is the board powered and the port free?")
        return None
    l0, r0 = d0["odl"], d0["odr"]
    prev = time.time()
    yaw = 0.0
    gz_seen = "gz" in d0
    t_start = time.time()

    try:
        while True:
            d = b.frame(timeout=1.0)
            if d is None:
                continue
            now = time.time()
            dt, prev = now - prev, now

            if "gz" in d:
                yaw += math.radians(float(d["gz"])) * dt

            dl = (d["odl"] - l0) * mpp
            dr = (d["odr"] - r0) * mpp
            dist = (dl + dr) / 2.0
            wheel_yaw = (dr - dl) / b.geom["tw"]

            print(f"\rdist {dist:+7.3f} m   L {dl:+7.3f}  R {dr:+7.3f}   "
                  f"gyro {math.degrees(yaw):+8.1f}deg   wheels {math.degrees(wheel_yaw):+8.1f}deg", end="")

            if duration and (now - t_start) > duration:
                break
    except KeyboardInterrupt:
        pass

    print()
    if not gz_seen:
        print("! no 'gz' in the telemetry - flash the firmware that adds it, "
              "otherwise gyro heading is unavailable")
    return {"dist": dist, "dl": dl, "dr": dr,
            "gyro_yaw": yaw, "wheel_yaw": wheel_yaw}


def ask(prompt, cast=float):
    while True:
        try:
            return cast(input(prompt).strip())
        except ValueError:
            print("  ...number please")


def do_straight(b):
    print(f"\ncurrent: wheel {b.geom['wd']*1000:.1f} mm, {b.geom['ppr']:.0f} pulses/rev "
          f"-> {b.m_per_pulse()*1000:.4f} mm per pulse\n")
    print("Put the robot against a wall or mark its start point.")
    print("Drive (or push) it in a STRAIGHT line - 2 m or more is better.")
    input("Press Enter to start measuring...")
    print("measuring - Ctrl-C when you have stopped\n")

    res = track(b)
    if not res:
        return
    actual = ask("\nMeasured distance with a tape (metres): ")
    if actual <= 0:
        return

    reported = res["dist"]
    err = (reported - actual) / actual * 100
    new_wd = b.geom["wd"] * actual / reported

    print(f"\n  odometry said : {reported:.3f} m")
    print(f"  truth         : {actual:.3f} m   ({err:+.1f}% error)")
    print(f"  wheel dia now : {b.geom['wd']*1000:.1f} mm")
    print(f"  should be     : {new_wd*1000:.1f} mm")

    skew = abs(res["dl"] - res["dr"]) / max(1e-6, abs(reported)) * 100
    if skew > 5:
        print(f"  ! left/right differ by {skew:.0f}% over a straight run - "
              f"one wheel may be slipping, dragging, or a different size")

    if input("\nWrite the corrected wheel diameter to the board? [y/N] ").lower().startswith("y"):
        b.set_geometry(wd=round(new_wd, 5))
        print(f"saved. wheel diameter is now {b.geom['wd']*1000:.1f} mm")


def do_spin(b):
    turns = ask("\nHow many full turns will you do? (4 or more is best): ")
    print("\nMark the robot's heading (tape on the floor helps).")
    print("Spin it on the spot - slowly and steadily - exactly that many turns.")
    input("Press Enter to start measuring...")
    print("measuring - Ctrl-C when you are back on the mark\n")

    res = track(b)
    if not res:
        return

    truth = math.radians(360.0 * turns)
    gyro = abs(res["gyro_yaw"])
    wheel = abs(res["wheel_yaw"])

    print(f"\n  truth        : {math.degrees(truth):.0f} deg")
    print(f"  gyro said    : {math.degrees(gyro):.0f} deg  ({(gyro-truth)/truth*100:+.1f}%)")
    print(f"  wheels said  : {math.degrees(wheel):.0f} deg  ({(wheel-truth)/truth*100:+.1f}%)")

    if gyro > 0:
        print(f"\n  gyro scale factor to apply: {truth/gyro:.4f}")
        if abs(truth / gyro - 1) < 0.03:
            print("  (within 3% - the gyro is fine, no correction needed)")

    # effective track width: what value would make the wheels agree with truth
    if wheel > 0:
        eff = b.geom["tw"] * truth / wheel
        print(f"\n  effective track width: {eff*1000:.0f} mm "
              f"(geometric is {b.geom['tw']*1000:.0f} mm)")
        print("  Bigger than the real track is normal on a skid-steer - that is the scrub.")
        if input("\nWrite the effective track width to the board? [y/N] ").lower().startswith("y"):
            b.set_geometry(tw=round(eff, 4))
            print(f"saved. track width is now {b.geom['tw']*1000:.0f} mm")


def do_drift(b):
    secs = 30
    print(f"\nLeave the robot COMPLETELY STILL for {secs} seconds.")
    input("Press Enter when it is settled...")
    print("measuring...\n")
    res = track(b, duration=secs)
    if not res:
        return
    drift = math.degrees(res["gyro_yaw"]) / secs
    print(f"\n  gyro drift: {drift:+.3f} deg/s  ({drift*60:+.1f} deg per minute)")
    if abs(drift) < 0.05:
        print("  excellent - nothing to worry about")
    elif abs(drift) < 0.3:
        print("  acceptable - SLAM will correct this as it maps")
    else:
        print("  high - headings will wander. Recheck that it was really still;")
        print("  if it is real, subtract this bias in the bridge before integrating.")


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "live"
    b = Board()
    print(f"board geometry: wheel {b.geom['wd']*1000:.1f} mm, "
          f"{b.geom['ppr']:.0f} ppr, track {b.geom['tw']*1000:.0f} mm")

    if what == "straight":
        do_straight(b)
    elif what == "spin":
        do_spin(b)
    elif what == "drift":
        do_drift(b)
    else:
        print("\nlive view - push the robot around, Ctrl-C to stop\n")
        track(b)


if __name__ == "__main__":
    try:
        main()
    except serial.SerialException as e:
        print(f"Serial error: {e}\nIs something else holding {PORT}?")
    except KeyboardInterrupt:
        pass
