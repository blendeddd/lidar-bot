"""Drive the four calibration routines against the simulated plant."""
import sys, types, math, importlib.util, os
serm = types.ModuleType("serial")
class _S:
    def __init__(s,*a,**k): pass
    def readline(s): return b""
    def write(s,b): pass
    def flush(s): pass
    @property
    def in_waiting(s): return 0
    def read(s,n): return b""
serm.Serial = _S; sys.modules["serial"] = serm
os.path.exists(os.path.expanduser("~/.lidarbot_drive.json")) and os.remove(os.path.expanduser("~/.lidarbot_drive.json"))
spec = importlib.util.spec_from_file_location("rs", os.path.expanduser("~/mnt/Robot/robot_server.py"))
rs = importlib.util.module_from_spec(spec); spec.loader.exec_module(rs)

class Clock:
    def __init__(s): s.t = 1000.0
    def time(s): return s.t
    def sleep(s, x): s.t += x
CK = Clock(); rs.time = CK
TW, DT = 0.172, 0.04
rs.GEO.update({"wd": 0.080, "ppr": 1650, "tw": TW, "ok": True})
K = math.pi * 0.080 / 1650

class Plant:
    """Real wheels are 5% smaller than configured and the real track width is
    190 mm not 172 - the routines must discover both."""
    TRUE_WD, TRUE_TW, TAU, EFF = 0.076, 0.190, 0.12, 1.0
    def __init__(s):
        s.h = 0.0; s.l = s.r = 0.0; s.cl = s.cr = 0.0
        s.gbias = 0.35        # deg/s of gyro offset the routine should find
        rs.hdg_zero(); rs.HOLD.reset()
    def send(s, o):
        if o.get("T") != 1: return
        a = DT / (s.TAU + DT)
        s.l += (o["L"] - s.l) * a
        s.r += (o["R"] - s.r) * a
        w = (s.r - s.l) / s.TRUE_TW * 57.29578 * s.EFF
        s.h += w * DT
        # counts come from the TRUE wheel, metres are computed with the configured one
        s.cl += s.l * DT / (math.pi * s.TRUE_WD / 1650)
        s.cr += s.r * DT / (math.pi * s.TRUE_WD / 1650)
        CK.t += DT
        rs.state["telem"].update({"odl": round(s.cl), "odr": round(s.cr), "gz": w + s.gbias})
        rs.hdg_update({"gz": w + s.gbias, "y": rs.wrap180(s.h),
                       "odl": round(s.cl), "odr": round(s.cr)})

P = Plant(); rs.ESP = P
q = lambda **kw: {k: [str(v)] for k, v in kw.items()}

print("plant truth: wheel %.1f mm, track %.1f mm, gyro bias %+.2f deg/s"
      % (P.TRUE_WD*1000, P.TRUE_TW*1000, P.gbias))
print("board thinks: wheel %.1f mm, track %.1f mm\n"
      % (rs.GEO["wd"]*1000, rs.GEO["tw"]*1000))

print("-- bias --")
rs.AUTO.update({"busy": True, "abort": False}); rs.r_bias(q(secs=3))
print("  ", rs.AUTO["msg"]); print("  ", rs.cal_apply("bias", 0)["msg"])

print("\n-- straight 2.0 m --")
P = Plant(); P.gbias = 0.0; rs.ESP = P
rs.AUTO.update({"busy": True, "abort": False}); rs.r_straight(q(dist=2.0, speed=0.25, hold=1))
print("  ", rs.AUTO["msg"])
true_m = (P.cl + P.cr) / 2 * (math.pi * P.TRUE_WD / 1650)
print("   true distance travelled %.3f m (this is what a tape measure would say)" % true_m)
print("  ", rs.cal_apply("wd", true_m)["msg"])
print("   board now thinks the wheel is %.1f mm, truth %.1f mm" % (rs.GEO["wd"]*1000, P.TRUE_WD*1000))

print("\n-- spin 720 deg --")
P = Plant(); P.gbias = 0.0; rs.ESP = P
rs.AUTO.update({"busy": True, "abort": False}); rs.r_spin(q(deg=720, rate=0.28, ccw=1))
print("  ", rs.AUTO["msg"])
print("   true rotation %.1f deg" % P.h)
print("  ", rs.cal_apply("scale", P.h)["msg"])
print("  ", rs.cal_apply("tw", P.h)["msg"])
print("   board now thinks the track is %.1f mm, truth %.1f mm" % (rs.GEO["tw"]*1000, P.TRUE_TW*1000))

print("\n-- drift test, hold off then on --")
for hold in (0, 1):
    P = Plant(); P.gbias = 0.0; rs.ESP = P
    rs.AUTO.update({"busy": True, "abort": False})
    rs.r_drift(q(turn=90, dist=1.5, speed=0.25, hold=hold))
    print("  ", rs.AUTO["msg"])

print("\n-- abort mid-run --")
P = Plant(); rs.ESP = P
rs.AUTO.update({"busy": True, "abort": True}); rs.r_straight(q(dist=5.0))
print("   aborted cleanly, step:", rs.AUTO["step"])
os.path.exists(os.path.expanduser("~/.lidarbot_drive.json")) and os.remove(os.path.expanduser("~/.lidarbot_drive.json"))
