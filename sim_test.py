"""Off-robot dry run for robot_server.py: stub the serial port, then drive a
two-wheel + caster plant (motor lag, traction loss, caster pull) through the
real mixer and heading controller so signs and tuning are checked on the bench."""
import sys, types, math, importlib.util, os, json

serm = types.ModuleType("serial")
class _S:
    def __init__(self, *a, **k): pass
    def readline(self): return b""
    def write(self, b): pass
    def flush(self): pass
    @property
    def in_waiting(self): return 0
    def read(self, n): return b""
serm.Serial = _S
sys.modules["serial"] = serm

spec = importlib.util.spec_from_file_location(
    "rs", os.path.expanduser("~/mnt/Robot/robot_server.py"))
rs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rs)

class Clock:
    def __init__(self): self.t = 1000.0
    def time(self): return self.t
    def sleep(self, s): self.t += s
CK = Clock(); rs.time = CK

class FakeEsp:
    def send(self, o): pass
rs.ESP = FakeEsp()

TW, DT = 0.172, 0.04
rs.GEO.update({"wd": 0.080, "ppr": 1650, "tw": TW, "ok": True})
K = math.pi * rs.GEO["wd"] / rs.GEO["ppr"]


class Bot:
    """lag on the motors, 80% of the commanded differential reaches the floor
    (scrub + caster drag), plus a decaying caster pull after a turn."""
    TAU, EFF = 0.12, 0.80

    def __init__(self):
        self.h = 0.0          # true heading, deg
        self.l = self.r = 0.0  # actual wheel speeds
        self.cl = self.cr = 0.0
        self.pull = 0.0
        self.decay = 0.4
        self.bias = 0.0       # permanent yaw bias, deg/s (unequal motors)
        rs.hdg_zero(); rs.HOLD.reset()

    def step(self, cl_, cr_):
        a = DT / (self.TAU + DT)
        self.l += (cl_ - self.l) * a
        self.r += (cr_ - self.r) * a
        w = (self.r - self.l) / TW * 57.29578 * self.EFF + self.pull + self.bias
        self.h += w * DT
        self.cl += self.l * DT / K
        self.cr += self.r * DT / K
        self.pull *= math.exp(-DT / self.decay)
        CK.t += DT
        rs.state["telem"].update({"odl": round(self.cl), "odr": round(self.cr)})
        rs.hdg_update({"gz": w, "y": rs.wrap180(self.h),
                       "odl": round(self.cl), "odr": round(self.cr)})
        return w

    def go(self, v, steer, secs):
        tr = []
        for _ in range(int(secs / DT)):
            w = self.step(*rs.mix_v(v, steer, True))
            tr.append((self.h, w, rs.HOLD.mode, rs.HOLD.wcmd, self.l, self.r))
        return tr


def line(name, tr, tail=1.0):
    k = int(tail / DT)
    ws = [x[1] for x in tr[-k:]]
    print("  %-30s end %+8.2f deg   rate %+7.1f deg/s   %-9s cmd %+6.1f"
          % (name, tr[-1][0], sum(ws) / len(ws), tr[-1][2], tr[-1][3]))
    return tr[-1][0]


def cmp(label, off, on, tol=1.0):
    if abs(on) <= max(abs(off) * 0.35, tol):
        print("  => PASS  %s: %+.1f -> %+.1f deg\n" % (label, off, on))
    else:
        print("  => FAIL  %s: %+.1f -> %+.1f deg\n" % (label, off, on))


print("counts->m %.6f, telemetry %.0f Hz, motor tau %.2fs, traction %.0f%%\n"
      % (K, 1 / DT, Bot.TAU, Bot.EFF * 100))

print("1. straight after a right turn - caster pulls 25 deg/s, decays over 0.4s")
for flag, nm in ((False, "hold OFF"), (True, "hold ON")):
    rs.CFG["hold_on"] = flag
    b = Bot(); b.pull = 25.0
    r = line(nm, b.go(0.25, 0.0, 4.0))
    off = r if not flag else off
cmp("caster pull rejected", off, r)

print("2. one motor permanently 5 deg/s stronger")
for flag, nm in ((False, "hold OFF"), (True, "hold ON")):
    rs.CFG["hold_on"] = flag
    b = Bot(); b.bias = 5.0
    r = line(nm, b.go(0.25, 0.0, 6.0))
    off = r if not flag else off
cmp("steady lean removed", off, r)

print("3. reversing with the same pull (differential->yaw must not flip)")
for flag, nm in ((False, "hold OFF"), (True, "hold ON")):
    rs.CFG["hold_on"] = flag
    b = Bot(); b.pull = 25.0
    r = line(nm, b.go(-0.25, 0.0, 4.0))
    off = r if not flag else off
cmp("works in reverse", off, r)

print("4. turn-rate tracking, stick right 0.6 while driving 0.25 m/s")
rs.CFG["hold_on"] = True
res = {}
for flag, nm in ((False, "rate OFF"), (True, "rate ON")):
    rs.CFG["rate_on"] = flag
    b = Bot(); tr = b.go(0.25, 0.6, 3.0)
    k = int(1.0 / DT)
    res[nm] = sum(x[1] for x in tr[-k:]) / k
    line(nm, tr)
want = rs.HOLD.want
print("  wanted %.1f deg/s | open loop %.1f (%.0f%% short) | closed loop %.1f (%.0f%% short)"
      % (want, res["rate OFF"], abs(1 - res["rate OFF"] / want) * 100,
         res["rate ON"], abs(1 - res["rate ON"] / want) * 100))
print("  => %s\n" % ("PASS" if abs(want - res["rate ON"]) < abs(want - res["rate OFF"]) * 0.4
                     else "FAIL"))

print("5. pivot on the spot, stick hard left")
b = Bot(); tr = b.go(0.0, -1.0, 3.0)
k = int(1.0 / DT)
print("  wanted %.0f, got %.0f deg/s (ceiling %.0f)\n"
      % (rs.HOLD.want, sum(x[1] for x in tr[-k:]) / k, rs.CFG["turn_rate"]))

print("6. stick back to centre mid-drive - does it keep the NEW heading?")
b = Bot()
b.go(0.25, 0.6, 1.5)
mark = b.h
b.pull = 20.0
b.go(0.25, 0.0, 3.0)
print("  heading when the stick centred %+.1f, 3 s later %+.1f  (drift %+.1f)"
      % (mark, b.h, b.h - mark))
print("  => %s\n" % ("PASS" if abs(b.h - mark) < 6 else "FAIL"))

print("7. what a wrong sign looks like (hdg_inv set backwards)")
rs.CFG["hdg_inv"] = True
b = Bot(); b.pull = 25.0
line("hold ON, inverted", b.go(0.25, 0.0, 3.0))
rs.CFG["hdg_inv"] = False
print()

print("8. wheel command stays inside the clamp under a 200 deg/s disturbance")
b = Bot(); b.bias = 200.0; b.decay = 1e9
tr = b.go(0.45, 0.0, 2.0)
ls = [x[4] for x in tr]; rr = [x[5] for x in tr]
print("  L %.2f..%.2f  R %.2f..%.2f  cap %.2f\n"
      % (min(ls), max(ls), min(rr), max(rr), rs.CFG["max_lin"] * 1.4))

print("9. weaving check - no disturbance at all, hold on for 6 s")
b = Bot(); tr = b.go(0.25, 0.0, 6.0)
hs = [x[0] for x in tr[int(2 / DT):]]
print("  heading stayed within %+.2f / %+.2f deg, peak-peak %.2f"
      % (min(hs), max(hs), max(hs) - min(hs)))
print("  => %s\n" % ("PASS" if max(hs) - min(hs) < 2.0 else "FAIL - weaving"))

print("10. calibration maths")
rs.AUTO["res"] = {"kind": "straight", "enc_m": 2.0}
print("   ", rs.cal_apply("wd", 1.90)["msg"])
rs.AUTO["res"] = {"kind": "spin", "gyro_deg": 720.0, "diff_m": 2.17}
print("   ", rs.cal_apply("scale", 745.0)["msg"])
print("   ", rs.cal_apply("tw", 745.0)["msg"])
rs.AUTO["res"] = {"bias": -0.42}
print("   ", rs.cal_apply("bias", 0)["msg"])

snap = dict(rs.state)
snap["hdg"] = {k: v for k, v in rs.HDG.items() if not k.startswith("_")}
snap["geo"] = dict(rs.GEO); snap["auto"] = dict(rs.AUTO)
snap["hold"] = {"mode": rs.HOLD.mode, "err": rs.HOLD.err, "wcmd": rs.HOLD.wcmd}
snap["odo"] = list(rs.odo_m())
print("\n11. /data serialises: %d bytes ok" % len(json.dumps(snap)))
