"""Boot the real HTTP handler with a stubbed board and hit every route."""
import sys, types, importlib.util, os, threading, time, json, urllib.request
serm = types.ModuleType("serial")
class _S:
    def __init__(s,*a,**k): pass
    def readline(s): time.sleep(0.05); return b""
    def write(s,b): pass
    def flush(s): pass
    @property
    def in_waiting(s): return 0
    def read(s,n): time.sleep(0.05); return b""
serm.Serial=_S; sys.modules["serial"]=serm
f = os.path.expanduser("~/.lidarbot_drive.json")
os.path.exists(f) and os.remove(f)
spec = importlib.util.spec_from_file_location("rs", os.path.expanduser("~/mnt/Robot/robot_server.py"))
rs = importlib.util.module_from_spec(spec); spec.loader.exec_module(rs)

class FakeEsp:
    def __init__(s): s.sent=[]
    def send(s,o): s.sent.append(o)
rs.ESP = FakeEsp()
rs.state["telem"].update({"odl": 1200, "odr": 1180, "gz": 0.4, "y": 12.3,
                          "v": 11.9, "pv": 19.2, "pa": 850, "temp": 41})
rs.hdg_update(rs.state["telem"])
rs.GEO.update({"wd":0.08,"ppr":1650,"tw":0.172,"ok":True})

srv = rs.ThreadingHTTPServer(("127.0.0.1", 8099), rs.Handler)
threading.Thread(target=srv.serve_forever, daemon=True).start()
time.sleep(0.4)

def hit(path, show=0):
    try:
        r = urllib.request.urlopen("http://127.0.0.1:8099"+path, timeout=4)
        b = r.read()
        note = ""
        if show and b[:1] == b"{":
            d = json.loads(b)
            note = "  " + json.dumps(d)[:show]
        print("  %-46s %d  %6d bytes%s" % (path, r.status, len(b), note))
        return b
    except Exception as e:
        print("  %-46s ERROR %s" % (path, e))

print("routes:")
hit("/")
hit("/calib")
hit("/data", show=150)
hit("/hdg?zero=1", show=120)
hit("/set?hold_kp=3.5&hdg_src=yaw&hold_on=0", show=90)
hit("/geo?wd=0.078&ppr=1650&tw=0.19", show=80)
hit("/cal/run?name=bias&secs=1", show=60)
time.sleep(1.6)
hit("/data", show=0)
hit("/cal/stop", show=40)
hit("/cal/apply?what=bias", show=120)
hit("/cal/run?name=nope", show=60)
hit("/mode?v=1", show=40)
hit("/cam?quality=45", show=60)

print("\nCFG after /set:  hold_kp=%s hdg_src=%s hold_on=%s"
      % (rs.CFG["hold_kp"], rs.CFG["hdg_src"], rs.CFG["hold_on"]))
print("GEO after /geo:  %s" % rs.GEO)
print("T:700 writes sent to the board: %s"
      % [o for o in rs.ESP.sent if o.get("T") == 700])
print("saved file keys: %d (incl _ver=%s)"
      % (len(json.load(open(f))), json.load(open(f)).get("_ver")))
os.remove(f)
