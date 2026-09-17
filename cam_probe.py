#!/usr/bin/env python3
"""Find which /dev/videoN is the real capture node on the Pi."""
import glob, os, grp, getpass, subprocess, sys

print("user:", getpass.getuser())
print("groups:", [g.gr_name for g in grp.getgrall() if getpass.getuser() in g.gr_mem])
print()

nodes = sorted(glob.glob("/dev/video*"), key=lambda p: int(p[10:]))
for n in nodes:
    print(f"{n}  readable={os.access(n, os.R_OK)}")
print()

try:
    import cv2
except ImportError:
    sys.exit("python3-opencv not installed")

for n in nodes:
    i = int(n[10:])
    cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"{n}: will not open")
        continue
    ok, f = cap.read()
    if ok:
        print(f"{n}: *** WORKS *** {f.shape[1]}x{f.shape[0]}  <- use index {i}")
    else:
        print(f"{n}: opens but reads nothing (metadata node)")
    cap.release()
