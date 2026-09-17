#!/usr/bin/env python3
"""
Standalone LDROBOT lidar tester — LD06 / LD19 / LD20 / STL-19P. No ROS needed.

Usage:
    python3 lidar_test.py /dev/tty.usbserial-XXXX
    python3 lidar_test.py /dev/tty.usbserial-XXXX --raw    # text mode, no plot
"""
import sys, math, struct, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/tty.usbserial-0001"
BAUD = 230400
RAW = "--raw" in sys.argv

def packets(ser):
    """Yield (angles_deg, distances_mm, intensities, speed) per packet."""
    buf = bytearray()
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
            angs, dists, ints = [], [], []
            for k in range(n):
                d, q = struct.unpack_from('<HB', pkt, 6 + 3 * k)
                angs.append((a0 + step * k) % 360.0)
                dists.append(d)
                ints.append(q)
            yield angs, dists, ints, speed

def main():
    print(f"Opening {PORT} at {BAUD}...")
    ser = serial.Serial(PORT, BAUD, timeout=1)
    gen = packets(ser)

    if RAW:
        for i, (angs, dists, ints, speed) in enumerate(gen):
            print(f"pkt {i:5d} | {len(angs):2d} pts | rpm~{speed/6:5.1f} | "
                  f"{angs[0]:6.1f}deg {dists[0]:5d}mm q{ints[0]:3d} ... "
                  f"{angs[-1]:6.1f}deg {dists[-1]:5d}mm")
            if i > 200:
                break
        return

    import matplotlib.pyplot as plt
    import numpy as np
    scan = {}                                  # angle(int deg) -> mm
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection='polar')
    ax.set_theta_zero_location('N'); ax.set_theta_direction(-1)
    sc = ax.scatter([], [], s=4, c='cyan')
    ax.set_ylim(0, 6000); ax.set_facecolor('#111')
    ax.set_title(f"{PORT} — close window to quit")
    plt.ion(); plt.show()

    count = 0
    for angs, dists, ints, speed in gen:
        for a, d in zip(angs, dists):
            if d > 0:
                scan[int(a)] = d
        count += 1
        if count % 20 == 0:                    # redraw ~every 20 packets
            if not plt.fignum_exists(fig.number):
                break
            pts = [(math.radians(a), d) for a, d in scan.items()]
            if pts:
                sc.set_offsets(np.array(pts))
            ax.set_title(f"{PORT}  |  {len(scan)} pts/rev  |  {speed/6:.0f} rpm")
            plt.pause(0.001)

if __name__ == "__main__":
    try:
        main()
    except serial.SerialException as e:
        print(f"Serial error: {e}\nCheck the port name with: ls /dev/tty.usb*")
    except KeyboardInterrupt:
        pass
