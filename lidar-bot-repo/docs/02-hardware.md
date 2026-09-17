# Hardware

## Core

| Part | Spec / model | Notes |
|---|---|---|
| Compute | Raspberry Pi 4, 1 GB | Headless. RAM is the tight constraint — no desktop, ever |
| Driver board | Waveshare General Driver for Robots (WS-23730) | ESP32-WROOM-32, TB6612FNG ×2, 9-axis IMU, INA219, OLED + LiDAR headers, powers the Pi over the 40-pin header |
| Motors | 2× JGB37-520, 12 V 333 RPM, hall encoder | ≈1:18.8 gearbox, 11 PPR → ≈827 counts per wheel rev (calibrated on bench), 6 mm D-shaft, 37 mm body, 6× M3 face mount on 31 mm circle |
| LiDAR | LDROBOT **FHL-LD20** (shipped in place of the D300/LD19P) | 360° DToF, ~20 m, UART 230400 8N1 via bundled USB adapter. **Protocol-compatible with LD19** — verified on the bench with `lidar_test.py`; use the driver's `ld19.launch.py` |
| Battery | Ozito PXC 18 V (2.5 Ah preferred over 4.0 Ah for weight) | 5S, 15–21 V. **Treat as unprotected at the terminals** — tool/charger enforce cutoff, not the pack |
| Camera *(optional)* | Pi Camera Module 3 Wide | FPV stream + arrival snapshots + AprilTags. Not used for navigation |

## Power chain

```
Ozito 18 V pack
  └─ 6.3 mm spades → rocker switch (10 A+ DC) → 10 A blade fuse
       └─ INA219 breakout (high-side, addr 0x41) — pack V & A telemetry
            └─ XL4016 buck @ 12.0 V ────► Waveshare board (7–13 V input, XH2.54)
                                              ├─ 5 V via 40-pin header ──► Pi 4 ──► LiDAR (USB), Camera (CSI)
                                              └─ PH2.0 6-pin ×2 ────────► motors + encoders
```

Only **one** buck converter is needed — the driver board's regulator feeds the Pi.
Single power inlet for the whole robot: **never** plug USB-C into the Pi while it is
mounted and fed from the board.

## Bill of materials — everything else

| Item | Purpose |
|---|---|
| XL4016 300 W buck (heatsinked) | 18 V → 12.0 V, set with multimeter before first connection |
| INA219 breakout | Battery voltage + current (address jumper → 0x41; board's own INA219 occupies 0x40) |
| SSD1306 OLED 0.91" 128×32 I²C | IP address + battery readout; plugs into board's OLED header |
| Active buzzer 5 V | Low-battery alarm at ~15.5 V, driven in firmware |
| Inline blade fuse holder + 10 A fuse | Over-current protection (7.5 A / 15 A spares) |
| Rocker switch, 10 A+ DC | Master power |
| XT60 pairs ×2–3 | Battery/bench-supply disconnect |
| 6.3 mm female spade terminals | Grip the Ozito pack's blades inside the printed adapter |
| 18 AWG silicone wire + heatshrink | Power runs |
| XH2.54 2-pin pigtail | Board power input |
| PH2.0 6-pin cables / crimp kit | Motor + encoder harness (and re-pinning if order differs) |
| Dupont F-F jumpers | INA219, buzzer |
| M3 brass standoffs + screws | Board/Pi/buck mounting |
| microSD 32 GB A1 (16 GB workable) | Speed and brand matter more than capacity |
| Wheels 70–80 mm, 6 mm D-hub + TPU tyres | ≈1.2–1.4 m/s top speed; nav cruise 0.2–0.3 m/s |
| Ball caster | Third contact point |

## 3D-printed parts

Chassis deck · Ozito battery adapter · motor brackets · wheels + TPU tyres · caster
housing · LiDAR mast · OLED window · bench wheel-lift stand.

Design notes: slotted mounting holes throughout, 360° clear view for the LiDAR above
wire height, buzzer vent, panel-mount cutouts for XT60 and switch.

---
