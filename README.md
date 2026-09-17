# Stereo vergence scanner

A 2D room scanner built from two cheap time-of-flight rangefinders on two hobby servos.
Instead of sweeping one sensor, **both sensors verge on the same point** at every bearing
— like eyes converging — and the disagreement between them is used as signal rather than
thrown away.

Roughly £15 of hardware doing something a single-sensor sweep can't: telling a flat wall
apart from an edge.

## Why vergence

A single ToF sensor on a servo gives you range at a bearing, and nothing else. If the
beam clips the corner of a doorway you get a reading that looks exactly like a wall a bit
further back.

With two sensors verged on a target, a smooth surface returns two readings whose
relationship is predictable. A depth discontinuity does not — one beam lands on the near
surface, the other slips past it. The residual between observed and expected separation
localises edges to within a degree.

## Architecture

```
NodeMCU (ESP8266)                          Laptop
┌──────────────────────────┐               ┌──────────────────────────────┐
│ 2x VL53L0X on 2x SG90    │  UDP/JSON     │ scanner.py   listener + CSV  │
│ verge at each bearing    │ ────────────► │ geometry.py  fusion + flags  │
│ -89°..+89°, 1° steps     │   port 5005   │ plot_scan.py top-down map    │
│ median-of-3 per point    │               └──────────────────────────────┘
└──────────────────────────┘
      dumb sensor pipe                        all geometry lives here
```

The firmware deliberately does no geometry beyond aiming. It reads, packetises, and
sends. Everything interpretive happens on the laptop, where it can be changed without a
reflash and tested without hardware.

One packet per bearing:

```json
{"seq":0,"t":12345,"theta":-89.00,"sL":93,"sR":87,"dL":812,"dR":809}
```

`dL`/`dR` are millimetres, `-1` for no return. UDP because the device has no business
blocking on a TCP buffer; a dropped packet is a hole in the scan, not a failure, and the
listener reports how many arrived rather than pretending a sweep is complete.

## The two bugs worth knowing about

Both were found by testing, and both are the actual engineering content here.

**1. A constant agreement tolerance is wrong.** With a fixed 5cm threshold, 103 of 179
bearings on a genuinely flat wall came back flagged as edges. Servo commands are whole
degrees, so quantisation alone throws each reading off by an amount proportional to
range — at 3m, one degree is already ~5cm. The tolerance has to scale.

**2. Verged beams don't coincide except at one distance.** This is the real cause, and
it's easy to miss. The sensors cross at `VERGE_RANGE_M`; at any other range they strike
the surface a predictable distance apart:

```
separation = B · |1 − r / R|
```

On a wall at 0.5m with a 10cm baseline verged at 1.5m, that's **6.7cm of pure geometry**
— an order of magnitude above sensor noise, present on a perfectly smooth surface.
Comparing the raw gap against zero therefore condemns most of a close wall. Disagreement
is only meaningful as a *residual against that expectation*.

There's also a `MAX_RANGE_M` gate: the VL53L0X returns a number well past its useful
range, and a handful of 70m readings will otherwise dominate the map.

## Output

Per-sweep CSV with the fused point and a flag per bearing:

| flag | meaning |
|---|---|
| `ok` | both sensors agree; point is their midpoint |
| `edge` | residual exceeds tolerance; keeps the **nearer** surface, since on an edge the far reading is the beam that slipped past |
| `single` | one sensor returned; degraded but recorded |
| `no_return` | neither; recorded as a hole, never as a zero |

`-1` is never treated as `0` — that would plant a phantom obstacle at the origin.

## Running it

```bash
# firmware: set WIFI_SSID / WIFI_PASSWORD / LAPTOP_IP, flash to the NodeMCU
cd laptop
pip install -r requirements.txt
python scanner.py --out scans/
python plot_scan.py scans/sweep_000_*.csv
```

## Tests

```bash
cd laptop && python test_geometry.py     # 10 tests, no hardware needed
```

The firmware computes servo angles from a bearing and the laptop computes a bearing back
from servo angles — two conversions, two languages, two machines, and nothing at runtime
can tell you they've drifted apart. A sign error wouldn't crash; it would produce a
plausible, wrong map. The tests pin the round trip against synthetic readings, including
the flat-wall regression from bug #1.

## Hardware

| | |
|---|---|
| MCU | NodeMCU / ESP8266 |
| Rangefinders | 2× VL53L0X (I²C, remapped to 0x30 / 0x29 via XSHUT at boot) |
| Servos | 2× SG90 |
| Baseline | 100mm, centre to centre |
| Verge range | 1.5m |

`firmware/prior_single_tof_imu.ino` is the earlier approach kept for contrast — one ToF
plus an MPU6050, no vergence, no way to distinguish an edge from a wall.
