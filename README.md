# 4-ToF scanner in C (Raspberry Pi Pico)

C firmware for a four-sensor Time-of-Flight scanner with an ST7735 display, ported from the MicroPython script `4ToF.py`. Part of a 360° environment scanner for INF2004.

## What it does
Every 5 seconds (`SCAN_PERIOD_MS`), or straight away when GP20 is pressed, it
reads all four sensors, assumes the head has rotated 5 degrees clockwise and
updates a LiDAR-style polar map:

- **LiDAR map**: 72 global bearing bins at 5-degree spacing. The four sensors
  are mounted 90 degrees apart. A newer reading replaces the older reading at
  the same global bearing even when a different sensor supplied it.
- **Point cloud**: one solid 4x4 dot per valid bearing, coloured by distance:
  red = near, then yellow, green, cyan, blue = far (0 to 2000 mm).
- **Scale from the maximum range**: `MAX_VALID_MM` (2000) reaches the edge of
  the plot in every direction. Change it and the scale and colours follow.
- **Nothing detected**: the current bearing is cleared rather than retaining a
  stale obstacle. Text shows `2000mm`; serial keeps the raw value (such as
  8190). An I2C error also clears that bearing and shows ` err`.
- **Text**: S1 to S4 distances appear at the bottom, with
  `clockwise=... points=...` diagnostics on USB serial.

After 18 captures (about 85 seconds from the first immediate capture), the four
sensors have visited all 72 bearings. Continued rotation refreshes every
bearing with the newest measurement.

Wait for the `S1`-`S4` labels after power-up before pressing GP20.

## 3D scanning with the yaw/pitch gimbal (`tools/`)
A PC script drives the gimbal Pico and this board over their two USB serial
ports, converts each (yaw, pitch, sensor, distance) sample to a point relative
to the motor origin, removes isolated points and plots the cloud.

```
python tools/scan3d.py sim --no-show              # demo, no hardware
python tools/scan3d.py scan --gimbal COM5 --tof COM7 --yaw-step 10 --pitch-step 10
python tools/scan3d.py view scan.csv              # re-filter / re-plot a saved scan
python tools/test_pointcloud.py                   # unit tests (stdlib only)
```
`scan` needs `pip install pyserial matplotlib`. It writes `scan.csv` (raw
samples), `scan.ply` (filtered cloud) and `scan.png`.

- **Frame**: origin where yaw and pitch axes meet; +X forward, +Y left, +Z up.
- **Sensor ring**: the four sensors lie on a ring about the pitch axis (pitch 0:
  forward, up, back, down), so pitch 0..90 plus yaw 0..360 covers every direction once.
- **Displacement**: each sensor's `radius_mm` (out along its beam) and
  `axial_mm` (along the pitch axis), plus the pitch-axis offset, are applied before
  adding the range. The defaults (25 mm radius) are placeholders: measure the
  head and pass `--geometry head.json` (a dump of `pointcloud.Geometry`). If
  the motors turn the wrong way, set `yaw_sign`/`pitch_sign` to -1; `*_zero_deg`
  sets the encoder angle that counts as 0.
- **Outliers**: a point is dropped unless at least `--min-neighbours` (2) other
  points lie within `--spacing-factor` (2) x range x angular step of it. The
  radius grows with range because neighbouring samples are further apart there.
- **Host mode**: the first `S` sent to this board switches off the timed and
  GP20 scans; each `S` then returns `S d1 d2 d3 d4` (raw mm, 8190 = nothing
  detected, -1 = I2C error). The gimbal is assumed to be motor 1 = yaw, motor 2 =
  pitch (`--yaw-motor`, `--pitch-motor`).

## Hardware
| Part | Pins |
|---|---|
| ST7735 128x160, software SPI at 500 kHz | SCK GP9, MOSI GP8, DC GP3, RST GP2, CS tied to GND |
| S1 VL53L0X (long-range mode) | SDA GP0, SCL GP1 |
| S2 VL53L1X | SDA GP4, SCL GP5 |
| S3 VL53L0X (long-range mode) | SDA GP6, SCL GP7 |
| S4 VL53L1X | SDA GP26, SCL GP27 |
| Button | GP20, active low |

Each sensor has its own software I2C bus because they all use address 0x29.

## Build and flash
On its own:
```
export PICO_SDK_PATH=/path/to/pico-sdk
mkdir build && cd build
cmake .. -DPICO_BOARD=pico_w
make -j
```
Inside pico-examples (with `add_subdirectory(tof4_c)`): build the `tof4_c` target, for example `ninja tof4_c`.

Hold BOOTSEL, plug in the Pico, and copy `tof4_c.uf2` to the `RPI-RP2` drive.

Two diagnostic programs build alongside it: `tof4_c_lcdtest` (shows which step stops the display) and `tof4_c_bench` (prints latency, update rate and noise on serial).

## Files
| File | What it is |
|---|---|
| `src/main.c` | The scanner, based on `4ToF.py` |
| `src/machine.c/.h` | MicroPython `Pin`, `SoftSPI` and `SoftI2C` in C, with the same timing |
| `src/st7735.c/.h`, `src/sysfont.c/.h` | Display driver and font from `ST7735.py` and `sysfont.py` |
| `src/vl53l0x.c/.h`, `src/vl53l1x.c/.h` | Sensor drivers from `vl53l0x.py` and `vl53l1x.py`, plus long-range mode for the VL53L0X |
| `src/pyexc.c/.h` | Python-style exceptions, so errors behave as in the script |
| `src/depth_color.c/.h` | Distance to colour |

## Measured on the bench
| | S1 / S3 (VL53L0X) | S2 / S4 (VL53L1X) |
|---|---|---|
| Time per measurement | 31 ms | new result every 175-185 ms |
| Scatter at 14-120 mm (standard deviation) | 1.4-2.7 mm | 1.0-1.4 mm |
| Datasheet range, white target, dark | 2 m or more | 3.6 m (capped at 2 m here) |
| Datasheet error | 4% at 1.2 m indoors | ±20 mm dark, ±25 mm bright |

Drawing a full update takes about 115 ms. Real accuracy has not been measured against known distances yet.

## Known issues
- **Display workaround**: the display stops updating after a few readings unless its settings are sent again before each draw (`tft_refresh()`). That works, but the root cause is unknown.
- **S1 and S3 fresh-reading workaround**: the driver discards the first
  VL53L0X result after idle and waits for the next completed measurement. This
  adds one ranging period but prevents assigning an old range to a new pose.
- **The angle is not measured yet**: it advances 5 degrees per scan whether or
  not the board has turned. Replace the assumed angle with encoder-confirmed
  pose for an accurate map.
- **No zone picture yet**: S2 and S4 give one point each.

Third-party notices are in [LICENSES.md](LICENSES.md).
