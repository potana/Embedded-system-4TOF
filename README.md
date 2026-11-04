# 4-ToF scanner in C (Raspberry Pi Pico)

C firmware for a four-sensor Time-of-Flight scanner with an ST7735 display, ported from the MicroPython script `4ToF.py`. Part of a 360° environment scanner for INF2004.

## What it does
Every 5 seconds (`SCAN_PERIOD_MS`), or straight away when GP20 is pressed, it reads all four sensors, moves the scan angle on by 15° and updates the screen:

- **Point cloud**: one solid 4x4 dot per sensor, coloured by distance: red = near, then yellow, green, cyan, blue = far (0 to 2000 mm).
- **Scale from the maximum range**: `MAX_VALID_MM` (2000) reaches the edge of the plot in every direction. Change it and the scale and colours follow.
- **Nothing detected**: a blue dot at the edge of the plot, and `2000mm` in the text. Serial keeps the raw value (such as 8190). An I2C error shows ` err` and plots nothing.
- **Text**: S1 to S4 distances at the bottom of the screen, and an `angle=... S1=...` line on USB serial (115200).

Wait for the `S1`-`S4` labels after power-up before pressing GP20.

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
- **S1 and S3 readings are one scan old**: after 5 s idle, the first reading is a leftover from the previous scan, so it is about 5 s old.
- **The angle is not real yet**: it goes up 15° per scan whether or not the board has turned, and old dots stay on screen.
- **No zone picture yet**: S2 and S4 give one point each.

Third-party notices are in [LICENSES.md](LICENSES.md).
