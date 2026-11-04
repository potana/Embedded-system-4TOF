# Third-party code

Parts of this project are ports of existing open-source code. Keep these notices if you redistribute it.

| File | Derived from | Licence |
|---|---|---|
| `src/machine.[ch]` | MicroPython `drivers/bus/softspi.c`, `extmod/machine_i2c.c`, rp2 port | MIT (MicroPython) |
| `src/st7735.[ch]` | `ST7735.py` from st7735-pico-micropython, after Guy Carver | Not confirmed: the copy used has no licence file. Check upstream before sharing. |
| `src/sysfont.[ch]` | `sysfont.py` from st7735-pico-micropython | Not confirmed: as above |
| `src/vl53l0x.[ch]` | MicroPython `vl53l0x.py`; long-range mode after Pololu `vl53l0x-arduino`, based on ST's VL53L0X API | Python driver not confirmed; Pololu MIT and ST BSD-3-Clause |
| `src/vl53l1x.[ch]` | MicroPython `vl53l1x.py`, with ST's default configuration | Not confirmed: check upstream before sharing |
| `pico_sdk_import.cmake` | Raspberry Pi pico-sdk | BSD-3-Clause |

`src/main.c` is based on the team's `4ToF.py`. `src/depth_color.*`, `src/pyexc.*`, `src/bench.c` and `src/lcd_test.c` are original to this project.
Full licence texts are in the upstream repositories.
