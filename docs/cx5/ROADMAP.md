# Roadmap

Rewriting the system: the firmware from scratch, and the control stack and model
inference in C++ instead of Python.

## Why the scope is smaller than it looks

```
board/ total                136,054 lines   ← almost all vendor CMSIS
board/stm32f407/ (ours)         927 lines   ← write from scratch
board/drivers/  (shared)      3,880 lines   ← write from scratch
opendbc safety/modes/mazda.h    636 lines   ← already C, reuse verbatim
host stack (Python)           9,897 lines   ← port to C++
```

Roughly 5k lines of firmware and 10k of host code.

## What not to rewrite

**The safety model.** `mazda.h` is already C, already compiled into the firmware, already
tested through the real compiled code, and it is the audited part. Rewriting the board
layer is reasonable; rewriting this is a much larger risk than it looks.

**The empirical constants.** The code is replaceable; these are not. Each cost real
driving to establish, and a clean-room rewrite discards them silently:

| | |
|---|---|
| `STEER_MAX`, `max_rate_up/down`, `max_torque` | the measured steering envelope |
| 23 IDs on bus 0 | the ID census for this car |
| `0x21c` MSG_1 redundancy quad | upstream templates invert it, so every synthesised frame was discarded |
| accel curve × 0.55, `T_FOLLOW_MIN = 0` | the tune that was settled on the road |
| UDS `0x764` answers only `10 02` | every other diagnostic service is silent |
| sensor 0 / flip 2, GPS 38400 | both differ from every default |

**The regression suites.** Point them at the new code rather than replacing them.

## Stages

Each is a project. Each has a gate you must pass before starting the next.

**0 · Toolchain and recovery.** Blinky, flashed two ways — DFU and SWD. Keep the bootstub.
*Gate:* you can flash a deliberately broken image and recover it. You will need this.

**1 · Clock, flash, linker.** 168 MHz PLL (the F407 ceiling; 180 is the F446). Sector map.
`.isr_vector` at `0x08004000`.
*Gate:* a timer-driven 1 Hz toggle measures 1.000 Hz on a scope.

**2 · bxCAN.** Internal loopback, then a bench partner, then the car read-only.
*Gate:* ~2870 frames/s, 23 IDs, stable for 10 minutes.

**3 · Host link.** Build it against the **existing Python host**, unchanged. If
Python ↔ new firmware works, the firmware is right, and exactly one variable moved.
*Gate:* one hour, engine running, `rx_ovf` flat, no frozen carState.

**4 · Safety and transmit.** Drop `mazda.h` in unchanged.
*Gate:* `override_test2` 16/16 and `button_engage_test` 14/14 against your build.
*At this point the firmware is done and the car works, on the Python stack.*

**5 · C++ transport.** libusb C API, or SPI.
*Gate:* your reader and the Python reader see identical per-ID frame counts.

**6 · C++ carState and control.** `govern_accel`, `SpeedTarget`, `RxLiveness`, the TX
thread. Keep the two-role split.
*Gate:* replay a logged drive through both and diff `aTarget` and steering torque sample
by sample — matching to floating-point noise, not "looks similar".

**7 · C++ inference.** TensorRT's native API, same `.trt` engine. The hard part is the
YUV pack, the warp and the 96-deep feature queue, not `enqueueV3`.
*Gate:* same frames in, diff the output tensors.

**8 · Car.** Parked, engine running, wheels chocked. Then stationary engage. Then a drive.

## Host link: USB now, SPI later

USB is what runs today and there is a guard flashed for its worst failure. SPI removes
that entire failure class instead of guarding it: no IN-token interrupt, no TX FIFO to arm
ahead of filling, every transaction acknowledged or refused.

`board/drivers/spi.h` and `python/spi.py` are already complete. The work is
`board/stm32f407/llspi.h`, currently a 12-line stub — about 110 lines of F4 DMA written
against RM0090. The H7 version cannot be copied; different DMA controller.

Four wires, no DATA_READY line, 3.3 V both sides. SPI2 on PB12–PB15 is free.

## Learning the firmware properly

Standards worth following, in the order to pick them up: **BARR-C:2018** (free, practical),
then **MISRA C**. opendbc already runs the MISRA checker in cppcheck and documents every
deviation in `suppressions.txt` — copy that model. **ISO 26262** is a process, not a
coding standard; learn the concepts, but compliance is an organisational thing, not
something one person produces.

Habits: no `malloc`, no recursion, every loop bounded, every return checked, fixed-width
types, warnings as errors, static analysis clean with documented exceptions, unit tests on
the host.

Reading: **RM0090** (the F405/407 reference manual) open at all times; Joseph Yiu,
*The Definitive Guide to ARM Cortex-M3 and Cortex-M4 Processors*; Elecia White,
*Making Embedded Systems*; James Grenning, *Test-Driven Development for Embedded C*.
