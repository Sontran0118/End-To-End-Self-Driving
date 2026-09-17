# openpilot on Jetson Orin + DIY STM32F446 panda → Mazda CX-5 2023

Port of comma.ai openpilot to run on an **NVIDIA Jetson Orin Nano** with a **DIY panda**
(STM32F446RE Nucleo + 2 CAN transceivers), targeting a **Mazda CX-5 2023** (`MAZDA_CX5_2022`
platform, supported in opendbc).

## Status

| Component | State |
|-----------|-------|
| supercombo driving model on Jetson (TensorRT) | ✅ working, 31 Hz temporal (>20 Hz req) |
| camera → model warp (calibrated) | ✅ ported + verified |
| output decode (path/lanes/lead/pose) | ✅ |
| recurrent feature feedback (temporal) | ✅ |
| panda F446 firmware (serial link) | ✅ **builds, flashes, RUNS on hardware** |
| pandad serial handle | ✅ **verified against the real board** |
| SConscript build / compile | ✅ `board/obj/panda_f446.bin.signed` |
| bench CAN loopback | ✅ **PASS** — both buses loop back |
| on-car | ⬜ TODO (safety-critical) |

## `model/` — the driving model on the Jetson (no openpilot install needed)

- `op_frame.py` — camera → model input. Ports openpilot's `get_warp_matrix` (calibration
  perspective warp) + YUV420 → 12-channel packing. IMX477 intrinsics `fx≈W*0.7` (tune).
- `op_parse.py` — decodes the 2576-float output using the ONNX-embedded `output_slices` metadata.
- `op_stream.py` — **`SupercomboRunner`**: full streaming runner with recurrent feedback
  (feat_q depth 96, subsample /4 → features_buffer 24×512). `.step(bgr)` → driving plan.
- `op_pipeline.py`, `op_run.py` — single-shot test harnesses.

Build the engine on the Orin:
```
trtexec --onnx=driving_supercombo.onnx --saveEngine=supercombo_fp32.trt --skipInference
```
FP32 runs at 42 Hz. (fp16 blocked by a TRT-10.16 gelu-fusion bug; FP32 is fast enough.)

## `panda_f446_firmware/` — DIY panda on the Nucleo-F446RE

Faithful port of comma's last STM32F4 panda firmware (panda commit `3dc21386`, before "bye bye f4")
to the F446, talking to the Jetson over the **ST-Link Virtual COM Port (serial)** instead of USB.

- `drivers/serial_comms.h` — UART transport, a faithful clone of the SPI transport framing.
  Reuses panda's transport-agnostic `comms_control_handler` / `comms_can_read` / `comms_can_write`.
- `drivers/serial_uart_raw.h` — USART2 byte I/O. TX polled; **RX via DMA1 Stream 5 into a 4 KB
  circular buffer**. Polled RX cannot survive the CAN load: at 1.5 Mbaud a byte lands every 6.7 µs
  into a one-byte register with no FIFO, so any ISR longer than that drops a byte.
- `drivers/mazda_filter.h` — `mazda_host_visible()`, the 17-ID gate on the **host feed only**.
  The CAN acceptance filters accept everything so forwarding carries the whole bus both ways.
- `h/peripherals.h` — pin map: **USART2 PA2/PA3** (VCP), **CAN1 PB8/PB9**, **CAN2 PB5/PB6** (all AF9).
- `boards/nucleo.h` — `board_nucleo` definition (2 CAN, minimal).
- `stm32f446/clock.h` — 180 MHz + PLLSAI 48 MHz (F446 dual-PLL).
- `stm32f446/stm32f446_flash.ld` — 512 K flash / 128 K RAM.
- `stm32f446/inc/stm32f446xx.h`, `startup_stm32f446xx.s` — official ST CMSIS.

**Wiring:** transceiver 1 → CAN1 (PB8 RX, PB9 TX) → car main bus; transceiver 2 → CAN2 (PB5 RX,
PB6 TX) → car camera/LKAS bus. Link + power + flash all over the Nucleo's single ST-Link USB.

### CAN topology: what crosses between the buses

Physical CAN1 = logical **bus 0** (car), physical CAN2 = logical **bus 2** (camera); openpilot's
`get_fwd_bus()` only ever pairs 0↔2. Forwarding happens in the RX ISR (`bxcan.h can_rx()` →
`safety_fwd_hook()`), and **only in a car safety mode** — `SAFETY_SILENT`/`SAFETY_NOOUTPUT` set
`disable_forwarding`, so in dashcam mode nothing crosses in either direction.

| frame | direction | what happens |
|---|---|---|
| `0x243` CAM_LKAS | cam → car | **blocked** — openpilot's injected steering frame replaces it |
| `0x440` CAM_LANEINFO | cam → car | **forwarded** (port deviation, see below) |
| every other camera frame | cam → car | **forwarded** — all 11 `CAM_*` IDs cross |
| everything | car → cam | **forwarded** — the whole main bus, radar included |

The acceptance filters accept everything, on both buses. They used to hold the 17-ID Mazda
whitelist, but the filter sits upstream of `can_rx()` — which is where forwarding happens — so it
was deleting 9 of the camera's 11 IDs cam→car and ~726 of the car's IDs car→cam, radar included.
That is what raises the front-camera fault. The whitelist now lives in software
(`mazda_filter.h`), applied at the `can_rx_q` push and the tx echo, so it trims the **host feed**
only. Measured on the car: `fwd` 2359/s car→cam and 226/s cam→car, host feed unchanged at ~800/s.

### Host link: why RX is DMA, and the byte-commit race

Forwarding the whole bus raises bus-0 RX from 776 to ~2325 frames/s, which broke the serial VCP
in two separate ways. Both are fixed; both are easy to reintroduce, so they are written down here.

1. **Polled RX cannot keep up.** USART2 has a one-byte receive register and no FIFO. At 1.5 Mbaud
   a byte lands every 6.67 µs, so any interrupt longer than that loses one. Control-transfer
   reliability fell 100% → 62% purely from the extra CAN ISR load. Fixed by moving RX to
   DMA1 Stream 5 circular (`serial_uart_raw.h`).
2. **NDTR announces a byte before it reaches SRAM.** The DMA cursor decrements when the write is
   *issued*, not when it lands, so a byte the cursor already claims can read back as the ring's
   previous contents. Caught by re-reading the same ring address: `0x00` first, `0x07` a moment
   later, the frame's own checksum confirming `0x07`. It corrupted ~40% of headers — one byte,
   mid-frame, no timeout, no overrun. Fixed by reading each byte twice with a fixed spin between
   and taking the second value; `rx_late` in the 0xd9 counters records every catch.

A third one was self-inflicted: the old error paths called `uart_flush_rx()`, which is right for a
lossy polled register and wrong for a lossless ring — it discarded good headers sitting behind the
bad one and turned one desync into a cascade. Resync is now a one-byte rewind, and a rejected
header is **silent** (a NACK for a header the host never sent gets read as the ack of a later
transaction). Read `0xd9` for `txn_ok / hdr_resync / hdr_timeout / mosi_* / overrun / rx_late`.

Measured on the car, engine running, forwarding active, 15 s in SAFETY_MAZDA:

| | |
|---|---|
| control transfers | **519/519 (100%)** — was 62-75% |
| bus 0 | rx 2325/s, fwd→cam 2325/s, rx_lost 0, err 0, bus_off 0 |
| bus 2 | rx 240/s, fwd→car 224/s, rx_lost 0, err 0, bus_off 0 |
| camera IDs crossing | all 11 `CAM_*` |
| car IDs to host | exactly the 16 gated IDs present on bus 0, no garbage |
| 0x243 inject @ 100 Hz | 1000 sent, **1000 tx-complete echoes**, 0 rejected, 0 lost |

`SerialPanda.can_recv()` now verifies each packet's own XOR checksum. Without it the resync path
invents plausible-looking frames out of packet data — that is where IDs like `0x1fe823f4` in
earlier captures came from, and `dashcam_web.py` uses this same parser.

`0x440` is forwarded by setting `.disable_static_blocking` on the `MAZDA_LKAS_HUD` entry of
`MAZDA_TX_MSGS`, behind `#ifdef PANDA_NUCLEO` in `opendbc/safety/modes/mazda.h`. Stock openpilot
blocks it because its Mazda CarController generates its own `0x440` at 2 Hz
(`mazdacan.create_alert_command`); this port does not, so without the deviation the car's LKAS
system receives **no lane state at all** and ignores the injected steering. `.check_relay` stays
`true`, so a `0x440` or `0x243` arriving on **bus 0** still latches `relay_malfunction` — the
protection that catches a camera which was never electrically cut off the main bus.

Because this is compiled into the firmware, **changing it requires a rebuild and reflash.**
Prove it without hardware first:

```bash
python3 control_stack/lane_fwd_test.py     # 27/27, builds libsafety stock vs -DPANDA_NUCLEO
```

**Harness requirement:** the forward camera must be *inline* — its CAN cut from the main bus and
run only to CAN2. The Nucleo has no intercept relay (`harness_init` forces `HARNESS_STATUS_NC`),
so this must be done in wire. If the camera stays on the main bus, its own `0x243` reaches bus 0,
`relay_malfunction` latches, and **all tx and all forwarding stop permanently** until the safety
mode is re-set.

## `pandad_serial/` — Jetson side

- `serial.cc` — `PandaSerialHandle`: same interface as `PandaSpiHandle`, over `/dev/ttyACM`.
- `panda_comms_serial.h` — class decl to merge into `selfdrive/pandad/panda_comms.h`.

## Building + flashing the F446 firmware (VERIFIED 2026-07-21)

The port files here are *overlays* on comma's panda at commit `3dc21386` (the last F4 commit,
before "bye bye f4"). Reproduce the working tree:

```bash
# toolchain
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi \
                 stlink-tools gdb-multiarch
pip install --break-system-packages scons pycryptodome

# base trees
git clone https://github.com/commaai/panda.git panda_base && \
  (cd panda_base && git checkout 3dc21386)
git clone https://github.com/commaai/opendbc.git opendbc_src && \
  (cd opendbc_src && git checkout 8ddffb37)   # version-matched; pip opendbc is TOO NEW

# assemble
cp -r panda_base panda_f446
cd panda_f446
cp -r  <this>/panda_f446_firmware/stm32f446        board/
cp     <this>/panda_f446_firmware/drivers/serial_*.h     board/drivers/
cp     <this>/panda_f446_firmware/drivers/mazda_filter.h board/drivers/
cp     <this>/panda_f446_firmware/boards/nucleo.h     board/boards/
git apply <this>/panda_base_patches/panda_base_3dc21386.patch

# opendbc-side port patch (compiled INTO the firmware -- see CAN topology below)
(cd ../opendbc_src && git apply <this>/opendbc_patches/opendbc_8ddffb37_mazda_lane_fwd.patch)

# build + flash
scons -j2 board/obj/bootstub.panda_f446.bin board/obj/panda_f446.bin.signed
sudo st-flash --reset write board/obj/bootstub.panda_f446.bin 0x8000000
sudo st-flash --reset write board/obj/panda_f446.bin.signed  0x8004000

# MUST PASS before any car connection
sudo python3 bench_can_loopback.py /dev/ttyACM0
```

### Wiring (Nucleo-F446RE)

The board silkscreens **Arduino** names, not port names:

| Signal | Port | Board label |
|--------|------|-------------|
| CAN1 RX | PB8 | **D15** (SCL) |
| CAN1 TX | PB9 | **D14** (SDA) |
| CAN2 RX | PB5 | **D4** |
| CAN2 TX | PB6 | **D10** |
| 3.3V / GND / VIN | — | **CN6** (left header) |

Transceivers: SN65HVD230 (3.3V native). **Remove the 120Ω termination jumper on the car side** —
the vehicle bus is already terminated at both ends. Link + power + flashing all run over the single
mini-USB (ST-Link VCP, `/dev/ttyACM0`), panda protocol at 1.5 Mbaud on USART2 (PA2/PA3).

### Port gotchas worth knowing

- `MCU_IDCODE` must be **`0x421`** (F446), not the F413's `0x463` — otherwise `early_initialization()`
  takes the "wrong chip" path and HardFaults in `led_init()` before GPIO clocks are on.
- The F446 has **no CAN3**: CAN3 IRQ branches removed, `CAN3` aliased to `CAN2` in `bxcan.h`.
- **128 K RAM** (F413 had 256 K): `CAN_RX_BUFFER_SIZE 4096→1024`, `CAN_TX_BUFFER_SIZE 416→128`,
  `REGISTER_MAP_SIZE 0x3FF→0xFF`. That last one is used as a **bitmask**, so it must stay `2^n-1`.
- The stock `main()` runs a multi-second LED fade between comms calls. Harmless for USB pandas
  (interrupt-driven) but it **starves a polled UART** — exactly one transaction worked per reset
  until the fade was skipped for `PANDA_NUCLEO`.
- Clock is 180 MHz / APB1 45 / APB2 90; `stm32f4_config.h` must agree with `clock.h` or every
  baud rate and timer is wrong.

### Debugging on-target

```bash
sudo st-util -n &
gdb-multiarch -q -ex "target extended-remote localhost:4242" \
  -ex bt board/obj/panda_f446/main.elf
```
`LR=0xFFFFFFF9` with `xPSR & 0x1FF == 3` means you are sitting in HardFault.

## Next steps (in order — SAFETY FIRST)

1. SConscript entry to compile `board_nucleo`; build firmware with `arm-none-eabi-gcc`.
2. Flash via ST-Link; verify the F446 enumerates and pandad connects over serial.
3. **Bench CAN loopback** — CAN1↔CAN2 wired together, verify send/recv, BEFORE any car.
4. Keep comma's Mazda safety model (`safety_mazda`) fully intact. Never bypass safety on-car.
5. Only then: car harness, read-only CAN first, then actuation with extreme care.

## ⚠️ Safety

This ultimately controls a real car's steering/gas/brake. comma's safety model exists for a reason.
Bench-test everything. Read-only before actuation. Never rush the on-car step.
