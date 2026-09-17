# Jetson Mazda port baseline — 2026-07-22, revised 2026-07-25

Originally recorded the prototype state before the deterministic-build and
CAN-topology corrections, when the CAN hardware was not yet wired and working.
Revised 2026-07-25 after those corrections landed and the transport was verified
against the car; see "Known blockers and accuracy notes" for what changed.

Still not an actuation-ready release — but the remaining gate is bring-up
evidence (engagement, calibration, road data), not hardware readiness.

## Source revisions

| tree | revision |
|---|---|
| `~/op_fork` | `90676e683d66e621cc7bba45f12a6d75b0c2b871` |
| GitHub `origin/jetson-mazda-port` before preservation | `d23e3a6faacb98b61aec045fe7edd78f0cbdfe0c` |
| `~/opendbc_src` | `8ddffb37c8d8ea51727042ebe8743014ef5b5bfc` |
| `~/panda_base` / `~/panda_f446` base | `3dc21386239e3073a623156b75901aa302340d6c` |
| `~/msgq_build` | `425b61a60358895b894e9a6920b1d3e06903132e` |

The openpilot branch was three commits ahead of GitHub at audit start:
`c4f5d15`, `c807270`, and `90676e6`.

## Platform and toolchain

- Jetson: Linux `6.8.12-1021-tegra`, aarch64
- L4T: R39 revision 2.0, GCID 45755727
- Python 3.12.3
- GCC/G++ 13.3.0
- arm-none-eabi-gcc 13.2.1 (Ubuntu package `15:13.2.rel1-2`)
- Cap'n Proto 1.0.1
- TensorRT 10.16.2 (`trtexec` v101602)
- Cython 3.2.8, cffi 2.1.0, crcmod 1.7, pyzmq 27.1.0
- setproctitle 1.3.7, tinygrad 0.13.0, pyserial 3.5, pycapnp 2.1.0

## Preserved artifact hashes

```text
3a660fcc14342b60da0d998043de986522a4f726ebb0e76bae9318a148b90ff4  bootstub.panda_f446.bin
45b44a0c992ace18375db351c57614969e4be60c26527e2d93865ff094e1520f  panda_f446.bin.signed
103b0c10032f4a3c6639572b40f68b40ff41f64e15147b111d1934a48d5870c1  supercombo_fp32.trt
b447d4fd0329aab266fe2903ed3b565fe601808fe280eb26f55b52f509a7778d  params_pyx.so
3018c7ad8e66fa2358a473d01b14fa28d806ea319d0fdfa5b3e6fb69aeed06e5  ipc_pyx.so
```

## Verified at this baseline

- STM32 serial health response: 58 bytes.
- Vehicle-state response: 27 bytes; live Mazda traffic decoded on the MCU.
- Cereal/msgq publish/subscribe round trip passed.
- Mazda safety regression passed 16/16.
- Multi-process synthetic controlsd test returned carControl 3/3.
- The panda reconstruction patch applies cleanly to panda `3dc21386`.

## Known blockers and accuracy notes

Resolved on 2026-07-25 (superseding the 2026-07-22 entries; the original list was
written before the CAN hardware was wired and working):

- Firmware builds clean. The `vehicle_state_update()` `-Werror` failure was a
  header-ordering bug (`bxcan.h` called it before `vehicle_state.h` was included,
  and unconditionally); it is now included at point of use behind `PANDA_NUCLEO`.
  Same commit also unbroke the stock `panda` and `panda_h7` targets.
- CAN3-aliasing is gone. Physical CAN2 now presents as **logical bus 2** via the
  `bus_config` table, which is what openpilot's `get_fwd_bus()` (0 <-> 2) and the
  Mazda mode's `MAZDA_CAM = 2` require. `can_init_all()` bounds real init with
  `F446_CAN_CNT`; `can_set_orientation()` is a no-op on F446 (the stock 0<->2 swap
  assumed a can_number 2 that does not exist and would clobber the mapping).
- CAN bit timing retuned for the F446's 45 MHz APB1: 15 tq, SEQ1/SEQ2 = 11/3,
  SJW 3 -> exactly 500 kbps at an 80.0% sample point (was 86.7%, SJW 2).
- Serial `miso_len` is clamped. pandad requests `RECV_SIZE` (16384) on the CAN
  read endpoint against a 2048-byte `ser_tx`; unclamped, `comms_can_read()` wrote
  ~14KB past the buffer on every poll. This surfaced as "Panda CAN checksum
  failed" but was RAM corruption on a 128KB part.
- CAN ignition detection works. The stock Mazda hook keys off 0x9E (MSG_05),
  which the CX-5 2023 does not transmit, so `ignition_can` was false forever and
  pandad held NO_OUTPUT. Now derived from ENGINE_DATA (0x202) RPM; verified
  `ignition_can=1` with the engine running.

Verified on the car (2026-07-25, stationary, engine running):

- 2-minute sustained soak in SAFETY_MAZDA with both buses actively ACKing:
  93,432 frames received on bus 0 and **forwarded to the camera bus**, 2,179
  frames back from the LKAS module, REC=0, no error-passive/bus-off, zero rx loss.
- The LKAS module only transmits when the panda ACKs it (isolated-segment
  behaviour); in SAFETY_SILENT that bus is dead.
- `pandad` connects over the serial VCP and runs clean for 60s.
- Real TensorRT supercombo -> curvature -> modelV2 -> real `controlsd` process
  -> `carControl.actuators.torque`, on both synthetic and live IMX477 frames.
  Torque tracks curvature and reverses sign with it.
- Persistent camera capture: 59.7 fps, 60/60 frames (was ~1 frame / 15 s).

Still open:

- ~~**Auto-exposure does not work.**~~ RESOLVED 2026-07-26. The premise was wrong:
  it was never necessary to drive exposure from Python at all. `nvarguscamerasrc`
  has AE in the ISP; it was simply switched off, because `auto_exposure=False` set
  `aelock=true` and pinned the sensor at 30-33ms / gain 20-22.25 — a night tuning.
  Measured in daylight that gave mean 254.6/255 with **99.2% of the frame clipped
  to white**. Two further details that hid it: the pin was applied by BOTH branches
  of `_open()`, so `auto_exposure=True` was also locked; and `GAIN_MAX = 22.25` is
  outside the sensor's advertised `1 16`, so Argus rejected it ("Invalid max gain
  value ... using default maximum gain: 0.000000") and the comment claiming high
  gain improved night colour was describing something that never took effect.

  Fix: `auto_exposure="argus"` (now the default) hands the full envelope —
  34us..33ms, gain 1..16, isp 1..8 — to Argus with `aelock=false`, so AE runs per
  frame in hardware with no pipeline rebuild. Measured in the same daylight:
  mean 102.5, p1 39, p50 73, p99 213, **0.0% clipped**, converging in 1.5s and
  holding ±0.3 over 13s with no hunting. Verified end to end off the live
  `/stream.mjpg`. `autolevel_strength` is now 0 in `dashcam_web.py`: it existed to
  stretch a crushed night histogram back out, and stretching a correctly-exposed
  frame just moves the model's input off its training distribution.

  Night has NOT been re-verified since the change — the envelope reaches the same
  33ms/high-gain corner the old pin used, so it should be at least as good, but
  confirm before relying on it after dark.
- **Calibration has never been learned.** `~/openpilot_jetson/calib/op_calib.json`
  does not exist (it was `/tmp/op_calib.json` when this was written), so
  every frame so far was warped with rpy = (0,0,0). The online calibrator
  (`op_calibrate.py`, a `calibrationd` port) needs a straight drive at speed.
- **Engagement has never been demonstrated**, on bench or car. A bench harness
  driving the real `selfdrived` + `controlsd` confirms `pedalPressed` fires on
  brake and `steerOverride` fires on steering input, but the state machine
  correctly refuses to leave `disabled` without the real daemon set
  (`posenetInvalid`, `cameraMalfunction`, `sensorDataInvalid`, `usbError`).
- `is_onroad` needs `hardwared`, which is gated on `HasAcceptedTerms` and
  `CompletedTrainingVersion` — user-consent flags, deliberately not set by tooling.
- Wheel speeds in the 0xd5 vehicle-state packet are raw DBC values; the
  documented `kph*100` contract omits the Mazda `-100 km/h` offset.
- The demonstrated control chain ends at `carControl`. Mazda CarController,
  mazdacan, pandad, and panda safety have not yet been joined into one send path.
- The model has never seen a road. Every frame to date is a parking lot or
  synthetic.

Actuation gate: the CAN transport, forwarding path and safety-mode plumbing are
now demonstrated on the car, so the hardware objection in the 2026-07-22 baseline
no longer applies. The remaining gate is **not** hardware readiness — it is that
engagement is undemonstrated, calibration is unlearned, and no road data exists.
Recommended next step is a **dashcam-mode drive** (camera mounted, model running,
calibration learning, everything logged, panda in SAFETY_SILENT) before any
actuation.
