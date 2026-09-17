# Findings

Bugs that cost real time, with the measurement that settled each one. Recorded because in
almost every case the first three explanations were wrong, and the thing that broke the
deadlock was an instrument, not an argument.

## Open

### STM32 USB endpoint race
*Guard flashed 2026-08-13. Not yet proven.*

`USB_WritePacket` arms the endpoint **before** filling its FIFO, leaving a window of about
16 store instructions where the endpoint is enabled and empty. EP1 uses `ITTXFE` — "IN
token received when TxFIFO is empty", an underrun notification — as its fill trigger. A
host token landing in that window raises it with `EPENA` already set, and the handler then
rewrote `DIEPTSIZ` on top of a transfer still in flight.

```
every bulk transfer   4 bytes        (asked for 16384)
CAN silicon           2327 frames/s  receiving normally
reaching the host        4 frames/s  99.8% loss
rx_buffer_overflow    1,163,362      and climbing
0xd8 board reset      clears it instantly
```

It failed after **11 minutes** on one drive and **61** on the next. A queue that is merely
too small fails at a repeatable load; a race fails whenever the timing lines up. That
variance is what redirected the investigation after three wrong answers — the radar, CPU
scheduling, and drain rate.

`comms_can_read` was ruled out arithmetically: with 15-byte packets into a 64-byte buffer
its overflow pointer cycles `0,11,7,3,14,10,6,2,13,9,5,1,12,8,4` and every state returns a
full 64 while the queue is non-empty.

The EP0 path has always checked `DTXFSTS` for room before writing. EP1 never did.
**The same code is unchanged in commaai/panda** — their shipping boards use SPI, so they
never load it.

### TIM2 and TIM9 clocks never enabled
*Found 2026-09-15. Not yet fixed.*

The July 25 Nucleo port rewrote `peripherals_init()` keeping only what the board is wired
to, and dropped upstream's whole timer block. Most of those timers genuinely have no
hardware here — fan PWM, IR, k-line. Two do not: **TIM2** is the microsecond clock and
**TIM9** is the supervisor tick.

Consequences, all silent, because an unclocked peripheral just reads 0:

- the supervisor tick never runs: no heartbeat fail-safe, no lost-message check, no
  hung-loop watchdog, `uptime` pinned at 0
- every time-based safety check sees no time pass — the 250 ms real-time torque window
  never advances, so `rt_torque_last` stays 0

This also explains the 2026-07-30 finding that the steering ceiling tracked `max_rt_delta`
exactly (450, then 900) — a simpler explanation than the jitter story recorded in
`mazda.h`. `max_rt_delta` was then raised to 2047 to stop it clamping, which leaves that
backstop doing nothing.

It was noticed on 2026-07-30 and written down as a board quirk, not traced to a cause. The
F407 port inherited the file verbatim. The same rewrite lost the USB clock enable too —
that one was caught, because a dead USB core hangs the firmware visibly.

## Settled

### Liveness from a decoded value
Twice. `set_speed_raw` was a latch that was never invalidated, which made the claim
"`0x21F` vanishes under suppression" unfalsifiable in either direction for weeks. Adding an
arrival timestamp proved the opposite: it survives.

Then `acc_active` latched, and the radar stayed suppressed for **264 seconds** with no
brake, cancel or MAIN-off able to clear it — because the release that frees it is driven by
the receive path that had died. The car logged fault codes throughout.

> **A decoded value can never establish liveness. Only an arrival time can.**

### A hardcoded follow distance overriding the model
`govern_accel` imposed a fixed 1.8 s time gap on the end-to-end action head — further back
than upstream's *relaxed* personality. It was not clamping the model, it was arguing with
it, and `min()` meant the formula won every disagreement.

```
708 firings measured
  417 (59%)  model asked to accelerate, rule commanded a brake
             e.g. a_raw=+1.28 → a_cmd=-0.71
  560 (79%)  fired at gaps ≥ 1.45 s, which upstream considers normal
  median gap at override: 1.60 s
```

### The ineffective recovery starving the effective one
Two watchdogs shared one cooldown timestamp. The rate-collapse one re-fires every ~15 s
during a collapse and runs first, so the wedge detector always saw 0 s elapsed and could
never fire — and `can_reset_communications` cannot clear a wedged USB endpoint, only a
`0xd8` can. Measured: wedged 264 s, then 452 s, with `wedge_recoveries` stuck at 0.
Simulated over 300 s: **0 board resets** before, first reset **within 25 s** after.

### A self-sustaining wrong lock
The packet stream has no delimiter, and the only validity tests were `bus <= 2`, an address
range and an 8-bit XOR. Weak enough that a wrong alignment passes often — and on a
repetitive bus the parser then locks onto it and keeps finding "valid" packets forever.
Measured: four minutes at full throughput (2316 → 2320 frames/s) with every decoded value
garbage, invisible to every watchdog because nothing was slow or missing.

Fixed with a per-packet `0xAA` marker, which decodes as bus 5 and so can never legally
start a packet.

### CRZ_CTRL's redundancy quad
Upstream templates invert it, so every synthesised `0x21c` was discarded by the PCM.

### bxCAN will not start on a sleeping bus
11 recessive bits are needed to leave initialisation, and a parked car never supplies them.
A wedged core reports no errors at all. Start the car first.

## Method

Three things this project kept relearning:

1. **Instrument before concluding.** Every wrong answer came from inferring a cause from
   damage instead of measuring it.
2. **Failure timing is diagnostic.** Repeatable load → capacity. Random → race.
3. **Make the test fail first.** If the old code passes your new test, the test proves
   nothing.
