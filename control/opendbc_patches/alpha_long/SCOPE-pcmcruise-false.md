# Scope: openpilot owns ACC engagement on the CX-5 (`pcmCruise = False`)

Written 2026-08-12, after the on-car A/B in `FINDINGS-2026-08-11.md`. **This is a
design scope, not an instruction to build.** It ends with a reflash, so nothing
here happens without the disassembly/hash evidence and an explicit go-ahead.

## Why

Every alpha-long configuration tried across two evenings sits inside one loop:

- `ret.pcmCruise = True` → openpilot sets `longActive` only once the car reports
  cruise engaged.
- `build_crz_info` asserts `ACC_ACTIVE = int(long_active)` → we announce ACC only
  once we are already active.
- The PCM engages only when told to, and the thing that told it was the radar,
  which alpha long removes.

Nobody moves first. Measured 2026-08-12: fourteen clean SET presses on `0x09d`,
`setspd` frozen at raw=100, `acc_active` 0 for 100 s, 9007 longitudinal frames
transmitted with `err=0` and `tx_blocked=0`. The frames are correct and the car
is listening; there is simply no first mover.

Breaking the loop means openpilot engages itself on the SET edge it can already
see, and asserts `ACC_ACTIVE` unprompted.

## What already exists

More than expected — the upstream PR anticipated this and then did not use it.

- **`opendbc/car/mazda/carstate.py:27` `update_button_enable()` is already
  written** and is dead code today, because its body is `if not self.CP.pcmCruise`.
  It engages on `accelCruise` **press** or `decelCruise` **release**.
- `opendbc/car/interfaces.py:261` already calls it into `ret.buttonEnable`.
- `openpilot/selfdrive/car/car_specific.py:150` already turns `CS.buttonEnable`
  into `EventName.buttonEnable`; `:101` computes
  `pcm_enable = self.CP.pcmCruise and brand != 'honda'`, so flipping the flag
  moves engagement from the PCM path to the button path with no edit there.
- `car_specific.py:157` already handles cancel-button disable when `not pcmCruise`.
- `openpilot/selfdrive/car/cruise.py` `VCruiseHelper._update_v_cruise_non_pcm()`
  already owns the set speed off button events when `not pcmCruise`.
- `opendbc/safety/modes/hyundai_common.h:91` is the canonical safety-side
  pattern: arm `controls_allowed` on the **falling edge** of SET or RES, clear it
  on CANCEL. Mazda's buttons are all in byte 0 of `0x09d`
  (`CAN_OFF=0, RES=2, SET_P=4, SET_M=5`), and `GET_BIT` uses the same numbering
  the DBC start bits do, so the port is nearly mechanical.

## What has to change

### 1. `opendbc/car/mazda/interface.py`

`ret.pcmCruise = False` when `openpilotLongitudinalControl`, and only then. The
non-alpha-long path must keep `pcmCruise = True` — stock MRCC really does own
engagement there.

### 2. `opendbc/car/mazda/carstate.py`

Under `openpilotLongitudinalControl`, stop reporting `cruiseState.enabled` from
`PEDALS.ACC_ACTIVE`. **This is a trap**, not a tidy-up:

```python
# selfdrived.py:425
cruise_mismatch = CS.cruiseState.enabled and (not self.enabled or not self.CP.pcmCruise)
```

With `pcmCruise = False`, *any* true `cruiseState.enabled` raises a permanent
cruise mismatch. It reads clean today only because `ACC_ACTIVE` never goes 1 —
i.e. it would start faulting at exactly the moment the change starts working.

`cruiseState.available` off `PEDALS.ACC_OFF` should stay: MAIN-on is still the
car's own state and still the right precondition.

### 3. `opendbc/safety/modes/mazda.h` — **the reflash**

- Under `mazda_longitudinal`, arm on the falling edge of `RES` (bit 2) or
  `SET_M` (bit 5) in `MAZDA_CRZ_BTNS`, mirroring
  `hyundai_common_cruise_buttons_check`. Keep the existing CANCEL clear at
  `mazda.h:167-172`; it is already correct.
- **Remove the `pcm_cruise_check()` call from the PEDALS branch under
  `mazda_longitudinal`** (`mazda.h:178-220`). It is driven by `ACC_ACTIVE`, which
  will stay 0, so it would clear `controls_allowed` immediately after every
  button arm. Leaving it in is the most likely way for this change to look like
  "the panda still blocks us" rather than "the gate is fighting itself".
  `acc_main_on` from the same branch stays.
- The `CRZ_INFO` accel window, the `CRZ_CTRL`-requires-`controls_allowed` check
  and the `0x764` UDS allowlist all stay exactly as they are. Once
  `controls_allowed` can actually become true, the existing
  `CRZ_ACTIVE && !controls_allowed → block` rule starts doing its intended job
  instead of blocking everything.

### 4. This port's host shim — **the part a naive flip would miss**

`dashcam_web.py` does not use opendbc's `CarState`. It builds `carState` by hand,
and it **never populates `buttonEvents` or `buttonEnable`** (grep: zero hits), and
hardcodes `cs.cruiseState.speed = 25.0` at `dashcam_web.py:4323`. So on this port
the entire button-engagement path is unplumbed and flipping `pcmCruise` alone
would change nothing except break the current engage gate.

Needed:
- populate `cs.buttonEvents` from `cs_can.buttons` — the decode already exists in
  `dashcam.py`, and the rising-edge counters added 2026-08-12 confirm the frames
  are read correctly on this car;
- populate `cs.buttonEnable` the way `interfaces.py:261` would;
- feed a real `cs.cruiseState.speed` from `VCruiseHelper` rather than the 25.0
  constant, or openpilot will chase a fixed 25 m/s;
- retire the `OP_ALPHA_ENGAGE` gate at `dashcam_web.py:4260-4271` for the
  alpha-long case — engagement stops being something this file decides.

### 5. Tests

`opendbc/safety/tests/test_mazda.py::TestMazdaLongitudinalSafety` currently
inherits the PCM-status engage helper from `TestMazdaSafety`. Button arming needs
the button-driven equivalents, plus a case for each of: arm on SET release, arm
on RES release, no arm on press alone, CANCEL clears, and **`ACC_ACTIVE` staying 0
does not clear** — that last one is the regression guard for the `pcm_cruise_check`
removal above.

## Consequences to accept, not discover later

- **No set speed on the cluster.** `0x21F` is the PCM's frame and we do not drive
  it, so the dash will not show what openpilot is targeting. openpilot's own UI
  will. This follows from the A/B result and is not fixable by synthesising
  `0x21F` — the PCM is already sending it, and a second sender would collide.
- **`resumeBlocked`** (`selfdrived.py:221`) fires if RES is pressed before
  `v_cruise` is ever initialised. Expected under button engagement; SET first.
- **MADS.** `dashcam_web` refuses `--alpha-long --mads` today because MADS gated
  engagement inside the `CRZ_CTRL` rx handler that alpha long suppresses. Once
  engagement is button-driven, that argument no longer holds and the interaction
  needs deciding again rather than inheriting the old refusal.
- **The driver's SET press now means something different** — it engages
  openpilot's longitudinal rather than the car's ACC, with no cluster feedback
  saying so. Worth a deliberate decision about how that is signalled.

## Order of work, and where it can stop

1. Host shim (4) and the car port (1, 2) — no reflash, and testable on the bench
   against the existing `libsafety`.
2. Safety change (3) plus tests (5) — bench only: `override_test2.py`,
   `lane_fwd_test.py`, and the new button cases must pass against a locally built
   `libsafety` before any firmware is written.
3. Reflash — build, disassemble, hash, show the evidence, **ask**. Never chained
   to the build.
4. Car, stationary, wheels chocked, engine running, kill switch ready: MAIN on,
   press SET, watch `btn SET-` increment and `ctrl_allowed` go 1. Only after that
   is reproducible, a commanded accel from standstill with a foot over the brake.

Steps 1-2 are reversible and prove most of the design. If the panda arms on the
button and `CRZ_CTRL` starts passing, the deadlock is broken and the rest is
tuning. If it arms and the car still ignores `ACC_ACTIVE`, then the PCM wants
something else entirely and this whole approach is wrong — which is worth knowing
before the tuning work, and is the reason to stop at step 4 and look rather than
push on.

## What this does not do

It does not restore FCW/AEB/SBS — those are gone whenever the radar is muted, by
both routes, measured. It does not make the cluster quiet: FSM/FSBM appear under
the programming session and under standby alike. And it does not change the
lateral path at all; `override_test2.py` and `lane_fwd_test.py` are regression
guards here, not targets.
