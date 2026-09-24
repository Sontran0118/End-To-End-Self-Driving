# Mazda Alpha Longitudinal — CX-5 2022–25

openpilot commands throttle and brake on the CX-5 by silencing the factory radar
and sending the frames it would have sent.

**This disables FCW, AEB and SBS for as long as it is enabled, and the cluster
will show malfunctions. That is the mechanism, not a bug.** The radar is the ECU
that runs all of them; suppressing it to take longitudinal necessarily takes them
too. Camera-based low-speed SCBS survives.

## How it works

The radar at UDS `0x764` on bus 0 owns longitudinal: it sends `0x21b` CRZ_INFO
(carrying `ACCEL_CMD`) and `0x21c` CRZ_CTRL, and the PCM obeys. Put the radar into
a UDS **programming session** and hold it there with raw tester-present
(`0x3E 0x80`), and it stops transmitting — at which point those two frames are
ours. Positive `ACCEL_CMD` accelerates, negative brakes. `0x21c` must be replaced
too or a fault locks out cruise.

Suppression is *soft*: stop sending tester-present and the radar times out back to
stock on its own. That is the intended failure direction.

## Provenance

Not invented here. From [community.sunnypilot.ai thread 1482](https://community.sunnypilot.ai/t/longitudinal-for-mazda/1482)
— on-car proof 2026-03-20 (yummydirt), stop-and-go solved 2026-04-23. Code from
`yummydirtx/opendbc @ mazda-longitudinal-upstream`. **Not upstream:**
`commaai/opendbc` master still has `MazdaFlags = GEN1` only.

## Layout

`overlay/` mirrors the opendbc tree, so installing is a copy:

```
overlay/opendbc/car/mazda/longitudinal.py   NEW — vendored verbatim
overlay/opendbc/car/mazda/carstate.py       hand-merged for 8ddffb37
overlay/opendbc/car/mazda/interface.py      hand-merged, keeps the measured 0.2 s delay
overlay/opendbc/safety/modes/mazda.h        hand-merged, keeps MAZDA_MADS + tuned limits
```

## Install

Two files still need the upstream hunks — their bases are **byte-identical**
between `opendbc_src` and `opendbc_repo`, so they apply cleanly:

```bash
cd ~/opendbc_src
curl -sL "https://github.com/commaai/opendbc/compare/master...yummydirtx:opendbc:mazda-longitudinal-upstream.diff" -o /tmp/mazda_long.diff
git apply --include='opendbc/car/mazda/carcontroller.py' \
          --include='opendbc/dbc/mazda_2017.dbc' /tmp/mazda_long.diff
cp -r ~/op_fork/jetson_port/opendbc_patches/alpha_long/overlay/* .
git diff --stat        # review before building anything
```

Then rebuild the F446 firmware (`mazda.h` is compiled into it) and flash.

## Verify before driving

Nothing below was run — the session that wrote this had no panda, no car, and was
blocked from compiling. **Do all of it.**

```bash
# 1. safety model compiles clean at -Werror, both build variants
gcc -shared -fPIC -Wall -Wextra -Werror -nostdlib -fno-builtin -std=gnu11 \
    -Wno-pointer-to-int-cast -DCANFD -DPANDA_NUCLEO \
    -I ~/op_fork/jetson_port/opendbc_patches/alpha_long/overlay -I ~/opendbc_src \
    -o /tmp/libsafety_long.so ~/opendbc_src/opendbc/safety/tests/libsafety/safety.c

# 2. existing lateral regression still passes
python3 ~/op_fork/jetson_port/control_stack/lane_fwd_test.py     # expect 27/27

# 3. the DBC signals longitudinal.py patches actually exist, and the packer's
#    ACCEL_CMD round-trips through the safety hook's unpacking
python3 control/opendbc_patches/alpha_long/test_long.py    # written, never run
```

Then, car stationary and wheels chocked: confirm `enter_radar_programming_session`
returns True, and that `0x21b` stops arriving from the radar before you ever move.

## Choosing how the radar is silenced (`OP_RADAR_SUPPRESS`)

`interface.py`'s `init()` picks between two routes, and they are **not**
interchangeable. The arm log prints which one ran (`radar suppression: mode=...`);
before 2026-08-12 it did not, which is why the older `d_p.*.log` files cannot be
attributed to either.

| value | what it sends | cost |
|---|---|---|
| `programming` | `0x10 0x02` DiagnosticSessionControl → programmingSession | every other module logs a communication DTC against the radar; cluster says *malfunction*. **This is what the working community port uses.** |
| `standby` (default) | extended session, `0x85` ControlDTCSetting OFF, `0x28` ENABLE_RX_DISABLE_TX | radar stays present and answering; should keep those DTCs off the cluster. Falls back to `programming` if the radar NAKs `0x28`. |
| `standby_keepdtc` | as above, without the `0x85` | for isolating whether DTC storage is what the cluster reacts to |

The default is `standby` only while `longitudinal.py` still *has* the standby
ladder; the pristine vendored module does not, and the resolution falls back to
`programming`.

**This choice may decide whether the car engages at all**, not just what the dash
says — see `FINDINGS-2026-08-11.md`. Run both.

## Engage first, suppress second (`OP_ALPHA_ENGAGE_FIRST=1`)

Every failure this port has had is at the ACC **entry** transition. MEASURED
2026-08-12, with our frames byte-identical to a real activation, the panda arming
from the button, openpilot enabled and asking for up to +0.99 m/s²: the PCM's
`acc_active` stayed 0 across every sample of every run. Entry has never worked.
**Sustain has never been tested.**

This mode stops asking the PCM to enter ACC:

1. Arm normally, but leave the radar **alive** — no UDS, no suppression.
2. The driver presses SET. The radar runs the entry handshake it has always run,
   the PCM enters ACC, and `acc_active` goes 1. openpilot engages off that
   transition, which is exactly what upstream's `pcmCruise = True` expects.
3. Our `0x21b`/`0x21c` open **first**, overlap the radar for `OP_ALPHA_OVERLAP_S`
   (default 0.30 s), and only then is the radar suppressed. The PCM sees two
   sources briefly rather than a gap; a gap is what would make it drop ACC.
4. From there, `acc_active` holding means the PCM is obeying **us**.

```bash
OP_ALPHA_ENGAGE_FIRST=1 OP_RADAR_SUPPRESS=programming DASHCAM_ARM=1 \
  python3 dashcam_web.py --role panda --alpha-long
```

Watch for `>>> HANDOVER:` in the log.

**IT WORKS. MEASURED ON THE CAR 2026-08-12**, the first working longitudinal on
this vehicle:

```
16.1s  24.7 kph  op=disabled   a_cmd -0.10
17.1s  25.8 kph  op=ENABLED    a_cmd +0.25    <- SET pressed, handover fires
18.1s  27.8 kph  op=ENABLED    a_cmd +0.65
19.1s  31.4 kph  op=ENABLED    a_cmd +0.70
20.1s  37.4 kph  op=ENABLED    a_cmd +0.69
21.1s  38.8 kph  op=disabled                  <- driver braked
```

Measured +1.01 m/s^2 against a commanded +0.65..+0.70, off-throttle, for three
seconds, ending only on the brake. Stock ACC set at 25.8 kph would have HELD ~26;
instead the car accelerated toward openpilot's `vtgt`. The PCM obeyed our
ACCEL_CMD.

### The re-arm loop

ACC ends the moment the driver brakes or cancels, and only the RADAR can perform
another ACC entry -- our frames cannot, which is the entire reason this mode
exists. So the handover is a **loop**, not one-shot:

```
wait acc_active=1  -> open our tx, overlap, suppress radar   (openpilot drives)
wait acc_active=0  -> close our tx, STOP tester-present      (brake/cancel)
                      session lapses, radar returns in ~5 s, AEB and dash restored
       loop back   -> press SET again, radar runs entry, we take over again
```

Releasing is just dropping tester-present; requesting the default session
explicitly makes this radar fault. Without the release you get exactly ONE
engagement per run, and are left with a suppressed radar while ACC is off -- no
AEB and a dash warning with nothing using the suppression. OBSERVED 2026-08-12
before the loop existed.

`>>> RELEASE COMPLETE` confirms the radar is transmitting again. If it does not
return within 20 s the thread gives up and says so; steering is unaffected.

Two consequences of leaving the radar alive until the handover:

- the **radar shadow is disabled** for the whole run (replaying its seven frames
  while it is still sending them would put a second counter-bearing copy of each
  on the bus). After the handover there is no shadow, so the front camera fault
  may appear at that point.
- the **radar-return watchdog is held off** until `handover_done`, since the
  radar owning `0x21b` is the design until then. It rebaselines each window so
  the first post-handover check is not poisoned by the engage-first phase.

## Reading a run

Three lines answer the questions this port keeps getting wrong:

- `CENSUS ... bus0 Hz (ours subtracted)` — per-address rates for the longitudinal
  frames, the two the panda gates on, and the radar's other seven. Our own
  transmits are subtracted, because everything we send echoes back through
  `can_census`. **`21f` is the one to read first**: nothing here has ever
  transmitted it, so its rate is the car's alone. ~50 Hz means the radar's HMI
  half is still running; 0 means it went with the rest.
- `setspd raw=... age=` — `raw` is a latch that is never invalidated, so it keeps
  reading plausible after `0x21F` stops. `age` is the live one: `-1` never seen,
  climbing past ~0.1 s means the 50 Hz frame has stopped.
- `RADAR ... age=` — time since that address was last on the bus, which once the
  shadow replay is running is **our own echo**. Near-zero here says the replay is
  healthy, nothing about the radar.

`OP_SHADOW_MAX_AGE_S` (default 20) bounds how long a captured radar frame keeps
being replayed. `0x366`/`0x499` are exempt — the DBC defines no varying signals
for them. Everything else expires, loudly, because a frozen lead is a false
statement about the road and the camera cross-checks it.

## Things that will bite

**Panda safety param.** `mazda_init()` reads bit 0 of the safety param to pick the
longitudinal TX table. `dashcam_web` now sends `1` when `--alpha-long` is set. Left
at `0` the panda silently rejects every `0x21b`, which looks like "the car ignores
us" rather than "wrong firmware mode".

**`MAZDA_MADS` and alpha-long are mutually exclusive.** MADS gates engagement
inside the `CRZ_CTRL` rx handler; alpha-long suppresses `CRZ_CTRL`, so that handler
is skipped and engagement reverts to the strict ACC-engaged gate. `dashcam_web`
refuses `--alpha-long --mads` rather than let MADS look like it did something.
This is the safe direction — stacking throttle authority on the permissive
MAIN-on gate would be a bad combination.

**`0x76C` must reach the host.** `IsoTpParallelQuery` uses `response_offset = 0x8`,
so the radar answers on `0x764 + 8`. It's been added to `mazda_filter.h`
(`MAZDA_HOST_IDS_LEN` 17→19, with `0x21B`). Without it the ISO-TP query blocks
forever and suppression silently never happens.

**`check_relay = false` on the three new senders.** The radar *is* a bus-0 ECU
being silenced in software, so a briefly-reviving radar — a real reported failure —
must not latch `relay_malfunction` and take steering down with it.

**No planner.** There is no plannerd and no longitudinal MPC on this board.
`longitudinalPlan.aTarget` is fed from the **model's own action head**. That is
openpilot's e2e longitudinal path, not a substitute for it — but it means no
lead-distance MPC, no jerk limiting, no speed-limit or curvature slowdown, and no
cruise-speed tracking. The car accelerates and brakes on what the network predicts
a human would do, clipped only by the accel scale and the panda's ±2000 window.

**`longitudinalActuatorDelay = 0.36` is not measured on this car.** Unlike
`steerActuatorDelay = 0.2`, which was measured by cross-correlating `LKAS_REQUEST`
against `LKAS_EFFECTIVE` in `0x241`. Re-measure it the same way before trusting
stop-and-go timing. It also feeds `op_stream.LONG_ACTION_T`.

## Still to do

`long_tx_thread` in `dashcam_web.py` drives the real opendbc `CarController` and
forwards only its longitudinal frames, because the stop-and-go state machine is
stateful and not worth reimplementing. It feeds that controller a **shim**
standing in for opendbc's `CarState`, which this process never builds. The shim
covers the fields `carcontroller.py` reads today — if it reads another one, the
thread prints `long_tx: CarController.update failed` rather than silently
commanding zero. **Watch for that line on the first bench run**; `CS.accel_button`
in particular is mapped to a `cs_can.res_button` attribute that may not exist,
in which case physical RES-to-resume from a stop will not work.
