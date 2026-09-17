# control_stack/ — openpilot control-and-safety stack brought up on the Jetson Orin

Scripts that stand up the real openpilot lateral pipeline on the Jetson for the
Mazda CX-5 2023, from the supercombo model through controlsd to a steering-torque
command. Everything here is **read-only / compute / print-only by default** — the
one exception is `dashcam_web.py --arm`, which does a real transmit and is
described under "Transmit / actuation" below. Run any script without `--arm` and
nothing reaches the car.

## Run environment (required for all scripts)

```bash
export PATH=$HOME/.local/bin:$PATH
export PARAMS_ROOT=/tmp/op_params
export PYTHONPATH=/home/tran/msgq_build:/home/tran/opendbc_src:/home/tran/op_fork:/home/tran/op_fork/openpilot
mkdir -p /tmp/op_params
```

Prereqs (one-time):
- `sudo apt install capnproto libcapnp-dev libzmq3-dev`
- `pip install --break-system-packages Cython cffi crcmod pyzmq setproctitle tinygrad`
- Build msgq: `git clone https://github.com/commaai/msgq.git ~/msgq_build && cd ~/msgq_build && scons -j4`
- Build cereal capnp C++ gen: `bash gen_capnp.sh`
- Build Params: `bash build_params.sh` (uses swaglog_stub.cc to avoid zmq/json11 chain)

## The pipeline (in dependency order)

| script | what it does |
|---|---|
| `test_bus.py` | verify the cereal message bus works (publish modelV2, receive it) |
| `gen_capnp.sh` | capnp -> C++ codegen for cereal (log/car/deprecated/custom) |
| `build_params.sh` + `swaglog_stub.cc` | compile common/params_pyx.so |
| `override_test2.py` | **safety regression test** — runs steering cmds through the REAL safety_mazda C code (libsafety). 16/16. Confirms disengaged->blocked, over-limit->blocked, rate->blocked, driver override widens envelope. |
| `init_controlsd.py` | build CX-5 CarParams, write to params store, instantiate `Controls()` |
| `run_controlsd_step.py` | drive controlsd through ONE control step with synthetic inputs -> prints the steering torque |
| `curvature_lib.py` | `path_to_curvature()` — proper path->curvature (quadratic fit, validated vs known arcs). supercombo outputs PATH not curvature. |
| `supercombo_publisher.py` | run supercombo on live IMX477 -> publish modelV2 onto the bus |
| `virtual_lkas.py` | model path -> 0x243 CAM_LKAS frame (counter+checksum), PRINT ONLY |
| `full_chain.py` | **the whole thing live**: camera -> supercombo -> curvature -> modelV2 bus -> controlsd -> steering torque |
| `dashcam.py` | whole pipeline as one system, real daemons, panda SAFETY_SILENT — records `would_command_torque`, transmits nothing |
| `dashcam_web.py` | same pipeline + phone dashboard; **the only script that can transmit** (`--arm`) — see below |
| `inspect_action.py` | probe supercombo step() output structure |
| `lane_fwd_test.py` | **regression for the 0x440 firmware-forward + steering-angle changes** — builds libsafety stock vs `-DPANDA_NUCLEO` and diffs the behaviour. 27/27, no hardware. |

## Key facts

- supercombo `step()` returns: path_xyz(33,3), path_vel, lane_lines(4,33,2),
  road_edges(2,33,2), lane_prob(8), lead_prob(3), pose(6), hidden_state(512).
  **NO curvature/action field** — curvature is derived (curvature_lib.py).
- controlsd subscribes to 14 topics, publishes carControl/controlsState.
  Mazda uses `LatControlTorque` (torque control, not angle/curvature).
- fork patches: `cereal/__init__.py` exports `car` (fixes capnp duplicate);
  `controlsd.py` curvature -> curvatureDEPRECATED (schema version skew).

## Transmit / actuation (`dashcam_web.py --arm`)

`dashcam_web.py` is the one script with a real transmit path, and it is **safe by
default**: with no flag the panda stays in `SAFETY_SILENT` and nothing is sent
(dashcam mode). Passing `--arm` (or `DASHCAM_ARM=1` in the env) switches it on:

- panda -> **Mazda safety model (13)**; a 50 Hz thread packs the real
  `0x243 CAM_LKAS` frame via `opendbc mazdacan.create_steering_control`, with
  torque rate-limited by `apply_driver_steer_torque_limits` (STEER_MAX 800,
  up 10 / down 25) straight from controlsd's `carControl.actuators.torque`.
  (Stock openpilot sends this at 100 Hz; the rate limit is per-message, so the
  halved rate halves the torque ramp to 500 units/s but blocks nothing.)
- `0x440 CAM_LANEINFO` is **forwarded cam->car by the firmware**, not by this
  script — see "CAN topology" in `jetson_port/README.md`. It used to be relayed
  here in Python over the serial link, which cost ~16 ms and dropped every 0x440
  but the last in each poll batch. **This needs the rebuilt firmware image**;
  with an older image the car gets no lane state and ignores the steering.
- the model's desired steering-wheel angle is derived from controlsd's clipped
  `actuators.curvature` through the `VehicleModel` (`actuators.steeringAngleDeg`
  is unusable — `LatControlTorque` always returns 0.0 for it). It is shown on
  the dashboard as `model_angle_deg` next to the measured `steer_angle_deg`, but
  is only written into `CAM_LKAS.STEERING_ANGLE` with `--steer-angle`. See the
  flag's help text for why that is opt-in.
- a **0xf3 heartbeat at 2 Hz** is mandatory — the panda firmware forces itself
  back to `SAFETY_SILENT` after 2–5 s without one, and revokes `controls_allowed`
  after 3 s unless the heartbeat carries `engaged=1`. The tx thread handles this.
- calibration is loaded from `~/openpilot_jetson/calib/op_calib.json` on startup
  (persisted across drives *and reboots*; override the dir with `OP_CALIB_DIR`).
  It used to live in `/tmp`, which systemd-tmpfiles wipes at boot. controlsd
  won't go `latActive` until `valid_blocks >= 5`.

The panda's Mazda safety is the hardware net: even armed, it passes a steering
frame **only while the car reports cruise engaged** (`controls_allowed`) and
clamps torque/rate to the Mazda limits validated by `override_test2.py` (16/16).
On exit the panda is re-asserted to `SAFETY_SILENT` first, before teardown.

First armed runs: car **stationary / wheels up, hand on the wheel, kill-switch
ready**. The harness must isolate the forward camera's own `0x243` from the main
bus — not just to stop two senders fighting over the ID, but because the panda
latches `relay_malfunction` the moment it sees `0x243`/`0x440` on bus 0, which
kills all tx *and* all forwarding until the safety mode is re-set.

Watch on the dashboard, in this order:
- `cam_age_s` / `cam_lane_age_s` — the camera is alive and being ACKed. `BIT_1`
  is latched from the last frame seen, so a stale camera keeps asserting "LKAS
  active" with nothing behind it.
- `controls_allowed` — the panda accepted the engage.
- `ck_ok` / `ck_bad` — mazdacan's 0x243 pack+checksum reproduced against the real
  camera. `ck_angle_seen` counts comparable frames with a *non-zero* angle; if it
  stays 0 the checksum's angle terms are still unproven and `--steer-angle`
  should stay off.

```bash
DASHCAM_ARM=1 python3 dashcam_web.py     # real transmit; actuates when you engage cruise
python3 dashcam_web.py                    # safe dashcam mode, transmits nothing
```

## Two-process split (`--role`)

Stock openpilot runs `pandad` (C++) and `card` (its own process), so the 100 Hz
CAN path shares no interpreter with the camera or the model. This port replaced
both with in-process threads and inherited a problem stock cannot have.

Measured 2026-08-09 on this board:

| competing CPU-bound Python threads | 10 ms deadline p99 | max | late |
|---|---|---|---|
| 0 | 10.02 ms | 10.21 ms | 0 |
| 1 | 10.07 ms | 10.14 ms | 0 |
| 3 | **442 ms** | **770 ms** | 21 |
| 3 doing numpy (releases the GIL) | 13.5 ms | 15.5 ms | 1 |

The CAN link is nowhere near the limit: `can_recv` 0.18 ms, `can_send` 0.19 ms,
frame parsing 2.75 us/frame, capnp building 107 us for all 17 topics — together
under 4% of a 10 ms period. The cap was GIL contention, not the transport.

### Running it

Start the panda role first; it owns the USB handle, and only one process may.

```bash
# process 1 -- pandad + card: CAN receive, 0x243 transmit, carState publish
python3 dashcam_web.py --role panda --arm --mads --lkas-hz 100

# process 2 -- camera, supercombo, message publishing, openpilot daemons
python3 dashcam_web.py --role model --display --display-view camera \
        --no-detect --no-depth --no-roadseg --no-voxels --no-freespace --no-scene
```

`--role both` (the default) is the historical single-process behaviour and stays
as the fallback. Nothing about it changed.

Effect on this board, car off, with `--role both` already stripped of detection,
depth, segmentation and scene:

| | threads | total CPU | busiest | threads >20% |
|---|---|---|---|---|
| `--role panda` | 9 | 11% of a core | 5% | **0** |
| `--role both` | 30 | 163% of a core | 27%, 25% | **2** |

### What lives where

`--role panda` owns the whole panda lifecycle: the open, the CAN-live gate,
arming (`SAFETY_MAZDA` and the alpha-long UDS session), the four CAN threads,
and the teardown back to `SAFETY_NOOUTPUT`. It publishes `carState`,
`pandaStates` and `carOutput`.

`--role model` never touches the panda. It takes `carState` over msgq and feeds
it into `_CarStateFromMsgq`, a shim presenting `CarStateFromCAN`'s attribute
names so the 45 sites reading `cs_can` — the model's `v_ego`, the angle-offset
learner, the desire helper, the display — keep working unchanged. It owns
`selfdrived` and `controlsd`, and publishes everything model-side.

Topics are split deliberately. The publish loop runs in both roles and sends
every topic it was given, so unsplit the panda role would publish an
all-defaults `modelV2` alongside the model role's real one — two publishers on
one topic, subscribers seeing whichever landed last.

### Caveats

`panda_lock` is still required. `can_thread` and `tx_thread` both remain in the
panda role, same process and same handle; the split moved the camera and the
model out, not the receive thread. Whether `tx_due`/`RECV_EVERY` still earn
their place is an open question that needs road data.

Only the Mazda-specific `carState` fields (`acc_armed`, `eps_request`,
`eps_effective`, `lkas_block`, `hands_off_5s`, `buttons`) have no home in the
cereal schema. They stay at their defaults in the model role rather than being
faked, which is why the `CAR` diagnostic line belongs to the panda role.
