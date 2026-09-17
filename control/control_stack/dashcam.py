#!/usr/bin/env python3
"""DASHCAM MODE — the whole pipeline running as one system, transmitting nothing.

  IMX477 (persistent capture, 60fps)
     -> supercombo (TensorRT)
     -> path -> curvature -> modelV2
     -> [cereal bus] -> selfdrived + controlsd (real daemons, own processes)
     -> carControl.actuators.torque          <-- RECORDED, never sent
  panda (SAFETY_SILENT, both buses) -> carState from real CAN

Purpose: get the four things that are still missing before actuation can be
justified --
  1. engagement actually reaching `enabled` (never yet observed, bench or car)
  2. a LEARNED calibration (rpy has been (0,0,0) for every frame so far)
  3. real road frames (every frame to date is a parking lot or synthetic)
  4. a replayable log of what torque WOULD have been commanded

The panda stays in SAFETY_SILENT for the whole run and is explicitly re-asserted
to SILENT on exit. Nothing is ever written to CAN.

Usage:  python3 dashcam.py [--seconds N] [--no-model]
"""
import os, sys, time, json, signal, subprocess, argparse, glob

os.environ.setdefault("PARAMS_ROOT", "/tmp/op_params")
sys.path.insert(0, "/home/tran/openpilot_jetson")
sys.path.insert(0, "/home/tran/op_fork/jetson_port")

import numpy as np
import openpilot.cereal.messaging as messaging
from cereal import log
from opendbc.car.structs import car as car_struct
from opendbc.car.mazda.values import CAR
from opendbc.car.mazda.interface import CarInterface
from openpilot.common.params import Params
from curvature_lib import path_to_curvature, T_IDXS
from bench_can_loopback import SerialPanda

SAFETY_SILENT = 0
LOG_PATH = os.environ.get("DASHCAM_LOG", "/tmp/dashcam_%d.jsonl" % int(time.time()))

# --- real CAN signal positions, verified against the car (see cruise_watch) ----
CRZ_CTRL, CRZ_BTNS, PEDALS, STEER_TORQUE, ENGINE_DATA, WHEELS = \
    0x21c, 0x09d, 0x165, 0x240, 0x202, 0x215
# CRZ_EVENTS carries CRZ_SPEED -- the ACC SET SPEED, the one number needed to move
# the factory cruise setpoint in closed loop instead of open-loop press-counting.
# It was already in MAZDA_HOST_IDS and simply never decoded.
#
# WHY IT MATTERS BEYOND BUTTONS: carState.cruiseState.speed is hardcoded to 25.0
# in dashcam_web.py, and the MPC is handed V_MAX_KPH as its target. Both mean the
# solver is permanently asked to reach a speed far above the current one, so its
# output saturates -- MEASURED 2026-08-11, longitudinalPlan.aTarget sat at exactly
# 1.200 (the OP_ACCEL_MAX cap) and carControl accel at 2.000, never once taking an
# intermediate value over 819 samples. A longitudinal controller built on a
# saturated constant would just wind the set speed to its limit and stop.
CRZ_EVENTS = 0x21f
STEER = 0x82   # STEER (130): STEER_ANGLE big-endian 16b @bit23, 0.05 deg, -1600 offset
STEER_RATE = 0x241   # STEER_RATE (577): the EPS's OWN account of the LKAS command
BLINK_INFO = 0x09a   # BLINK_INFO (154): turn signal lamps -> DesireHelper
BSM = 0x477          # BSM (1143): blindspot status, the lane-change veto
# Driver-override threshold on STEER_TORQUE_SENSOR (0x240 byte0, offset -127).
#
# MEASURED 2026-08-08 while engaged: |driver torque| median 22.5, mean 20.6,
# max 37 -- with hands merely RESTING on the wheel. At 15 that tripped
# steeringPressed in 19 of 28 engaged samples (68%), and openpilot sat in
# `overriding` for 27 of 28.
#
# Why that breaks curves specifically: LatControlTorque does
#     freeze_integrator = steer_limited_by_safety or CS.steeringPressed or vEgo < 5
# so resting hands FREEZE THE INTEGRATOR. Entering a bend the feedforward turns
# the car in, then the integrator -- which is what holds the extra torque a
# sustained curve needs -- stops accumulating, torque decays to feedforward
# alone, and the wheel unwinds back toward centre. That is the "steers into the
# curve then returns" symptom exactly, and no rate or gain change fixes it
# because the loop is being told the driver has taken over.
#
# 15 -> 45: above resting-hand torque (max 37 measured) but well under a real
# grab. This makes openpilot LESS sensitive to override, so a genuine takeover
# needs a firmer input -- the panda's own limits and the brake/pedal disengage
# are unchanged, and the driver can always brake or cancel.
STEER_THRESHOLD = float(os.environ.get("OP_STEER_THRESHOLD", "45"))
BLINKER_HOLD_S = 0.4    # bridge the lamp's off-phase; upstream uses 40 frames


def bit(d, s):
    return (d[s // 8] >> (s % 8)) & 1


class CarStateFromCAN:
    """Decode the real bus into the fields carState needs. Mirrors opendbc's
    MazdaCarState for the signals we have verified on this car."""

    def __init__(self):
        self.v_ego = 0.0
        self.cruise_available = False
        self.cruise_enabled = False
        # ACC set speed off CRZ_EVENTS. -1 means "never decoded", which is
        # distinguishable from a genuine 0 -- a controller must not treat an
        # absent set speed as "the driver asked for zero".
        #
        # THESE THREE ARE A LATCH, NOT A LIVE READ. They hold the last decoded
        # value forever and are never invalidated when 0x21F stops arriving, so
        # a stale reading is indistinguishable from a live one BY VALUE. -1
        # only means "never arrived since this object was built" -- on a car
        # that was awake at startup, 0x21F latches before suppression and the
        # field then reads plausible for the rest of the run.
        #
        # That is exactly what made "0x21F vanishes under radar suppression"
        # unfalsifiable from the logs: every suppressed run shows raw=100 (the
        # idle value latched at startup) and the -1 runs are the ones where the
        # car was ASLEEP from the start. set_speed_t is the field that settles
        # it -- 0.0 means never seen, and anything else is a real arrival time.
        self.set_speed_raw = -1
        self.set_speed_kph = -1.0
        self.set_speed_ms = -1.0
        self.set_speed_t = 0.0
        # PEDALS-derived MRCC state -- see the PEDALS branch in update().
        self.acc_armed = False
        self.acc_active = False
        self.brake_pressed = False
        self.gas_pressed = False
        self.steering_torque = 0.0
        self.steering_pressed = False
        self.steering_angle = 0.0
        self.rpm = 0
        self.buttons = dict(set_p=0, set_m=0, res=0, off=0)
        # Rising edges per button since startup, and when the last one landed.
        # See the CRZ_BTNS branch in update() for why counts and not levels.
        self.btn_counts = dict(set_p=0, set_m=0, res=0, off=0)
        self.btn_last_t = 0.0
        # CRZ_BTNS rolling counter. create_button_cmd() sends (counter + 1) % 16,
        # so a virtual RES/CANCEL only looks like the next frame in the car's own
        # sequence if this tracks the real one. Frozen -> every button frame
        # carries the same CTR and the ACC module ignores it.
        self.crz_btns_counter = 0
        self.seen = 0
        # --- EPS feedback (0x240 / 0x241) ------------------------------------
        # The ONLY direct evidence of what the steering rack does with a command.
        # Everything else in this stack is what we ASKED for; these are what the
        # EPS did about it. eps_effective vs eps_request in particular is the
        # EPS's own commanded-vs-applied pair, so a divergence between them is
        # the EPS clamping us and nothing else -- which is the one question
        # STEER_MAX/ramp tuning cannot answer from the sending side.
        self.eps_motor_torque = 0.0    # STEER_TORQUE_MOTOR, 0x240, 0.1 units
        self.eps_request = 0           # LKAS_REQUEST as the EPS received it
        self.eps_effective = 0         # LKAS_EFFECTIVE, what it actually applied
        self.steer_angle_rate = 0.0    # deg/s
        self.lkas_block = False        # EPS refusing LKAS (steerFaultTemporary)
        self.hands_off_5s = False      # the hands-off lockout timer
        self.lkas_track_state = 0
        self.eps_seen = 0              # 0x241 frames decoded
        # --- lane change inputs ----------------------------------------------
        self.left_blinker = False
        self.right_blinker = False
        # last time each lamp was seen lit; see the debounce note in update()
        self._left_lamp_t = 0.0
        self._right_lamp_t = 0.0
        self.left_blindspot = False
        self.right_blindspot = False

    def update(self, addr, d):
        if len(d) != 8:
            return
        self.seen += 1
        if addr == CRZ_CTRL:
            self.cruise_available = bool(bit(d, 17))
            self.cruise_enabled = bool(bit(d, 3))
        elif addr == CRZ_EVENTS:
            # CRZ_SPEED : 7|16@0+ (0.005, -0.5) -- big-endian, start bit 7 is the
            # MSB of byte 0, so the value spans bytes 0..1.
            #
            # UNITS ARE km/h. MEASURED 2026-08-12 with stock MRCC engaged and
            # holding a set speed: raw=14582, which is 72.4 as km/h and 260.7 as
            # m/s-converted-to-km/h. A CX-5 has no 260 km/h ACC set speed, so the
            # m/s reading is eliminated. (72.4 km/h is also exactly 45.0 mph, so a
            # US cluster displaying 45 is consistent with the same conclusion.)
            #
            # set_speed_ms is kept only so existing readers do not break; it is
            # the disproven interpretation and nothing new should use it.
            self.set_speed_raw = (d[0] << 8) | d[1]
            _v = self.set_speed_raw * 0.005 - 0.5
            self.set_speed_kph = _v          # if the DBC unit is km/h
            self.set_speed_ms = _v           # if it is m/s (x3.6 for km/h)
            # Arrival time, so a reader can tell a live 0x21F from a latched
            # one. See the note on these fields in __init__ -- without this the
            # value alone cannot answer whether suppression silences CRZ_EVENTS.
            self.set_speed_t = time.time()
        elif addr == CRZ_BTNS:
            _prev_btns = self.buttons
            self.buttons = dict(set_p=bit(d, 4), set_m=bit(d, 5),
                                res=bit(d, 2), off=bit(d, 0))
            # RISING-EDGE COUNTS, because the instantaneous state is useless to a
            # 1 Hz diagnostic: a button is held for a few hundred ms and the
            # sample almost always lands between presses. Two alpha-long runs on
            # 2026-08-12 asked "did SET do anything?" and could not answer it,
            # because nothing recorded that SET was pressed at all.
            for _k in self.btn_counts:
                if self.buttons[_k] and not _prev_btns.get(_k):
                    self.btn_counts[_k] += 1
                    self.btn_last_t = time.time()
            # CTR : 29|4@0+ -- big-endian, start bit 29 is the MSB, so byte 3
            # bits 5..2. Same start-bit convention bit() already assumes above.
            self.crz_btns_counter = (d[3] >> 2) & 0x0F
        elif addr == STEER_TORQUE:
            self.steering_torque = float(d[0] - 127)
            self.steering_pressed = abs(self.steering_torque) > STEER_THRESHOLD
            # STEER_TORQUE_MOTOR: 46|15@0- (0.1) -- big-endian, start bit is the
            # MSB, so byte5 bits 6..0 then all of byte6. Signed in 15 bits, NOT
            # 16: sign-extend at 0x4000 or every left-hand torque reads as a
            # large positive number.
            raw = ((d[5] & 0x7F) << 8) | d[6]
            if raw >= 0x4000:
                raw -= 0x8000
            self.eps_motor_torque = raw * 0.1
        elif addr == STEER_RATE:
            # LKAS_REQUEST: 3|12@0+ (-2048) -- byte0 low nibble + byte1. Same
            # layout as CAM_LKAS's own LKAS_REQUEST, which _validate_cam_checksum
            # in dashcam_web.py already decodes this way against the real camera.
            self.eps_request = (((d[0] & 0x0F) << 8) | d[1]) - 2048
            # LKAS_EFFECTIVE: 39|12@0+ (-2048) -- byte4 then byte5 high nibble.
            self.eps_effective = ((d[4] << 4) | (d[5] >> 4)) - 2048
            # STEER_ANGLE_RATE: 23|16@0+ (0.25, -8192)
            self.steer_angle_rate = (((d[2] << 8) | d[3]) * 0.25) - 8192.0
            self.lkas_block = bool(bit(d, 50))
            self.hands_off_5s = bool(bit(d, 51))
            self.lkas_track_state = bit(d, 52)
            self.eps_seen += 1
        elif addr == BLINK_INFO:
            # LEFT_BLINK 18|1@1+, RIGHT_BLINK 19|1@0+ -- both land in byte 2,
            # bits 2 and 3. Raw lamp state: upstream MazdaCarState debounces
            # these through update_blinker_from_lamp(40, ...) because the lamp
            # blinks, and a raw read is False for half of every blink cycle.
            # DesireHelper triggers on the RISING EDGE of (left != right), so
            # feeding it the raw flicker would re-arm a lane change ~1.5 times a
            # second. Debounced in dashcam_web before it reaches carState.
            # DEBOUNCED, not raw. The lamp physically blinks at ~1.5 Hz, so the
            # raw bit is False for half of every cycle. DesireHelper triggers on
            # the RISING edge of (left != right), so feeding it the raw flicker
            # would re-arm a lane change roughly twice a second for as long as
            # the stalk is held.
            #
            # Upstream does this with update_blinker_from_lamp(40, ...), a
            # 40-iteration counter at carState rate (0.4 s). Here update() is
            # driven by CAN arrival rather than a fixed tick, so hold by TIME
            # instead -- same 0.4 s, and it does not silently change meaning if
            # the frame rate of 0x09A differs from what upstream assumes.
            now = time.time()
            if bit(d, 18):
                self._left_lamp_t = now
            if bit(d, 19):
                self._right_lamp_t = now
            self.left_blinker = (now - self._left_lamp_t) < BLINKER_HOLD_S
            self.right_blinker = (now - self._right_lamp_t) < BLINKER_HOLD_S
        elif addr == BSM:
            # LEFT_BS_STATUS 13|2@0+, RIGHT_BS_STATUS 15|2@0+ -> byte 1, bits
            # 5..4 and 7..6. DBC VAL_: 0 none, 1 object in blindspot, 2 object
            # + blinker (warning). Anything non-zero is an object.
            self.left_blindspot = ((d[1] >> 4) & 0x03) != 0
            self.right_blindspot = ((d[1] >> 6) & 0x03) != 0
        elif addr == STEER:
            # STEER_ANGLE: big-endian 16-bit at bit 23 -> bytes 2,3
            raw = (d[2] << 8) | d[3]
            self.steering_angle = raw * 0.05 - 1600.0
        elif addr == ENGINE_DATA:
            self.rpm = ((d[0] << 8) | d[1]) // 4
            self.v_ego = (((d[2] << 8) | d[3]) / 100.0) / 3.6   # kph -> m/s
            self.gas_pressed = bool(((d[4] << 4) | (d[5] >> 4)) > 0)
        elif addr == PEDALS:
            # PEDALS was in the address list from the start but had no branch, so
            # brake_pressed sat at its False initialiser for entire drives -- and
            # it feeds carState.brakePressed, so openpilot never saw a brake press.
            #
            # The panda decodes this frame ITSELF (mazda_rx_hook) and drops
            # controls_allowed on the rising edge while moving. With our copy stuck
            # False, openpilot kept commanding full torque into a closed gate: every
            # non-zero frame is a tx violation, so 0x243 left the bus entirely until
            # controlsMismatch fired ~1.7 s later. That blackout is what raises the
            # front camera fault on the cluster.
            #
            # Bit taken to MATCH THE PANDA EXACTLY (opendbc/safety/modes/mazda.h:
            # `brake_pressed = msg->data[0] & 0x10U`). If the two disagree, the
            # same divergence comes back in a subtler form.
            self.brake_pressed = bool(d[0] & 0x10)
            # MRCC state, taken from PEDALS rather than CRZ_CTRL. Under alpha long
            # the radar is suppressed and WE send 0x21c, so reading cruise state
            # back off CRZ_CTRL would be reading our own output -- circular, and
            # the reason MADS and alpha long looked mutually exclusive. PEDALS is
            # the PCM's own report and stays truthful with the radar silent.
            #
            # Bits match the panda exactly (mazda.h: GET_BIT(msg, 3U) / 2U) and
            # the DBC (ACC_ACTIVE : 3|1@0+, ACC_OFF : 2|1@1+). GET_BIT and bit()
            # are the same formula, so these cannot drift apart.
            self.acc_active = bool(bit(d, 3))       # stock ACC actually engaged
            self.acc_armed = bool(bit(d, 2)) or self.acc_active   # MRCC MAIN on
        elif addr == WHEELS:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=int, default=0, help="0 = until Ctrl-C")
    ap.add_argument("--no-model", action="store_true", help="skip supercombo")
    args = ap.parse_args()

    CP = CarInterface.get_non_essential_params(CAR.MAZDA_CX5_2022)
    params = Params()
    params.put("CarParams", CP.to_bytes())
    params.put_bool("OpenpilotEnabledToggle", True)
    params.put_bool("DisengageOnAccelerator", True)
    print(f"CarParams: {CP.carFingerprint}")

    # ---- panda: assert SILENT before anything else touches the bus ----
    # Same by-id resolution as dashcam_web: ttyACM<n> follows probe order, so any
    # other CDC-ACM device can steal ttyACM0 and this opens the wrong hardware.
    panda = SerialPanda(os.environ.get("OP_PANDA_PORT") or next(
        (os.path.realpath(p) for p in sorted(glob.glob("/dev/serial/by-id/*STM32_STLink*"))),
        "/dev/ttyACM0"))
    panda.control_write(0xdc, SAFETY_SILENT, 0)
    time.sleep(0.3)
    print("panda: SAFETY_SILENT asserted (read-only for the whole run)")

    # ---- camera + model ----
    cam = runner = None
    if not args.no_model:
        from op_camera_ae import CameraAE
        # Argus's own AE runs unlocked across the full 34us-33ms / gain 1-16
        # envelope, and a slow outer loop re-aims it at the ROAD band via
        # exposurecompensation -- Argus meters the whole frame, so a bright sky
        # drags the road down (measured sky p50 184 / road p50 69 in one frame).
        # Not auto_exposure=False: that set aelock=TRUE and pinned the sensor at
        # the night end of the range, 99.2% of the frame clipped white in daylight.
        cam = CameraAE(auto_exposure=True)
        print("camera: persistent capture up")
        from op_stream import SupercomboRunner
        runner = SupercomboRunner()
        print("supercombo: TRT engine loaded")

    # ---- real openpilot daemons ----
    env = dict(os.environ)
    env["PARAMS_ROOT"] = os.environ["PARAMS_ROOT"]
    env["PYTHONPATH"] = ("/home/tran/msgq_build:/home/tran/opendbc_src:"
                         "/home/tran/op_fork:/home/tran/op_fork/openpilot")
    # See dashcam_web.py: no camerad/sensord/locationd on this board, so selfdrived's
    # liveness checks for them would raise four NO_ENTRY events and block engagement.
    env["JETSON_CAMERA_BYPASS"] = "1"
    procs = []
    for mod in ("openpilot.selfdrive.selfdrived.selfdrived",
                "openpilot.selfdrive.controls.controlsd"):
        p = subprocess.Popen(["python3", "-m", mod], cwd="/home/tran/op_fork/openpilot",
                             env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p)
        print(f"launched {mod.split('.')[-1]} (pid {p.pid})")
    time.sleep(4.0)

    pub = ['deviceState', 'pandaStates', 'peripheralState', 'modelV2', 'liveCalibration',
           'carOutput', 'driverMonitoringState', 'longitudinalPlan', 'livePose', 'liveDelay',
           'managerState', 'liveParameters', 'radarState', 'liveTorqueParameters',
           'driverAssistance', 'alertDebug', 'lateralManeuverPlan', 'carState']
    # only pandaStates is list-typed in this schema; managerState/liveTracks are
    # structs here and init(size) on them raises.
    LIST_TOPICS = {'pandaStates'}
    avail = []
    for t in pub:
        try:
            messaging.new_message(t, 0) if t in LIST_TOPICS else messaging.new_message(t)
            avail.append(t)
        except Exception:
            pass
    pm = messaging.PubMaster(avail)
    sm = messaging.SubMaster(['selfdriveState', 'carControl', 'onroadEvents'])
    time.sleep(0.5)

    cs_can = CarStateFromCAN()
    logf = open(LOG_PATH, "w")
    print(f"logging -> {LOG_PATH}\n")
    print(f"  {'t':>6} {'v':>5} {'rpm':>5} {'avail':>5} {'CRZ':>4} {'brk':>4} {'str':>4} "
          f"{'curv':>8} {'torque':>7} {'lat':>5} {'enabled':>7} {'state':>10}")
    print("  " + "-" * 100)

    frame = 0
    t0 = time.time()
    last_print = 0.0
    stop = {"v": False}

    def onsig(*_):
        stop["v"] = True
    signal.signal(signal.SIGINT, onsig)

    out = {"path_xyz": None}
    curv = 0.0
    act = None      # last model action; stays None until the first step with a v_ego
    try:
        while not stop["v"]:
            if args.seconds and (time.time() - t0) > args.seconds:
                break

            # 1) drain real CAN into carState
            for addr, d, bus in panda.can_recv():
                if bus == 0:
                    cs_can.update(addr, d)

            # 2) camera -> model -> curvature
            if runner is not None:
                f = cam.read()
                if f is not None:
                    # v_ego is REQUIRED: op_stream.step() only feeds the online
                    # calibrator when v_ego is not None, and the calibrator itself
                    # additionally gates on >15km/h and near-zero yaw rate. Calling
                    # step(f) without it means handle() never runs and rpy stays
                    # (0,0,0) forever no matter how far you drive.
                    out = runner.step(f, v_ego=cs_can.v_ego)
                    # STOCK: modeld.get_action_from_model, not a quadratic fit to the
                    # path over ~2.5 s. step() has already run it -- it is populated
                    # whenever v_ego is passed, which it is above. On the current
                    # weights ("Rebellious Hope") this is the model's own action head;
                    # on the older ones it is reconstructed from the plan's yaw.
                    act = out.get("action")
                    if act is not None:
                        curv = act["desiredCurvature"]

            # 3) publish everything the daemons need
            for name in avail:
                m = messaging.new_message(name, 0) if name in LIST_TOPICS else messaging.new_message(name)
                if name == 'carState':
                    cs = m.carState
                    cs.vEgo = cs_can.v_ego; cs.vEgoRaw = cs_can.v_ego
                    cs.steeringAngleDeg = 0.0
                    cs.steeringTorque = cs_can.steering_torque
                    cs.steeringPressed = cs_can.steering_pressed
                    cs.brakePressed = cs_can.brake_pressed
                    cs.gasPressed = cs_can.gas_pressed
                    cs.standstill = cs_can.v_ego < 0.3
                    cs.gearShifter = car_struct.CarState.GearShifter.drive
                    cs.canValid = cs_can.seen > 0
                    cs.cruiseState.available = cs_can.cruise_available
                    cs.cruiseState.enabled = cs_can.cruise_enabled
                    cs.cruiseState.speed = 25.0
                elif name == 'deviceState':
                    m.deviceState.started = True
                    m.deviceState.freeSpacePercent = 80.0
                elif name == 'liveParameters':
                    lp = m.liveParameters
                    lp.steerRatio = 15.5; lp.stiffnessFactor = 1.0
                    lp.angleOffsetDeg = 0.0; lp.roll = 0.0; lp.valid = True
                elif name == 'liveTorqueParameters':
                    tp = m.liveTorqueParameters
                    # MAZDA_CX9_2021 tune (CX5_2022 substitutes to it). Was 2.5 / 0.1.
                    tp.useParams = True; tp.latAccelFactorFiltered = 1.7601682915983443
                    tp.latAccelOffsetFiltered = 0.0; tp.frictionCoefficientFiltered = 0.17713792194297195
                elif name == 'liveCalibration':
                    lc = m.liveCalibration
                    rpy = list(getattr(runner, "calib_euler", (0.0, 0.0, 0.0))) if runner else [0.0]*3
                    lc.rpyCalib = [float(x) for x in rpy]
                    lc.calStatus = log.LiveCalibrationData.Status.calibrated
                elif name == 'modelV2':
                    md = m.modelV2
                    md.frameId = frame
                    p_xyz = out.get("path_xyz")
                    if p_xyz is not None:
                        md.position.x = [float(p_xyz[i][0]) for i in range(33)]
                        md.position.y = [float(p_xyz[i][1]) for i in range(33)]
                        md.position.z = [float(p_xyz[i][2]) for i in range(33)]
                        md.position.t = [float(t) for t in T_IDXS]
                    try:
                        md.action.desiredCurvature = float(curv)
                        # All three fields stock modeld publishes, not just the one
                        # this port steers on. Nothing here actuates longitudinally,
                        # so these are for the log -- but they are now predictions
                        # rather than defaults.
                        if act is not None:
                            md.action.desiredAcceleration = float(act["desiredAcceleration"])
                            md.action.shouldStop = bool(act["shouldStop"])
                    except Exception: pass
                pm.send(name, m)
            frame += 1

            # 4) read back what controlsd decided -- recorded, never sent
            sm.update(0)
            ss, cc = sm['selfdriveState'], sm['carControl']
            el = time.time() - t0
            rec = dict(t=round(el, 2), frame=frame,
                       v_ego=round(cs_can.v_ego, 2), rpm=cs_can.rpm,
                       cruise_available=cs_can.cruise_available,
                       cruise_enabled=cs_can.cruise_enabled,
                       brake=cs_can.brake_pressed, gas=cs_can.gas_pressed,
                       steer_torque=cs_can.steering_torque,
                       steer_pressed=cs_can.steering_pressed,
                       buttons=cs_can.buttons, curvature=round(float(curv), 6),
                       op_enabled=bool(ss.enabled), op_active=bool(ss.active),
                       state=str(ss.state), lat_active=bool(cc.latActive),
                       would_command_torque=round(float(cc.actuators.torque), 4),
                       calib=[float(x) for x in (getattr(runner, "calib_euler", (0,0,0)) if runner else (0,0,0))],
                       events=[str(e.name) for e in sm['onroadEvents']][:6])
            logf.write(json.dumps(rec) + "\n")

            if el - last_print > 0.5:
                last_print = el
                logf.flush()
                print(f"  {el:6.1f} {cs_can.v_ego:5.1f} {cs_can.rpm:5d} "
                      f"{int(cs_can.cruise_available):5d} {int(cs_can.cruise_enabled):4d} "
                      f"{int(cs_can.brake_pressed):4d} {int(cs_can.steering_pressed):4d} "
                      f"{curv:8.5f} {cc.actuators.torque:7.3f} {int(cc.latActive):5d} "
                      f"{int(ss.enabled):7d} {str(ss.state):>10}")
    finally:
        print("\nshutting down...")
        for p in procs:
            p.send_signal(signal.SIGINT)
        time.sleep(0.5)
        for p in procs:
            p.terminate()
            try: p.wait(timeout=5)
            except Exception: p.kill()
        if cam is not None:
            cam.close()
        logf.close()
        # re-assert SILENT: the panda was never taken out of it, but make the
        # end state explicit rather than assumed.
        try:
            panda.control_write(0xdc, SAFETY_SILENT, 0)
            print("panda: SAFETY_SILENT re-asserted")
        except Exception as e:
            print(f"panda: could not re-assert SILENT ({e}) -- power-cycle before driving again")

    print(f"\nlog: {LOG_PATH}  ({frame} frames)")
    print("NOTHING WAS TRANSMITTED. 'would_command_torque' is what openpilot")
    print("would have sent to the EPS -- replay it before enabling actuation.")


if __name__ == "__main__":
    main()
