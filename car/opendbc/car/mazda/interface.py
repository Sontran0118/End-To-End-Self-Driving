#!/usr/bin/env python3
import os
from opendbc.car import get_safety_config, structs
# NOT `from opendbc.car import carlog` -- that binds the MODULE
# opendbc.car.carlog, which has no .error, so the fallback path below raised
# AttributeError instead of falling back. Same form longitudinal.py uses.
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.longitudinal import enter_radar_programming_session
# BASELINE TEST 2026-08-11: longitudinal.py is temporarily the PRISTINE vendored
# version (yummydirtx @ mazda-longitudinal-upstream, the one with on-car results
# and stop-and-go solved). It has no enter_radar_standby -- that, the 4-rung
# ladder, the radar-derived frames and the ignition timing are all local additions
# made across five commits, none of which was ever tested against the original as a
# baseline. Since acc_active has never once reached 1 on this car, the possibility
# that our own changes broke a working implementation has to be ruled out first.
try:
  from opendbc.car.mazda.longitudinal import enter_radar_standby
except ImportError:
  enter_radar_standby = None
from opendbc.car.mazda.values import CAR, LKAS_LIMITS

# Must equal MAZDA_PARAM_LONGITUDINAL in opendbc/safety/modes/mazda.h. This is the
# single bit that swaps the panda between the stock tx table and the one that also
# permits 0x21b / 0x21c / 0x764.
MAZDA_LONG_SAFETY_PARAM = 1


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mazda"

    # ALPHA LONGITUDINAL. Off unless the caller asks for it AND the platform is
    # one it has been driven on. CX-5 2022-25 covers the 2023 this port runs on.
    #
    # Turning this on suppresses the radar, which is the same ECU that runs FCW,
    # AEB and SBS -- the car has none of them while it is enabled, and the dash
    # will say so. See opendbc_patches/alpha_long/README.md.
    ret.alphaLongitudinalAvailable = candidate == CAR.MAZDA_CX5_2022
    ret.openpilotLongitudinalControl = alpha_long and ret.alphaLongitudinalAvailable
    # WHO DECIDES ENGAGEMENT.
    #
    # Stock (no alpha long): the PCM does, off the real MRCC state. True.
    #
    # Alpha long: WE do. Upstream keeps pcmCruise = True here on the theory that
    # "Mazda-long still engages on the stock ACC-active transition", and on this
    # CX-5 2023 that transition never comes -- it is a deadlock. openpilot sets
    # longActive only once the car reports engaged; build_crz_info asserts
    # ACC_ACTIVE only once longActive; and the PCM engages only when told to, by
    # the radar alpha long just silenced. Nobody moves first.
    #
    # MEASURED 2026-08-12, parked, both suppression routes: 14 clean SET presses
    # on 0x09d, setspd frozen at raw=100, acc_active 0 for 100 s, 9007
    # longitudinal frames out with err=0 and tx_blocked=0. The frames are right
    # and the car is listening; there is no first mover.
    #
    # False makes openpilot the first mover, engaging on the button edge through
    # CarState.update_button_enable() -- which this port already implements and
    # which has been dead code precisely because this flag was True.
    ret.pcmCruise = not ret.openpilotLongitudinalControl
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda,
                                           MAZDA_LONG_SAFETY_PARAM if ret.openpilotLongitudinalControl else None)]
    ret.radarUnavailable = True

    ret.dashcamOnly = candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021)

    # 0.1 -> 0.2, MEASURED on this CX-5 2026-07-30 over a 10.3k-record drive
    # above 20 kph. 0x241 STEER_RATE carries LKAS_REQUEST and LKAS_EFFECTIVE in
    # the SAME frame -- the EPS's own account of "what I was told" against "what
    # I applied" at one instant -- so cross-correlating them measures the rack's
    # internal lag with no contribution from our CAN pipeline or model rate:
    #
    #   lag (50 ms records):  0:+0.75  1:+0.81  2:+0.86  3:+0.88  4:+0.89  5:+0.87
    #   peak r=0.887 at lag 4 = 200 ms
    #
    # LatControlTorque uses this to pick WHICH past setpoint to compare today's
    # measurement against (lat_accel_request_buffer[-delay_frames]). Told 100 ms
    # when the rack takes 200, it charges the missing 100 ms of actuator lag to
    # tracking error, over-commands, then reverses when the response finally
    # lands -- a delay-driven limit cycle. Measured on the same drive:
    # applied_torque crossed zero 1.84 times/s while the curvature command it
    # follows crossed only 0.87 times/s, i.e. the oscillation is generated
    # INSIDE the torque loop, not inherited from the plan.
    #
    # This is the rack's lag alone; the wheel-to-lateral-accel response adds
    # more on top, so 0.2 is a floor rather than a fitted optimum. Re-measure
    # with the same cross-correlation before moving it again.
    #
    # NB: op_stream.LAT_ACTION_T must carry the same number -- it replaces modeld
    # on this port, so the MODEL's action_t input comes from there, not from here.
    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    # LATERAL JITTER. MEASURED 2026-08-08 from a 250 Hz trace while engaged:
    #   curvature   (what the planner asks)  zero-crossings 0.10 Hz  -- smooth
    #   torque_norm (controller output)      zero-crossings 1.57 Hz  -- ringing
    #   applied     (counts on the wire)     zero-crossings 1.33 Hz, stdev 219
    # The command is smooth and the output is not, so the oscillation is
    # generated INSIDE the torque loop -- it is not model noise, camera noise or
    # rate limiting (delta_up/down were already cut to 10/25 and the rail is
    # never reached).
    #
    # configure_torque_tune gives every car kp=1.0 ki=0.3, and this platform's
    # LAT_ACCEL_FACTOR/FRICTION are not even its own -- substitute.toml maps
    # MAZDA_CX5_2022 -> MAZDA_CX9_2021, a 4217 lb car standing in for a 3655 lb
    # one. Fleet defaults on borrowed parameters, through ~0.3 s of actuator
    # delay, is a recipe for exactly this limit cycle.
    #
    # In LatControlTorque the FEEDFORWARD (kf * requested lat accel) does the
    # work in a curve; kp/ki only correct the residual error -- and the error
    # path is what rings. So cutting feedback damps the jitter while leaving
    # curve response almost untouched, which is the trade we want: smooth on a
    # straight, still able to hold a sharp bend.
    #
    # Env-tunable so this can be swept from the launch line without a rebuild.
    ret.lateralTuning.torque.kp = float(os.environ.get("OP_LAT_KP", "0.5"))
    ret.lateralTuning.torque.ki = float(os.environ.get("OP_LAT_KI", "0.10"))
    ret.lateralTuning.torque.latAccelFactor = float(
        os.environ.get("OP_LAT_ACCEL_FACTOR", ret.lateralTuning.torque.latAccelFactor))
    ret.lateralTuning.torque.friction = float(
        os.environ.get("OP_LAT_FRICTION", ret.lateralTuning.torque.friction))

    if candidate not in (CAR.MAZDA_CX5_2022,):
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

    if ret.openpilotLongitudinalControl:
      ret.startingState = True
      ret.startAccel = 1.2
      ret.vEgoStarting = 0.15
      ret.vEgoStopping = 0.5
      # Unlike steerActuatorDelay above, this one is NOT measured on this car --
      # it is the value the community port was tuned with. It feeds
      # op_stream.LONG_ACTION_T the same way the lateral delay does; re-measure
      # it the same way (command-in vs response-out on one frame) before trusting
      # the stop-and-go timing.
      ret.longitudinalActuatorDelay = 0.36
      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [1.2, 1.0, 0.8]
      ret.longitudinalTuning.kiBP = [0., 5., 20.]
      ret.longitudinalTuning.kiV = [0.18, 0.12, 0.08]

    ret.centerToFront = ret.wheelbase * 0.41

    return ret

  @staticmethod
  def init(CP, can_recv, can_send):
    # Called once at startup, before controls run. This is where the radar is
    # actually silenced: a UDS programming session on 0x764. It only STAYS
    # silenced because CarController re-sends tester-present every 50 frames --
    # stop that and the radar times out back to stock on its own, which is the
    # intended failure direction.
    # HOW the radar is silenced, selectable, standby by default.
    #
    # "programming" is the original: DiagnosticSessionControl -> programmingSession.
    # It works, but it is the heaviest tool ISO 14229 offers, and to every other
    # module on the bus an ECU in that state is one BEING REFLASHED -- it stops
    # answering normal requests, so the camera, cluster and PCM each log their own
    # communication DTC against it and the dash reports malfunctions.
    #
    # "standby" is 0x28 CommunicationControl with ENABLE_RX_DISABLE_TX, preceded by
    # an extended (not programming) session and 0x85 ControlDTCSetting OFF. The
    # radar keeps listening, keeps running and keeps answering diagnostics -- it
    # just stops putting its normal messages on the wire. Present and healthy from
    # the network's point of view, which is the difference that WAS EXPECTED to
    # keep the fault off the cluster.
    #
    # IT DOES NOT. MEASURED ON THE CAR 2026-08-12, both routes back to back with
    # the census running: the cluster raises the forward-sensing faults (FSM /
    # FSBM) under the programming session TOO. There is no dash-quietness
    # difference between the two, which is the entire reason standby was written.
    #
    # That is what you would expect from the fault taxonomy: these warnings are
    # cause 1 -- the radar IS FCW/AEB/SBS, so silencing it removes them and the
    # cluster says so, whichever UDS route did the silencing. Nothing about how
    # an ECU is muted changes the fact that it is muted.
    #
    # Neither route engages either (see FINDINGS-2026-08-11.md), so there is now
    # no measured reason to prefer standby over the simpler programming session
    # that the upstream community port uses. Keep both until something else
    # separates them, but do not repeat the claim that standby is quieter.
    #
    # Either way FCW/AEB/SBS are gone: the radar cannot command them if it cannot
    # transmit. That is inherent to taking over its messages, not a property of how
    # it was silenced. What may change is whether the car COMPLAINS about it.
    #
    # 0x28 is manufacturer-specific and the radar may simply NAK it, so standby
    # falls back to the programming session rather than leaving longitudinal with a
    # radar that is still driving 0x21b.
    if CP.openpilotLongitudinalControl:
      # Default flipped to "programming" while the pristine module is in place.
      # That is what the version with on-car results actually used, and it is the
      # only route this module offers. Set OP_RADAR_SUPPRESS=standby to go back to
      # the local ladder once longitudinal.py is restored.
      mode = os.environ.get("OP_RADAR_SUPPRESS",
                            "programming" if enter_radar_standby is None else "standby")
      if mode == "programming" or enter_radar_standby is None:
        enter_radar_programming_session(can_recv, can_send)
      else:
        if not enter_radar_standby(can_recv, can_send,
                                   disable_dtc=(mode != "standby_keepdtc")):
          carlog.error("mazda radar standby refused, falling back to programming session")
          enter_radar_programming_session(can_recv, can_send)

  @staticmethod
  def deinit(CP, can_recv, can_send):
    if CP.openpilotLongitudinalControl:
      # Mazda's radar faults if we explicitly request the default/active session
      # on teardown. Exiting cleanly is just stopping tester present and letting
      # the radar time out back to stock behavior on its own.
      return
