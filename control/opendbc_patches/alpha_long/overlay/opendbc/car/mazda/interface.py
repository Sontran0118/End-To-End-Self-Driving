#!/usr/bin/env python3
from opendbc.car import get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.longitudinal import enter_radar_programming_session
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
    # Mazda-long still engages on the stock ACC-active transition even though
    # we suppress the radar-owned CRZ_CTRL path and synthesize replacement
    # longitudinal messages.
    ret.pcmCruise = True
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
    if CP.openpilotLongitudinalControl:
      enter_radar_programming_session(can_recv, can_send)

  @staticmethod
  def deinit(CP, can_recv, can_send):
    if CP.openpilotLongitudinalControl:
      # Mazda's radar faults if we explicitly request the default/active session
      # on teardown. Exiting cleanly is just stopping tester present and letting
      # the radar time out back to stock behavior on its own.
      return
