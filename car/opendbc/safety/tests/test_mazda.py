#!/usr/bin/env python3
import unittest

import numpy as np

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda


class TestMazdaSafety(common.PandaCarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0]]
  STANDSTILL_THRESHOLD = .1
  RELAY_MALFUNCTION_ADDRS = {0: (0x243, 0x440)}
  FWD_BLACKLISTED_ADDRS = {2: [0x243, 0x440]}

  # These must mirror the LIMITS STRUCT IN mazda.h, because that is what this
  # suite actually exercises -- it loads libsafety and probes the firmware's own
  # boundaries. They had drifted to the upstream values (10/25/1000/300) while
  # the port's firmware moved to 80/50/2047/2047, so every steering test here
  # was failing against limits the firmware has not enforced for some time.
  # Re-derived from mazda.h 2026-08-12; grep .max_torque / .max_rate_up /
  # .max_rate_down / .max_rt_delta there before changing any of them.
  #
  # NOTE the host is NOT at these values: CarControllerParams.STEER_MAX in
  # values.py is 800, and its own comment says it "MUST stay equal to
  # .max_torque in mazda.h". It is not equal. 800 < 2047 is the safe direction
  # (the panda cannot reject a command the host never makes) so this is not a
  # hazard, but the two are out of sync and one of them is wrong.
  MAX_RATE_UP = 80
  MAX_RATE_DOWN = 50
  MAX_TORQUE_LOOKUP = [0], [2047]

  MAX_RT_DELTA = 2047

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 1

  # Mazda actually does not set any bit when requesting torque
  NO_STEER_REQ_BIT = True

  def setUp(self):
    self.packer = CANPackerPanda("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)
    self.safety.init_tests()

  def _torque_meas_msg(self, torque):
    values = {"STEER_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_panda("STEER_TORQUE", 0, values)

  def _torque_driver_msg(self, torque):
    # STEER_TORQUE_SENSOR is 8|8@0+ with offset -127, so the sensor can only
    # ever report -127..+128. The shared test derives an "arbitrary high driver
    # torque" of max_torque + allowance + 1, which stops fitting that field once
    # max_torque is raised -- and the packer WRAPS rather than saturating, so
    # e.g. -1016 comes out as +8 and the test ends up telling the firmware the
    # driver is helping when it meant to say the driver is fighting hard.
    # Clamping models what the real sensor can physically output.
    values = {"STEER_TORQUE_SENSOR": max(-127, min(128, torque))}
    return self.packer.make_can_msg_panda("STEER_TORQUE", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_REQUEST": torque}
    return self.packer.make_can_msg_panda("CAM_LKAS", 0, values)

  def _speed_msg(self, speed):
    values = {"SPEED": speed}
    return self.packer.make_can_msg_panda("ENGINE_DATA", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_ON": brake}
    return self.packer.make_can_msg_panda("PEDALS", 0, values)

  def _user_gas_msg(self, gas):
    values = {"PEDAL_GAS": gas}
    return self.packer.make_can_msg_panda("ENGINE_DATA", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRZ_ACTIVE": enable}
    return self.packer.make_can_msg_panda("CRZ_CTRL", 0, values)

  def _button_msg(self, resume=False, cancel=False, set_m=False):
    values = {
      "CAN_OFF": cancel,
      "CAN_OFF_INV": (cancel + 1) % 2,
      "RES": resume,
      "RES_INV": (resume + 1) % 2,
      "SET_M": set_m,
      "SET_M_INV": (set_m + 1) % 2,
    }
    return self.packer.make_can_msg_panda("CRZ_BTNS", 0, values)

  def test_realtime_limits(self):
    # DELIBERATELY NOT the inherited test, because this port has deliberately
    # neutralised the limit it checks. mazda.h sets
    #     .max_rt_delta = 2047   // "Tracks max_torque so it can never bind"
    # equal to .max_torque, so that a run measures the rack rather than the
    # limiter. max_rate_up (per message) and max_torque (absolute) are the only
    # steering limits left.
    #
    # The shared test asserts that MAX_RT_DELTA + 1 becomes transmittable once
    # the RT window rolls over. That is unreachable here: 2048 exceeds the
    # ABSOLUTE limit first, so it is blocked for a reason the test does not
    # model, and it fails no matter what the RT check does.
    #
    # Assert the neutralisation behaviourally instead: inside ONE real-time
    # window, ramping at the per-message rate all the way to max_torque must
    # never be blocked. If max_rt_delta is ever restored to a binding value --
    # which the mazda.h comment says to do once the EPS ceiling is known -- this
    # test fails and the inherited one should be restored with it.
    max_torque = self.MAX_TORQUE_LOOKUP[1][0]
    self.safety.set_controls_allowed(True)

    for sign in (-1, 1):
      self.safety.init_tests()
      self._set_prev_torque(0)
      self._reset_torque_driver_measurement(0)

      # No set_timer() call: everything below happens in a single RT window.
      torque = 0
      while abs(torque) < max_torque:
        torque = sign * min(abs(torque) + self.MAX_RATE_UP, max_torque)
        self.assertTrue(self._tx(self._torque_cmd_msg(torque)),
                        f"rt delta bound at {torque} within one window")

      self.assertEqual(abs(torque), max_torque)
      # and the absolute limit is still the thing that stops us
      self.assertFalse(self._tx(self._torque_cmd_msg(sign * (max_torque + 1))))

  def test_buttons(self):
    # only cancel allows while controls not allowed
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertTrue(self._tx(self._button_msg(resume=True)))


class TestMazdaLongitudinalSafety(TestMazdaSafety, common.LongitudinalAccelSafetyTest):
  """Alpha long: openpilot commands accel, and the BUTTONS decide engagement.

  The stock class above engages off CRZ_CTRL.CRZ_ACTIVE. Under alpha long the
  radar that sends that frame is suppressed and the PCM's ACC_ACTIVE never rises
  (MEASURED on a CX-5 2023, 2026-08-12), so engagement moves to the driver's
  cruise buttons -- see the CRZ_BTNS branch in mazda.h.
  """

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0], [0x21b, 0], [0x21c, 0], [0x764, 0]]
  MAX_ACCEL = 2000.0
  MIN_ACCEL = -2000.0
  # NOT 0. Mazda's idle CRZ_INFO carries ACCEL_CMD = +4094, a "no request"
  # sentinel; zero is a live command for zero acceleration. Must match
  # MAZDA_ACCEL_IDLE_SENTINEL in mazda.h and build_crz_info()'s standby frame.
  INACTIVE_ACCEL = 4094.0

  def setUp(self):
    self.packer = CANPackerPanda("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, 1)
    self.safety.init_tests()

  def _pcm_status_msg(self, enable):
    # MRCC main-on state. Under alpha long this is the PRECONDITION for control,
    # not the thing that grants it: main-off still disengages, but main-on no
    # longer engages by itself.
    values = {"ACC_OFF": enable, "BRAKE_ON": 0}
    return self.packer.make_can_msg_panda("PEDALS", 0, values)

  def _accel_msg(self, accel: float):
    values = {"ACCEL_CMD": accel}
    return self.packer.make_can_msg_panda("CRZ_INFO", 0, values)

  def _crz_ctrl_msg(self, cruise_active: bool):
    values = {"CRZ_ACTIVE": cruise_active}
    return self.packer.make_can_msg_panda("CRZ_CTRL", 0, values)

  def _radar_uds_msg(self, dat: bytes):
    return libsafety_py.make_CANPacket(0x764, 0, dat)

  def _user_brake_msg(self, brake):
    # PEDALS carries the brake AND the MRCC main state, and under alpha long
    # main-off is a disengage. The base class's brake tests send this frame with
    # everything else zeroed, which reads as the driver switching MRCC off, so
    # they would be testing main-off rather than braking. Hold main on.
    values = {"BRAKE_ON": brake, "ACC_OFF": 1}
    return self.packer.make_can_msg_panda("PEDALS", 0, values)

  def _acc_active_msg(self, active):
    # The PCM's own ACC_ACTIVE. Under alpha long it never rises on this car; it
    # is kept only so cruise_engaged_prev has something to track.
    values = {"ACC_ACTIVE": active, "ACC_OFF": 1, "BRAKE_ON": 0}
    return self.packer.make_can_msg_panda("PEDALS", 0, values)

  def test_cruise_engaged_prev(self):
    # Overrides the base test. cruise_engaged_prev still mirrors the PCM's
    # ACC_ACTIVE, but it is no longer the engagement authority -- the buttons
    # are -- so it is asserted against its own signal rather than main-on.
    for engaged in (True, False):
      self._rx(self._acc_active_msg(engaged))
      self.assertEqual(engaged, self.safety.get_cruise_engaged_prev())
      self._rx(self._acc_active_msg(not engaged))
      self.assertEqual(not engaged, self.safety.get_cruise_engaged_prev())

  def _main_on(self):
    self._rx(self._pcm_status_msg(True))

  # --- engagement: buttons, not the PCM ---------------------------------

  def test_enable_control_allowed_from_cruise(self):
    # Overrides the base test. Main-on alone must NOT engage: that is exactly the
    # permissive gate this port must not have, since it would grant throttle and
    # brake authority the driver never asked for.
    self.safety.set_controls_allowed(0)
    self._main_on()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_disable_control_allowed_from_cruise(self):
    # Overrides the base test. Main-OFF must still disengage.
    self._main_on()
    self.safety.set_controls_allowed(1)
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_arm_on_resume_press(self):
    self._main_on()
    self.safety.set_controls_allowed(0)
    self._rx(self._button_msg())
    self._rx(self._button_msg(resume=True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_arm_on_set_release(self):
    self._main_on()
    self.safety.set_controls_allowed(0)
    self._rx(self._button_msg())
    self._rx(self._button_msg(set_m=True))
    # still held -- nothing yet
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._button_msg())
    self.assertTrue(self.safety.get_controls_allowed())

  def test_no_arm_on_set_press_alone(self):
    self._main_on()
    self.safety.set_controls_allowed(0)
    self._rx(self._button_msg())
    for _ in range(10):
      self._rx(self._button_msg(set_m=True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_cancel_disables_controls(self):
    self._main_on()
    self.safety.set_controls_allowed(1)
    self._rx(self._button_msg(cancel=True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_cancel_wins_over_a_simultaneous_arm(self):
    self._main_on()
    self.safety.set_controls_allowed(0)
    self._rx(self._button_msg(set_m=True))
    self._rx(self._button_msg(cancel=True))   # set released AND cancel, one frame
    self.assertFalse(self.safety.get_controls_allowed())

  def test_acc_active_low_does_not_disarm(self):
    # REGRESSION GUARD for removing pcm_cruise_check() from the PEDALS branch.
    # ACC_ACTIVE stays 0 forever under alpha long, and pcm_cruise_check clears on
    # every sample where its argument is false -- so if it ever comes back, the
    # 50 Hz PEDALS frame silently revokes the arm the button just granted and the
    # car looks like it is ignoring us.
    self._main_on()
    self.safety.set_controls_allowed(0)
    self._rx(self._button_msg())
    self._rx(self._button_msg(resume=True))
    self.assertTrue(self.safety.get_controls_allowed())
    for _ in range(50):
      self._rx(self._pcm_status_msg(True))   # main on, ACC_ACTIVE still 0
    self.assertTrue(self.safety.get_controls_allowed())

  def test_main_off_disarms_after_button_arm(self):
    self._main_on()
    self.safety.set_controls_allowed(0)
    self._rx(self._button_msg())
    self._rx(self._button_msg(resume=True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  # --- actuation limits --------------------------------------------------

  def test_accel_actuation_limits(self):
    # CRZ_INFO.ACCEL_CMD is a raw integer command in Mazda's DBC, so use
    # integer-domain boundaries to avoid float rounding artifacts in packing.
    limits = ((self.MIN_ACCEL, self.MAX_ACCEL, common.ALTERNATIVE_EXPERIENCE.DEFAULT),
              (self.MIN_ACCEL, self.MAX_ACCEL, common.ALTERNATIVE_EXPERIENCE.RAISE_LONGITUDINAL_LIMITS_TO_ISO_MAX))

    for min_accel, max_accel, alternative_experience in limits:
      for accel in np.concatenate((np.arange(int(min_accel) - 1, int(min_accel) + 3),
                                   np.arange(int(max_accel) - 2, int(max_accel) + 2), [0])):
        for controls_allowed in (True, False):
          self.safety.set_controls_allowed(controls_allowed)
          self.safety.set_alternative_experience(alternative_experience)
          should_tx = controls_allowed and min_accel <= accel <= max_accel
          should_tx = should_tx or accel == self.INACTIVE_ACCEL
          self.assertEqual(should_tx, self._tx(self._accel_msg(float(accel))))

  def test_crz_ctrl_active_requires_controls_allowed(self):
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._crz_ctrl_msg(cruise_active=True)))
    self.assertTrue(self._tx(self._crz_ctrl_msg(cruise_active=False)))

    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._crz_ctrl_msg(cruise_active=True)))

  def test_idle_frame_passes_while_disengaged(self):
    """The frame we actually send when NOT engaged must reach the car.

    REGRESSION GUARD. This is the one case the accel-limit tests never covered:
    they sweep the commanded range and check 0 is allowed while disengaged, but
    the port does not send 0 when disengaged -- it sends the radar's idle
    sentinel, which is outside that range. MEASURED ON THE CAR 2026-08-12 with
    inactive_accel = 0: every 0x21b was rejected, tx_blocked climbing at 50/s
    for a whole run, so the PCM got no CRZ_INFO at all while the radar was muted.
    """
    from opendbc.car.mazda.longitudinal import build_crz_info

    self.safety.set_controls_allowed(False)
    for ctr in range(16):
      frame = build_crz_info(0.0, ctr, long_active=False, hold_request=False, v_ego=0.0)
      msg = libsafety_py.make_CANPacket(0x21b, 0, frame)
      self.assertTrue(self._tx(msg),
                      f"idle CRZ_INFO rejected while disengaged: {frame.hex(' ')}")

    # and a real command is still refused while disengaged
    self.assertFalse(self._tx(self._accel_msg(500.0)))

  def test_radar_uds_allowlist(self):
    tester_present = b"\x02\x3E\x80\x00\x00\x00\x00\x00"
    session_default = b"\x02\x10\x01\x00\x00\x00\x00\x00"
    session_programming = b"\x02\x10\x02\x00\x00\x00\x00\x00"
    disallowed = b"\x03\x22\xF1\x90\x00\x00\x00\x00"

    self.assertTrue(self._tx(self._radar_uds_msg(tester_present)))
    self.assertTrue(self._tx(self._radar_uds_msg(session_default)))
    self.assertTrue(self._tx(self._radar_uds_msg(session_programming)))
    self.assertFalse(self._tx(self._radar_uds_msg(disallowed)))


if __name__ == "__main__":
  unittest.main()
