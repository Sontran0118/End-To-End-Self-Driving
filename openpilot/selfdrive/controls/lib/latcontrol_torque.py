import math
import numpy as np
from collections import deque

from openpilot.cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects the
# proportional gain is increased at low speeds by the PID controller.
# Additionally, there is friction in the steering wheel that needs
# to be overcome to move it at all, this is compensated for too.

KP = 0.8
KI = 0.15

INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]

LP_FILTER_CUTOFF_HZ = 1.2
JERK_LOOKAHEAD_SECONDS = 0.19
# BACK TO 0.3 from 0.5 (2026-07-30, same day). This scales the desired-lateral-
# jerk term fed into get_friction(), which pre-loads torque to break the
# steering's static friction as a turn begins.
#
# The note that raised it warned "too high and it will pre-load on noise -- the
# LP_FILTER_CUTOFF_HZ below is what keeps that in check". That is what happened.
# The desired-curvature signal it differentiates crosses zero 0.87 times/s in
# normal driving, so the jerk term is not a turn-entry pulse, it is a
# continuously reversing lead term. At 0.5 it fed 67% more of that into an
# already phase-lagged loop (steerActuatorDelay was 100 ms against a measured
# 200 ms rack lag) and the result was a 1.8 Hz limit cycle -- torque crossing
# zero 1.84 times/s, twice the rate of the command it was following.
#
# Restored to stock while the delay fix is validated. It is a REAL lever for
# turn-in feel and worth revisiting -- but on a loop whose phase is right, and
# one change at a time.
JERK_GAIN = 0.3
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 1

# --- sunnypilot look-ahead jerk friction (ported) ----------------------------
# Ported from sunnypilot/selfdrive/controls/lib/latcontrol_torque_ext_base.py
# (MIT, Haibin Wen and contributors). Their comment states the problem exactly:
#
#   "Instantaneous lateral jerk changes very rapidly, making it not useful on
#    its own, however, we can look ahead to the future planned lateral jerk in
#    order to gauge whether the current desired lateral jerk will persist into
#    the future, i.e. whether it is deliberate or not. This allows us to simply
#    ignore short-lived jerk."
#
# WHY IT MATTERS HERE. This file derives jerk by DIFFERENTIATING its own request
# buffer: (buf[i+1] - buf[i-1]) / (2*dt) with dt = 0.01. A central difference at
# 100 Hz multiplies any noise on the setpoint by ~1/(2*dt) = 50, and that product
# is fed straight into get_friction(), which pre-loads torque. So setpoint noise
# becomes torque noise with a 50x gain, filtered only by a first-order lowpass.
#
# MEASURED on this car at highway speed: applied torque ranging -1834..+2029
# counts (essentially the full +-2047 scale) with a mean change of 562 counts per
# SECOND, while the driver was barely touching the wheel (median driver torque
# 1.0). The curvature command driving all of that had a zero-crossing rate of
# 0.09 Hz -- i.e. smooth. The oscillation is manufactured inside this loop.
#
# The fix is not a smaller gain, it is REJECTION: if the planned jerk changes
# sign anywhere inside the look-ahead window, the signal is a transient and is
# discarded outright rather than scaled down.
LAT_ACCEL_FRICTION_FACTOR = 0.7   # sunnypilot: scales the error term into friction
LAT_JERK_FRICTION_FACTOR = 0.4    # sunnypilot: scales the surviving jerk term
FRICTION_LOOK_AHEAD_BP = [9.0, 30.0]   # m/s
FRICTION_LOOK_AHEAD_V = [1.4, 2.0]     # seconds to look ahead at those speeds


def _sign(x):
  return 1.0 if x > 0.0 else (-1.0 if x < 0.0 else 0.0)


def get_lookahead_value(future_vals, current_val):
  """sunnypilot's transient filter, verbatim in behaviour.

  Returns 0 if ANY future value disagrees in sign with the current one -- the
  manoeuvre is not sustained, so do not pre-load torque for it. Otherwise return
  the smallest magnitude, which is the conservative choice.
  """
  if len(future_vals) == 0:
    return current_val
  same_sign_vals = [v for v in future_vals if _sign(v) == _sign(current_val)]
  if len(same_sign_vals) < len(future_vals):
    return 0.0
  return min(same_sign_vals + [current_val], key=lambda x: abs(x))


class LatControlTorque(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, rate=1/self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len , maxlen=self.lat_accel_request_buffer_len)
    self.lookahead_frames = int(JERK_LOOKAHEAD_SECONDS / self.dt)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)

  def reset(self):
    # controlsd calls this on every `not CC.latActive` frame precisely so the
    # controller does not carry state across a disengage. The base class only
    # clears sat_time, and this class used to inherit that unchanged -- so the
    # PI integrator survived. LatControlTorque's `not active` branch below also
    # returns early WITHOUT calling self.pid.update(), which freezes the
    # integral at whatever it held rather than decaying it.
    #
    # Net effect: the integral wound up during a turn was still there when the
    # driver re-engaged, and got dumped into the rack on the first active frame.
    # That is the "jitter when cruise is switched off and back on" case, and it
    # is worst exactly where it is least wanted -- mid-corner, where the integral
    # is largest.
    #
    # latcontrol_curvature.py, the sibling controller, already does this both in
    # reset() and in its own inactive branch. This is that omission, fixed.
    # ONLY the integrator. Deliberately NOT lat_accel_request_buffer: update()
    # appends to it before the active check, and while disengaged controlsd
    # holds desired_curvature AT the current curvature ("Reset desired curvature
    # to current to avoid violating the limits on engage"). Those entries are
    # therefore live and correct, and zeroing them would make the setpoint 0 for
    # the first delay_frames after engaging -- commanding the car to null out a
    # turn it is already in. The jerk filter needs no help either: it keeps
    # updating while inactive and decays on its own.
    super().reset()
    self.pid.reset()

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION
    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    measurement = measured_curvature * CS.vEgo ** 2
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
    self.lat_accel_request_buffer.append(future_desired_lateral_accel)

    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
    curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    delay_frames = int(np.clip(lat_delay / self.dt + 1, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
    setpoint = expected_lateral_accel
    error = setpoint - measurement

    lookahead_idx = int(np.clip(-delay_frames + self.lookahead_frames, -self.lat_accel_request_buffer_len+1, -2))
    raw_lateral_jerk = (self.lat_accel_request_buffer[lookahead_idx+1] - self.lat_accel_request_buffer[lookahead_idx-1]) / (2 * self.dt)
    desired_lateral_jerk = self.jerk_filter.update(raw_lateral_jerk)

    # Look-ahead transient rejection. Sample the jerk implied by the request
    # buffer across the window, and drop the whole term if it reverses inside
    # it. Sustained jerk (entering a real bend) survives; the 50x-amplified
    # differentiation noise that dominates a straight road does not.
    _look_s = float(np.interp(CS.vEgo, FRICTION_LOOK_AHEAD_BP, FRICTION_LOOK_AHEAD_V))
    _n = int(np.clip(_look_s / self.dt, 4, self.lat_accel_request_buffer_len - 2))
    _step = max(1, _n // 8)
    _future = []
    for _k in range(lookahead_idx, min(lookahead_idx + _n, -2), _step):
      _future.append((self.lat_accel_request_buffer[_k+1] - self.lat_accel_request_buffer[_k-1]) / (2 * self.dt))
    lookahead_lateral_jerk = get_lookahead_value(_future, desired_lateral_jerk)
    if lookahead_lateral_jerk == 0.0:
      desired_lateral_jerk = 0.0
      friction_error_factor = 1.0
    else:
      desired_lateral_jerk = lookahead_lateral_jerk
      friction_error_factor = LAT_ACCEL_FRICTION_FACTOR
    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
    ff = gravity_adjusted_future_lateral_accel
    # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
    ff -= self.torque_params.latAccelOffset
    # sunnypilot weights the two friction inputs separately rather than adding a
    # raw error to a raw jerk: the error is scaled to 0.7 when a sustained
    # manoeuvre is present, and left at 1.0 when the jerk term was rejected.
    friction_input = friction_error_factor * error + LAT_JERK_FRICTION_FACTOR * desired_lateral_jerk
    ff += get_friction(friction_input, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error, speed=CS.vEgo, feedforward=ff, freeze_integrator=freeze_integrator)
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque) # TODO: log lat accel?
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
