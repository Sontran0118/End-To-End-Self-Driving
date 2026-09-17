import numpy as np
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.realtime import DT_CTRL, DT_MDL

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0
# This is a turn radius smaller than most cars can achieve
MAX_CURVATURE = 0.2
MAX_VEL_ERR = 5.0  # m/s
MIN_STABLE_DELAY = 0.3

# EU guidelines.
#
# The note that used to sit here said these were NOT binding because
# LatControlTorque saturates below 3.0 anyway. That was measured against a
# STEER_MAX of 800-1000 and it conflated the two constants. Re-measured
# 2026-07-30 over a 14.5k-record drive:
#
#   MAX_LATERAL_JERK   caps the curvature RATE at 5.0/v^2. Observed p95 decay
#                      rate 0.0415 1/m/s against a 0.0481 limit at the median
#                      10.2 m/s -- 86% utilised. THIS ONE BINDS. It is the
#                      reason turn-in and recovery both felt slow, and the
#                      torque rate limiter was only at 75% of its own ceiling
#                      at the same moments, so it was not the constraint.
#
#   MAX_LATERAL_ACCEL  caps curvature at 3.0/v^2. Observed 0.053 at 6.6 m/s
#                      against a 0.0698 limit -- 76%. Near, not binding. Its
#                      real cost is the floor it puts under turn radius:
#                      24 m at 19 mph, which is why ACC's ~19 mph minimum made
#                      tight turns impossible regardless of torque (see MADS).
#
# RESTORED to 5.0 (2026-07-30, same day it was doubled).
#
# The 86% figure above does not reproduce. Re-measured on the 2026-07-30 drive
# (10.7k lat_active records, 77 kph max) as |d(curvature)/dt| against the
# 10.0/v^2 limit, split by speed so a single median cannot hide a low-speed
# problem:
#
#     band kph    recs     p50    p90    p99   %at 10.0 / %at 5.0
#     10-20        400      5%    24%    74%     0.0%  /  3.8%
#     20-30        446      5%    19%    54%     0.0%  /  1.6%
#     30-45       2499      4%    15%    39%     0.1%  /  0.5%
#     45-60       4154      5%    16%    40%     0.1%  /  0.6%
#     60-90       3174      7%    24%    45%     0.0%  /  0.8%
#
# The limit is not close to binding anywhere -- typical use is 4-7% of it, and
# even the stock 5.0 would clip under 1% of samples outside the 10-20 kph band.
# So doubling it bought no turn-in and cost the only guard against the model
# commanding an abrupt curvature change.
#
# The earlier measurement compared a p95 decay RATE against the limit at the
# median speed, which mixes two different samples: the fastest curvature changes
# do not happen at the median speed, and the limit scales as 1/v^2. Comparing a
# p95 of one distribution to a limit computed from the median of another is what
# produced 86%.
#
# What actually made turn-in feel slow was loop phase, not this cap: the rack
# takes 200 ms to apply torque and steerActuatorDelay claimed 100 ms (see
# opendbc/car/mazda/interface.py).
MAX_LATERAL_JERK = 5.0  # m/s^3
MAX_LATERAL_ACCEL_NO_ROLL = 3.0  # m/s^2


def clamp(val, min_val, max_val):
  clamped_val = float(np.clip(val, min_val, max_val))
  return clamped_val, clamped_val != val

def smooth_value(val, prev_val, tau, dt=DT_MDL):
  alpha = 1 - np.exp(-dt/tau) if tau > 0 else 1
  return alpha * val + (1 - alpha) * prev_val

def clip_curvature(v_ego, prev_curvature, new_curvature, roll) -> tuple[float, bool]:
  # This function respects ISO lateral jerk and acceleration limits + a max curvature
  v_ego = max(v_ego, MIN_SPEED)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego ** 2)  # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  new_curvature = np.clip(new_curvature,
                          prev_curvature - max_curvature_rate * DT_CTRL,
                          prev_curvature + max_curvature_rate * DT_CTRL)

  roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY
  max_lat_accel = MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
  min_lat_accel = -MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
  new_curvature, limited_accel = clamp(new_curvature, min_lat_accel / v_ego ** 2, max_lat_accel / v_ego ** 2)

  new_curvature, limited_max_curv = clamp(new_curvature, -MAX_CURVATURE, MAX_CURVATURE)
  return float(new_curvature), limited_accel or limited_max_curv


def get_accel_from_plan(speeds, accels, t_idxs, action_t=DT_MDL, vEgoStopping=0.3):
  if len(speeds) == len(t_idxs):
    v_now = speeds[0]
    a_now = accels[0]
    if action_t < MIN_STABLE_DELAY:
      v_target = v_now + (action_t / MIN_STABLE_DELAY) * (np.interp(MIN_STABLE_DELAY, t_idxs, speeds) - v_now)
    else:
      v_target = np.interp(action_t, t_idxs, speeds)
    a_target = 2 * (v_target - v_now) / (action_t) - a_now
  else:
    v_now = 0.0
    v_target = 0.0
    a_target = 0.0
  should_stop = (v_now < vEgoStopping and a_target < 0.1)
  return a_target, should_stop

def curv_from_psis(psi_target, psi_rate, vego, action_t):
  vego = np.clip(vego, MIN_SPEED, np.inf)
  curv_from_psi = psi_target / (vego * action_t)
  return 2*curv_from_psi - psi_rate / vego

def get_curvature_from_plan(yaws, yaw_rates, t_idxs, vego, action_t):
  if action_t < MIN_STABLE_DELAY:
    psi_target = (action_t / MIN_STABLE_DELAY) * np.interp(MIN_STABLE_DELAY, t_idxs, yaws)
  else:
    psi_target = np.interp(action_t, t_idxs, yaws)
  psi_rate = yaw_rates[0]
  return curv_from_psis(psi_target, psi_rate, vego, action_t)
