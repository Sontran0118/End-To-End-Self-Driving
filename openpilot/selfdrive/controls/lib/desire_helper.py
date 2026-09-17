import os

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5

# NUDGELESS LANE CHANGE (Jetson port, off by default).
# Stock openpilot requires the driver to nudge the wheel TOWARD the new lane to
# leave preLaneChange -- signalling alone only arms it. That is a deliberate
# confirmation step: the blinker says "I intend to", the nudge says "now, and I
# am holding the wheel".
#
# With OP_AUTO_LANE_CHANGE=1 the dwell below replaces the nudge: hold the blinker
# for OP_AUTO_LANE_CHANGE_DELAY seconds and the change starts on its own. The
# blindspot check is NOT bypassed -- it still blocks, exactly as with a nudge --
# and the blinker must still be held the whole time, so releasing it aborts.
#
# What you give up is the hands-on confirmation. openpilot has no way to know you
# are looking, so the delay is the only thing standing between a brushed stalk
# and an unattended lane change. Keep it non-zero.
AUTO_LANE_CHANGE = os.environ.get("OP_AUTO_LANE_CHANGE", "0") == "1"
AUTO_LANE_CHANGE_DELAY = float(os.environ.get("OP_AUTO_LANE_CHANGE_DELAY", "1.5"))

# RE-ARM WITHOUT THE DWELL AFTER AN INCOMPLETE CHANGE.
#
# laneChangeStarting is left as soon as the model reports lane_change_prob < 0.02
# ("I am done"), which upstream treats as completion. On this car the model drops
# under that threshold about a second in, while the car is only at the lane edge.
#
# OBSERVED 2026-08-09: the car steers toward the next lane, stops at the boundary,
# returns, then tries again -- lc_state cycling 1->2->1->1->2 with lc_prob going
# 1.000 -> 0.015 -> 0.000 -> 1.000 at one sample per second.
#
# The return trip is the real damage, and it is caused by the GAP, not by the
# early exit. supercombo takes desire as a PULSE on the rising edge, so while the
# state machine sits back in preLaneChange waiting out AUTO_LANE_CHANGE_DELAY the
# model has no desire input at all -- it does the only thing left and re-centres
# in the lane it started from, undoing the progress made.
#
# So do not serve the dwell twice. It exists to make the FIRST commitment
# deliberate: hold the stalk long enough to prove intent. Coming back from an
# incomplete change is not a new intent, it is the same one still in progress, so
# re-arm immediately and let the next frame re-pulse. That turns a 1.5 s gap of
# counter-steering into a continuous sequence of pulses.
#
# Bounded by LANE_CHANGE_TIME_MAX measured across the WHOLE attempt rather than
# per-pulse, so a model that never reports completion cannot retry forever -- it
# gives up and drops the desire exactly as before. The blinker and blindspot
# checks are untouched and still abort at any point.
LC_REARM = os.environ.get("OP_LC_REARM", "1") == "1"

class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.pre_lane_change_timer = 0.0   # dwell in preLaneChange, for nudgeless
    self.attempt_timer = 0.0           # whole attempt, across re-arms (LC_REARM)
    self.prev_one_blinker = False
    self.desire = log.Desire.none

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob):
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
      self.attempt_timer = 0.0
    else:
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        self.pre_lane_change_timer = 0.0
        self.attempt_timer = 0.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
        self.lane_change_direction = self.get_lane_change_direction(carstate)

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        # Dwell only while the change is actually permissible: a blindspot
        # return must not quietly bank time toward an auto-start.
        if blindspot_detected:
          self.pre_lane_change_timer = 0.0
        else:
          self.pre_lane_change_timer += DT_MDL
        auto_ready = AUTO_LANE_CHANGE and self.pre_lane_change_timer >= AUTO_LANE_CHANGE_DELAY

        # An attempt already under way keeps ticking while it is re-armed, so the
        # LANE_CHANGE_TIME_MAX bound covers the gaps as well as the pulses.
        if self.attempt_timer > 0.0:
          self.attempt_timer += DT_MDL

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
          self.pre_lane_change_timer = 0.0
          self.attempt_timer = 0.0
        elif (torque_applied or auto_ready) and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0
          self.pre_lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL
        self.attempt_timer += DT_MDL

        # A blindspot return mid-change aborts it, same as it would have blocked
        # the start. Stock openpilot only checks this in preLaneChange, which
        # leaves a car arriving in the blindspot after the change began
        # unhandled -- the nudgeless path makes that window longer, not shorter.
        if ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
            (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right)):
          self.lane_change_state = LaneChangeState.preLaneChange
          self.lane_change_timer = 0.0
          self.pre_lane_change_timer = 0.0
          self.attempt_timer = 0.0

        elif lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self.lane_change_timer = 0.0
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self.get_lane_change_direction(carstate)
            if LC_REARM and self.attempt_timer < LANE_CHANGE_TIME_MAX:
              # Still mid-attempt: skip the dwell so the next frame re-pulses
              # before the model has time to steer back into the old lane.
              self.pre_lane_change_timer = AUTO_LANE_CHANGE_DELAY
            else:
              # Out of budget -- fall back to demanding a fresh deliberate dwell.
              self.pre_lane_change_timer = 0.0
              self.attempt_timer = 0.0
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none
            self.attempt_timer = 0.0

    self.prev_one_blinker = one_blinker and lateral_active

    self.desire = log.Desire.none
    if self.lane_change_state == LaneChangeState.laneChangeStarting:
      if self.lane_change_direction == LaneChangeDirection.left:
        self.desire = log.Desire.laneChangeLeft
      elif self.lane_change_direction == LaneChangeDirection.right:
        self.desire = log.Desire.laneChangeRight
