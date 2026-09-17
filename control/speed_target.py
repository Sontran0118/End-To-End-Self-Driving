#!/usr/bin/env python3
"""Turn a GPS position into a cruise target, per drive mode.

    STANDARD   target = the posted speed limit + OP_STD_OVER_MPH  (default 10 mph)
    MADS       target = the posted speed limit + OP_MADS_OVER_MPH (default 15 mph)

THE TARGET IS WHERE ACCELERATION STOPS, NOT WHERE BRAKING STARTS. Those were the
same number until 2026-08-13, which is what made the car brake-check the moment it
touched the target. `govern_accel` now opens a COAST BAND above this number
(OP_V_COAST_BAND_MPH, default 5 mph) inside which it commands neither throttle
nor brake and simply lets the car roll. So STANDARD settles at limit + 10 and only
brakes past limit + 15.

WHY A SEPARATE MODULE. The MPC target used to be one env var read once at
startup (`OP_V_TARGET_KPH`), so nothing in the stack had a place to reason about
*where the number comes from*. Everything here is about the failure cases --
what to do when there is no fix, no matching road, or only a guessed limit --
because those are the normal condition on a real drive, not the exception.

UNITS. The over-limit allowances are in MPH because that is the unit they were
specified in, and getting it wrong is a 60% error. Everything else here stays in
km/h to match `OP_V_TARGET_KPH` and `V_MAX_KPH`. `OP_MADS_OVER_KPH` is still
honoured when set explicitly, for anything that already passes it.

NOTHING HERE LATCHES INDEFINITELY. A speed limit is a fact about a place, and it
stops being true the moment the position is stale. On loss of fix or match the
last limit is held for HOLD_S (you are probably still on the same road), then
the target decays to FALLBACK_KPH rather than staying at a highway number
somewhere it no longer applies. This is the opposite of `set_speed_raw`'s
forever-latch in dashcam.py, which cost a whole evening of misdiagnosis.

THE TARGET IS NOT AUTHORITY. This only sets what the MPC aims for. `govern_accel`
still clamps afterwards, `V_MAX_KPH` is still the hard ceiling, and the panda's
limits are untouched. A wrong number here makes the car aim for the wrong speed;
it cannot make the car exceed the caps that were already there.
"""
from __future__ import annotations

import os
import time

MPS_TO_KPH = 3.6
KPH_TO_MPS = 1.0 / 3.6
MPH_TO_KPH = 1.609344


def _envf(name: str, default: float) -> float:
  try:
    return float(os.environ.get(name, default))
  except (TypeError, ValueError):
    return default


# How far above the posted limit each mode aims.
#
# STANDARD used to aim exactly AT the limit, which meant any slope, any lag in
# the taper, or simply the limit stepping down put the car over it and the cap
# braking fired. Aiming 10 mph high gives the controller somewhere to sit.
#
# WHY MADS MOVED FROM 15 km/h TO 15 mph. The original request was "+15" with no
# unit and this file recorded the ambiguity rather than resolving it. Adding
# STANDARD at +10 mph (16.1 km/h) forced the issue: at the old default MADS
# (+15 km/h) STANDARD would have aimed HIGHER than MADS, inverting the two modes.
# 15 mph is also what was actually asked for -- "mad mode is 15mph more".
#
# OP_MADS_OVER_KPH still wins if set, so an existing launcher keeps its number.
STD_OVER_KPH  = _envf("OP_STD_OVER_MPH", 10.0) * MPH_TO_KPH
MADS_OVER_KPH = _envf("OP_MADS_OVER_KPH",
                      _envf("OP_MADS_OVER_MPH", 15.0) * MPH_TO_KPH)

# Keep the last known limit this long after the fix or the road match is lost.
# 20 s at 100 km/h is ~550 m -- long enough to cross a tunnel, an overpass or a
# gap in the map, short enough not to carry a limit onto a different road.
HOLD_S = _envf("OP_LIMIT_HOLD_S", 20.0)

# Where the target goes once the hold expires and we genuinely do not know.
# Deliberately low: unknown road, no data, so aim for something that is legal
# nearly everywhere and let the driver override by driving.
FALLBACK_KPH = _envf("OP_LIMIT_FALLBACK_KPH", 40.0)

# Inferred limits (from the highway-type table, no maxspeed tag on the way) get
# a haircut. They are a guess about a road nobody surveyed.
INFERRED_SCALE = _envf("OP_LIMIT_INFERRED_SCALE", 0.9)

# The target moves at most this fast, so a map error or a bad match produces a
# ramp the driver can react to rather than a step. km/h per second.
SLEW_KPH_S = _envf("OP_LIMIT_SLEW_KPH_S", 8.0)

# How far BELOW our measured speed an INFERRED limit may sit before it is treated
# as a bad match rather than a real limit. See the long note in update().
#
# 30 km/h (~19 mph) is wide enough to still obey a genuine inferred limit the
# driver is over -- a 50 km/h residential road while doing 70 is believed and
# acted on -- but rejects the 24 km/h service-road match that appeared while
# travelling at 88. Explicit maxspeed tags are never rejected.
#
# Set to 0 to disable the check and believe every match, as before.
INFERRED_MAX_DROP_KPH = _envf("OP_LIMIT_INFERRED_MAX_DROP_KPH", 30.0)


class SpeedTarget:
  """Holds the current cruise target and the reason for it.

  `update()` is cheap and expects to be called at a few Hz; the GPS itself only
  produces ~1 Hz, so calling faster costs nothing and gains nothing.
  """

  def __init__(self, mads: bool, v_max_kph: float, index=None, gps=None):
    self.mads = bool(mads)
    self.v_max_kph = float(v_max_kph)
    self.index = index
    self.gps = gps
    self.target_kph = FALLBACK_KPH
    self.limit_kph: float | None = None
    self.inferred = False
    self.highway = ""
    self.why = "startup"
    self._last_good_t = 0.0
    self._last_update = 0.0

  # ---- the mode rule -------------------------------------------------------
  def _target_for(self, limit_kph: float, inferred: bool) -> float:
    base = limit_kph * (INFERRED_SCALE if inferred else 1.0)
    # The allowance is added AFTER the inferred haircut, not before. A guessed
    # limit gets discounted because the limit itself is doubtful; the driver's
    # allowance over whatever the limit is has nothing to do with that doubt, so
    # scaling it too would silently shrink the band on ~89% of roads.
    base += MADS_OVER_KPH if self.mads else STD_OVER_KPH
    return min(base, self.v_max_kph)

  # ---- main ----------------------------------------------------------------
  def update(self, v_ego_kph: float | None = None) -> float:
    """v_ego_kph is optional and is used ONLY to sanity-check inferred limits.

    Without it the old behaviour is unchanged: every match is believed.
    """
    now = time.time()
    dt = (now - self._last_update) if self._last_update else 0.0
    self._last_update = now

    want = None
    if self.index is not None and self.gps is not None:
      fx = self.gps.fix()          # None when stale -- see gps_reader
      if fx is None:
        self.why = "no gps fix"
      else:
        r = self.index.lookup(fx["lat"], fx["lon"], fx.get("heading_deg"))
        if r is None:
          self.why = "no road matched"
        else:
          _lim = r["limit_mps"] * MPS_TO_KPH

          # DISTRUST AN INFERRED LIMIT THAT IS FAR BELOW OUR ACTUAL SPEED.
          #
          # The index matches within 25 m perpendicular -- deliberately wide, so a
          # divided highway's far carriageway still matches -- which is also wide
          # enough for a parallel service road, driveway or slip road to match.
          # ~89% of ways carry no maxspeed tag, so most matches are a guess from
          # the highway type.
          #
          # MEASURED 2026-08-12 over one drive: the limit took the values
          # 88.5 / 64.4 / 56.3 / 48.3 / 24.1 kph, and the INFERRED 24.1 (15 mph,
          # a residential/service default) accounted for 30% of samples. Believed
          # while actually travelling at ~88, it commands sustained braking toward
          # ~22 kph -- released again when the match returns to the real road.
          # That is the "rush to the limit, brake, go over, brake" cycle.
          #
          # Physical argument: on a genuine 15 mph road you are not doing 55. If an
          # INFERRED limit sits more than INFERRED_MAX_DROP_KPH below our measured
          # speed, the match is far likelier to be the wrong road than the driver
          # to be exceeding it that grossly -- so keep the previous limit rather
          # than brake for a guess.
          #
          # An EXPLICIT maxspeed tag is always honoured, however large the drop:
          # a surveyed limit is exactly what we want to obey.
          _bad_inferred = (INFERRED_MAX_DROP_KPH > 0.0 and r["inferred"]
                           and v_ego_kph is not None
                           and _lim < (v_ego_kph - INFERRED_MAX_DROP_KPH))
          if _bad_inferred:
            self.why = ("%s %.0f kph inferred REJECTED (%.0f below v=%.0f)"
                        % (r["highway"], _lim, v_ego_kph - _lim, v_ego_kph))
          else:
            self.limit_kph = _lim
            self.inferred = r["inferred"]
            self.highway = r["highway"]
            self._last_good_t = now
            want = self._target_for(self.limit_kph, self.inferred)
            self.why = "%s %.0f kph%s" % (r["highway"], self.limit_kph,
                                          " (inferred)" if r["inferred"] else "")
    else:
      self.why = "no index/gps"

    if want is None:
      held = (now - self._last_good_t) if self._last_good_t else 1e9
      if self.limit_kph is not None and held <= HOLD_S:
        # Still probably the same road. Keep the number, say we are coasting.
        want = self._target_for(self.limit_kph, self.inferred)
        self.why += " (hold %.0fs)" % held
      else:
        want = min(FALLBACK_KPH, self.v_max_kph)
        if self.limit_kph is not None:
          self.limit_kph = None
          self.highway = ""
        self.why += " -> fallback"

    # Slew. A step change in the target is a step change in commanded accel.
    if dt > 0.0 and SLEW_KPH_S > 0.0:
      step = SLEW_KPH_S * dt
      if want > self.target_kph:
        self.target_kph = min(want, self.target_kph + step)
      else:
        # Downward is not slew-limited: if the limit drops, aim lower NOW.
        # The MPC and govern_accel decide how hard to decelerate; delaying the
        # target only means overshooting the lower limit for longer.
        self.target_kph = want
    else:
      self.target_kph = want

    return self.target_kph

  # ---- reporting -----------------------------------------------------------
  def status(self) -> dict:
    return {
      "mode": "MADS" if self.mads else "STANDARD",
      "target_kph": round(self.target_kph, 1),
      "limit_kph": round(self.limit_kph, 1) if self.limit_kph is not None else None,
      "inferred": self.inferred,
      "highway": self.highway,
      "over_kph": round(MADS_OVER_KPH if self.mads else STD_OVER_KPH, 1),
      "why": self.why,
    }

  def line(self) -> str:
    """One compact field for the CAR line."""
    lim = ("%.0f" % self.limit_kph) if self.limit_kph is not None else "--"
    over = MADS_OVER_KPH if self.mads else STD_OVER_KPH
    return "tgt=%.0f lim=%s%s %s+%.0f" % (
      self.target_kph, lim, "i" if self.inferred else "",
      "MADS" if self.mads else "STD", over)


if __name__ == "__main__":
  import sys
  sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
  from gps_reader import GpsReader
  from speed_limits import SpeedLimitIndex

  mads = "--mads" in sys.argv
  ix = SpeedLimitIndex()
  print("index:", ix.status())
  if not ix.ok:
    sys.exit(1)
  st = SpeedTarget(mads=mads, v_max_kph=_envf("OP_V_MAX_KPH", 120.0),
                   index=ix, gps=GpsReader().start())
  print("mode:", "MADS +%.0f kph" % MADS_OVER_KPH if mads else "STANDARD")
  try:
    while True:
      time.sleep(1.0)
      st.update()
      print("  %s   | %s" % (st.line(), st.why))
  except KeyboardInterrupt:
    pass
