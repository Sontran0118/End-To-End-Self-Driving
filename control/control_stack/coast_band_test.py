#!/usr/bin/env python3
"""STANDARD +10 mph target with a coast band to +15 -- no brake-check at the top.

Two things are being checked, and they are separate:

  1. speed_target.SpeedTarget puts the TARGET at limit + 10 mph in STANDARD and
     limit + 15 mph in MADS, so STANDARD can never sit above MADS.
  2. govern_accel stops giving throttle AT the target but does not brake until
     the coast band above it is crossed.

Runs against the real functions, not a reimplementation. No CAN, no car.
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # control/

MPH = 1.609344
ok = fail = 0
def check(name, cond, detail=""):
    global ok, fail
    if cond: ok += 1; print("  PASS  %s" % name)
    else:    fail += 1; print("  FAIL  %s   %s" % (name, detail))

# ---------------------------------------------------------------- 1. targets
import speed_target as st_mod

check("STANDARD allowance is 10 mph", abs(st_mod.STD_OVER_KPH - 10 * MPH) < 0.01,
      "%.2f kph" % st_mod.STD_OVER_KPH)
check("MADS allowance is 15 mph", abs(st_mod.MADS_OVER_KPH - 15 * MPH) < 0.01,
      "%.2f kph" % st_mod.MADS_OVER_KPH)
check("MADS aims higher than STANDARD", st_mod.MADS_OVER_KPH > st_mod.STD_OVER_KPH,
      "the two modes were inverted")

def target(limit_kph, mads, inferred=False, v_max=200.0):
    s = st_mod.SpeedTarget(mads=mads, v_max_kph=v_max)
    return s._target_for(limit_kph, inferred)

L = 30 * MPH                                   # a 30 mph road
check("STANDARD 30 mph road -> 40 mph", abs(target(L, False) - 40 * MPH) < 0.05,
      "%.1f mph" % (target(L, False) / MPH))
check("MADS 30 mph road -> 45 mph", abs(target(L, True) - 45 * MPH) < 0.05,
      "%.1f mph" % (target(L, True) / MPH))
# The haircut applies to the doubtful limit, not to the driver's allowance.
_inf = target(L, False, inferred=True)
check("inferred limit discounted, allowance not",
      abs(_inf - (L * st_mod.INFERRED_SCALE + st_mod.STD_OVER_KPH)) < 0.05,
      "%.1f kph" % _inf)
check("V_MAX still bounds the target", target(200.0, True, v_max=100.0) == 100.0)

# ------------------------------------------------------- 2. the coast band
import importlib
os.environ.setdefault("OP_NO_CAMERA", "1")
dw = importlib.import_module("dashcam_web")
govern_accel = dw.govern_accel

check("coast band is 5 mph", abs(dw.V_COAST_BAND_KPH - 5 * MPH) < 0.01,
      "%.2f kph" % dw.V_COAST_BAND_KPH)

CAP = 40 * MPH            # STANDARD target on a 30 mph road
BRAKE_AT = CAP + dw.V_COAST_BAND_KPH

def a_at(v_kph, a_model=1.0, cap=CAP):
    """Steady state: a_prev = the answer, so the jerk limit is not what we measure."""
    a = 0.0
    for _ in range(200):
        a, why = govern_accel(a_model, v_kph / 3.6, None, None, a, 0.05, v_max_kph=cap)
    return a, why

a, why = a_at(CAP - 8.0)
check("below the target it still accelerates", a > 0.05, "a=%.3f %s" % (a, why))

a, why = a_at(CAP - 0.2)
check("arrives at the target with ~no throttle", abs(a) < 0.05, "a=%.3f %s" % (a, why))

for over_mph in (0.5, 2.0, 4.9):
    a, why = a_at(CAP + over_mph * MPH)
    check("+%.1f mph over: coasts, no brake" % over_mph, abs(a) < 1e-6,
          "a=%.3f %s" % (a, why))
    check("+%.1f mph over: reason says coast" % over_mph, why.startswith("coast"),
          "why=%r" % why)

# THE REGRESSION. Before the band this braked, and that was the brake-check.
a_in_band, _ = a_at(CAP + 3.0 * MPH)
check("no braking anywhere inside the band", a_in_band >= 0.0, "a=%.3f" % a_in_band)

a, why = a_at(BRAKE_AT + 3.0)
check("past the band it brakes", a < -0.05, "a=%.3f %s" % (a, why))
check("braking grows with the overshoot",
      a_at(BRAKE_AT + 8.0)[0] < a_at(BRAKE_AT + 3.0)[0])

# ------------------------------------------- 3. the model owns lead distance
# The geometric time-gap rule is off. It used to fire at 1.8 s and, measured over
# 708 samples, inverted the model's sign in 59% of them -- asked for +1.28 m/s^2,
# commanded -0.71. These checks are the regression against putting it back by
# accident.
check("time-gap rule is off by default", dw.T_FOLLOW_MIN == 0.0,
      "T_FOLLOW_MIN=%.2f" % dw.T_FOLLOW_MIN)

# 8 m at 25 mph is a 0.72 s gap -- deep inside anything the old rule would have
# braked hard for. Well under CAP (40 mph) on purpose, so the accel ceiling and
# the approach taper are not what is being measured, and iterated to steady state
# so the jerk limit is not either.
V_MID = 25 * MPH / 3.6
a_close, why = 0.0, ""
for _ in range(200):
    a_close, why = govern_accel(0.8, V_MID, 8.0, 0.95, a_close, 0.05, v_max_kph=CAP)
check("close lead does not force a brake", a_close > 0.0,
      "a=%.3f %s -- the geometric rule is back" % (a_close, why))
check("close lead leaves no lead reason", "lead" not in why, "why=%r" % why)

# The model's OWN braking still passes through -- nothing here clamps negative
# accel, which is what makes removing the rule safe.
a_brake, why = govern_accel(-2.5, V_MID, 8.0, 0.95, -2.5, 0.05, v_max_kph=CAP)
check("model braking passes through untouched", abs(a_brake - (-2.5)) < 1e-6,
      "a=%.3f %s" % (a_brake, why))

# Restorable: a positive OP_T_FOLLOW brings the old behaviour back verbatim.
# Iterated to steady state: one call only moves by the jerk limit (0.1 m/s^2 at
# dt=0.05), so a single sample cannot show where the rule settles.
_saved = dw.T_FOLLOW_MIN
dw.T_FOLLOW_MIN = 1.8
a_restored, why = 0.8, ""
for _ in range(200):
    a_restored, why = govern_accel(0.8, V_MID, 8.0, 0.95, a_restored, 0.05,
                                   v_max_kph=CAP)
dw.T_FOLLOW_MIN = _saved
check("OP_T_FOLLOW restores the rule", a_restored < 0.0 and "lead" in why,
      "a=%.3f %s" % (a_restored, why))

# The hard ceiling is not a speed limit: it must brake at itself, no band.
a, why = govern_accel(1.0, (dw.V_MAX_KPH + 2.0) / 3.6, None, None, -0.5, 0.05,
                      v_max_kph=None)
check("V_MAX_KPH gets no coast band", a < 0.0, "a=%.3f %s" % (a, why))
# ...and a limit cap sitting AT V_MAX cannot be coasted past either.
a, why = a_at(dw.V_MAX_KPH + 2.0, cap=dw.V_MAX_KPH)
check("band never carries the car past V_MAX_KPH", a < 0.0, "a=%.3f %s" % (a, why))

print("\n=== RESULT: %d passed, %d failed ===" % (ok, fail))
sys.exit(0 if fail == 0 else 1)
