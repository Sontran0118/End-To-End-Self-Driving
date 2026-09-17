#!/usr/bin/env python3
"""Phone-viewable dashboard for the dashcam run.

Serves on 0.0.0.0:8080 using only the stdlib (no flask on this box):
  /            live page: MJPEG video + calibration convergence + car state
  /stream.mjpg motion-JPEG of what the MODEL sees
  /status.json machine-readable state (also what the page polls)

Runs the same pipeline as dashcam.py -- panda read-only in SAFETY_SILENT, real
CAN -> carState, camera -> supercombo -> curvature -> modelV2 -> controlsd -- and
adds the view layer so you can watch calibration converge from the phone instead
of guessing after the fact.

The calibration panel is the point: rpy is fed from the model's pose odometry and
ONLY updates while the car is moving, so if it stays at 0,0,0 the drive is not
producing usable data and you can see that immediately rather than at the end.

Usage:  python3 dashcam_web.py [--port 8080] [--no-daemons] [--arm]

ARMING (real transmit):
  By default the panda is held in SAFETY_SILENT and NOTHING is transmitted --
  this is the dashcam behaviour. Passing --arm (or DASHCAM_ARM=1 in the env)
  switches the panda to the Mazda safety model and streams the real
  0x243 CAM_LKAS steering frame at 70 Hz, built from the same torque controlsd
  produces. The panda firmware's Mazda safety hooks are the hardware net: they
  only pass a steering frame when the car reports cruise engaged (controls
  allowed) and clamp torque/rate to the Mazda limits (800 / 10up / 25down),
  exactly as validated by override_test2.py. Read the arming banner before use.
"""
import os, sys, time, json, math, threading, argparse, socket, struct, collections
_CX5_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir))
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

os.environ.setdefault("PARAMS_ROOT", "/tmp/op_params")
sys.path.insert(0, "/home/tran/openpilot_jetson")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # control/

import numpy as np
import cv2


def _calib_file_path():
    """Where op_calibrate persists the learned mount rpy (single source of truth)."""
    from op_calibrate import CALIB_FILE
    return CALIB_FILE


# --- longitudinal governor -------------------------------------------------
# There is no plannerd and no longitudinal MPC on this board: longitudinalPlan
# .aTarget comes straight from the model's action head. That means none of the
# things a planner normally provides exist -- no jerk limit, no cruise-speed
# tracking, no lead-distance following. The model just predicts what a human
# would do from this frame, and the only clamps are longitudinal.py's accel
# scale and the panda's +-2000 window.
#
# These four put the missing floor/ceiling back. All are env-overridable so they
# can be tuned from the command line without editing code.
#   OP_ACCEL_MAX   m/s^2, ceiling on commanded acceleration (braking unclamped)
#   OP_JERK_MAX    m/s^3, how fast aTarget may change -- this is what stops the
#                  throttle stepping straight to full
#   OP_V_MAX_KPH   km/h, speed cap: above it, positive accel is not allowed
#   OP_T_FOLLOW    s, minimum time gap to the lead before accel is cut.
#                  DEFAULT 0 = OFF: the model owns following distance. See the
#                  long note at step 1 of govern_accel.
_envf = lambda k, d: float(os.environ.get(k, d))
# Absolute ceiling, all speeds. Lowered 1.2 -> 0.9 alongside OP_ACCEL_SCALE so it
# is a real guarantee rather than a number the scaled curve never reaches: at the
# old 1.2 nothing below 0 m/s could ever hit it, so it capped nothing.
ACCEL_MAX_CMD = _envf("OP_ACCEL_MAX", 0.9)
JERK_MAX_CMD  = _envf("OP_JERK_MAX", 2.0)      # m/s^3
# SPEED-DEPENDENT ACCELERATION CEILING, from openpilot's own A_CRUISE_MAX table
# (longitudinal_planner.py:18-19). A FLAT cap is the wrong shape: 1.2 m/s^2 is
# gentler than openpilot from a standstill (it uses 1.6) and TWICE as aggressive
# at highway speed (it uses 0.6 at 40 m/s). The same number cannot be right at
# both ends, and the highway end is where a brisk command is least welcome.
#
# The effective ceiling is min(ACCEL_MAX_CMD, interp(v_ego) * OP_ACCEL_SCALE).
# OP_ACCEL_SCALE is the single "how eager" knob: 1.0 matches openpilot, lower is
# gentler everywhere while keeping the speed-dependent shape.
#
# BRAKING IS NOT AFFECTED. govern_accel only ever clamps positive accel with
# this -- see the docstring -- so making the car gentler cannot make it worse at
# slowing down.
ACCEL_MAX_BP = (0.0, 10.0, 25.0, 40.0)         # m/s
ACCEL_MAX_V  = (1.6, 1.2, 0.8, 0.6)            # m/s^2, openpilot's own numbers
# GENTLER EVERYWHERE, on request 2026-08-14: "reduce the acceleration at all
# speeds". Scaling rather than reshaping keeps openpilot's curve -- brisker from
# rest where you need to actually get moving, calmest at speed where a firm push
# is least welcome -- and makes the reduction one reversible number.
#
#   v_ego        openpilot   was (0.70)   now (0.55)
#   0 m/s   0 kph    1.60       1.12         0.88
#   10 m/s 36 kph    1.20       0.84         0.66
#   25 m/s 90 kph    0.80       0.56         0.44
#   40 m/s 144 kph   0.60       0.42         0.33
#
# ~21% off the previous setting at every speed. Raise OP_ACCEL_SCALE back toward
# 0.7 if it now feels too slow to reach the target; braking is untouched either
# way, since this only ever clamps POSITIVE accel.
ACCEL_SCALE  = _envf("OP_ACCEL_SCALE", 0.55)
# Width of the approach taper below the speed cap, km/h. Available acceleration
# falls linearly to zero across this band so the car SETTLES onto the limit
# rather than overshooting and being braked back. Wider = gentler arrival and
# more time spent slightly under the limit; narrower = closer to the old
# hunting behaviour. 10 km/h is roughly 6 mph of run-in.
V_TAPER_KPH = _envf("OP_V_TAPER_KPH", 10.0)


def _max_accel_for(v_ego):
    """Speed-dependent positive-accel ceiling, scaled and hard-capped."""
    v = max(0.0, float(v_ego))
    if v <= ACCEL_MAX_BP[0]:
        base = ACCEL_MAX_V[0]
    elif v >= ACCEL_MAX_BP[-1]:
        base = ACCEL_MAX_V[-1]
    else:
        base = ACCEL_MAX_V[-1]
        for i in range(1, len(ACCEL_MAX_BP)):
            if v <= ACCEL_MAX_BP[i]:
                x0, x1 = ACCEL_MAX_BP[i - 1], ACCEL_MAX_BP[i]
                y0, y1 = ACCEL_MAX_V[i - 1], ACCEL_MAX_V[i]
                base = y0 + (y1 - y0) * (v - x0) / (x1 - x0)
                break
    return min(ACCEL_MAX_CMD, base * ACCEL_SCALE)
# Set in MPH, because that is the unit the limit was specified in. Everything
# downstream works in m/s and the dashboard reads km/h, so the conversion lives
# here once rather than in someone's head. OP_V_MAX_KPH still overrides directly.
V_MAX_MPH     = _envf("OP_V_MAX_MPH", 72.0)
V_MAX_KPH     = _envf("OP_V_MAX_KPH", V_MAX_MPH * 1.609344)
# FOLLOWING DISTANCE IS THE MODEL'S JOB. 0 disables the geometric rule entirely;
# see step 1 of govern_accel for what it was doing and why it had to go. Set a
# positive number to put it back (1.45 = openpilot standard, 1.25 aggressive,
# 1.75 relaxed; this port used to hardcode 1.8, i.e. further back than relaxed).
T_FOLLOW_MIN  = _envf("OP_T_FOLLOW", 0.0)
# How hard each cap is allowed to pull back. Both are well inside the panda's
# +-2000 raw window (-2.0 m/s^2 is about -820 raw) and gentler than ACCEL_MIN
# (-3.5), so the model can still brake harder than either cap ever will.
# Only used when OP_T_FOLLOW is set back to a positive number; the geometric
# lead rule is off by default and the model brakes for leads itself.
LEAD_DECEL_MAX = _envf("OP_LEAD_DECEL", 1.5)   # m/s^2 at half the target gap
V_CAP_GAIN     = _envf("OP_VCAP_GAIN", 0.10)   # m/s^2 per km/h over the limit
V_CAP_DECEL_MAX = _envf("OP_VCAP_DECEL", 1.0)  # m/s^2 ceiling on cap braking
# COAST BAND above the speed-limit cap: how far the car may roll past the target
# before the cap brakes for it. See step 2b of govern_accel. In MPH like the
# allowances in speed_target.py, because that is the unit these were asked for.
# 0 restores the old behaviour, where braking began the instant the target was
# crossed. Only applies to the GPS/OSM cap -- V_MAX_KPH still brakes at itself.
V_COAST_BAND_KPH = _envf("OP_V_COAST_BAND_MPH", 5.0) * 1.609344


# --- longitudinal MPC ---------------------------------------------------------
# OP_MPC=1 replaces govern_accel's clamp with openpilot's real longitudinal MPC.
#
# WHY. govern_accel is a BOUND, not a plan: it sees one number (the model action
# head's current desired accel), limits its size and how fast it may change, and
# passes it on. It has no horizon, so it cannot anticipate anything -- approaching
# a slower car it keeps commanding accel until the gap check trips, then cuts.
# That is the surging you feel.
#
# The MPC solves a constrained optimisation over a 13-step horizon each cycle:
# it predicts how speed and accel evolve, penalises jerk and closing on the lead,
# respects hard accel/jerk limits, and applies only the first step. Because it
# re-plans every cycle it starts easing off SECONDS before it needs to.
#
# acados (the solver) ships prebuilt for aarch64 in comma-deps-acados and was
# already installed; the generated solver was built into
# longitudinal_mpc_lib/c_generated_code/acados_ocp_solver_pyx.so.
#
# CRUISE TARGET. Upstream takes v_cruise from the set speed. Under MADS there is
# no set-speed concept, so the speed cap doubles as the cruise target: the MPC is
# asked to reach V_MAX_KPH and no more. OP_V_TARGET_KPH overrides it.
#
# govern_accel still runs AFTER the MPC as a safety clamp. The MPC should produce
# sane values, but a bound costs nothing and keeps the lead/speed/jerk caps as a
# second line if the solver ever returns something wild.
MPC_ENABLED = os.environ.get("OP_MPC") == "1"
# Follow distance, via the MPC's personality parameter. log.capnp:140-144 orders
# the enum aggressive=0, standard=1, relaxed=2, and long_mpc.py:73 maps those to
# T_FOLLOW 1.25 / 1.45 / 1.75 s. The call sites used a bare 0, i.e. AGGRESSIVE,
# which was almost certainly meant to read "default" -- so the car followed at the
# tightest gap on offer with no way to change it.
_PERSONALITY = {"aggressive": 0, "standard": 1, "relaxed": 2}.get(
    os.environ.get("OP_PERSONALITY", "standard").strip().lower(), 1)


class _MpcLead:
    """Minimal stand-in for radarState.leadOne/leadTwo.

    LongitudinalMpc.process_lead() reads exactly these four fields. This board
    publishes radarState as an empty message, and the lead comes from the model's
    own lead head instead of a radar -- which is the same source openpilot uses
    when the radar is unavailable.
    """
    def __init__(self, d=None, v=None, a=None, prob=None, v_ego=0.0):
        ok = (d is not None and prob is not None and prob > 0.5)
        # process_lead() checks .present (NOT .status) and reads dRel/vLead/
        # aLeadK/aLeadTau. When absent it fabricates a fast lead 50 m ahead so the
        # solver stays in the same mode -- so a missing lead is handled for us.
        self.present = bool(ok)
        self.status = bool(ok)
        self.dRel = float(d) if ok else 0.0
        # THE MODEL'S LEAD VELOCITY IS ABSOLUTE. Settled 2026-08-12 from
        # openpilot's own radard.py, which is unambiguous in three places:
        #
        #   :119  prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])
        #   :129  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10)
        #   :137  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
        #
        # 119 and 129 compare `vRel + v_ego` -- an absolute ground speed -- against
        # lead.v[0], and 137 derives the RELATIVE speed by SUBTRACTING v_ego from
        # it. lead.v[0] is therefore ground speed, not closing speed.
        #
        # This used to add v_ego on top, which double-counted it: following a car
        # at 30 kph while doing 30 kph told the MPC the lead was doing 60 and
        # pulling away, so it accelerated instead of holding station, then braked
        # hard when the gap finally collapsed. REPORTED FROM THE CAR 2026-08-12 as
        # "does not stop when there's leads, accelerates really hard, and brakes
        # really hard" -- all three follow from this one sign error.
        #
        # govern_accel never showed it because it is a bound, not a planner: it
        # clamps the end-to-end action head, which slows for leads implicitly, and
        # never chases a target that a wrong vLead could run away with.
        self.vLead = float(max(0.0, v if (ok and v is not None) else 0.0))
        self.aLeadK = float(a) if (ok and a is not None) else 0.0
        self.vLeadK = self.vLead
        self.modelProb = float(prob) if ok else 0.0
        self.aLeadTau = 1.5


class _MpcRadar:
    def __init__(self, lead):
        self.leadOne = lead
        self.leadTwo = _MpcLead()


class RxLiveness:
    """When did the bus last say anything?

    A decoded VALUE can never establish liveness -- carstate fields are plain
    assignments that hold their last value when the frames stop, which is
    indistinguishable from the car still reporting that value. Only an arrival
    TIME can. This is that time, shared by everything that needs to know whether
    what it is reading is current or merely remembered.

    Fails CLOSED before the first frame: at startup nothing has been observed,
    so nothing may be asserted about the car.
    """

    # 2 s is ~6x the longest legitimate gap between bus-0 frames (PEDALS 0x165
    # alone runs at 100 Hz), so it cannot fire on a live bus; and it is well
    # inside the radar's own ~5 s UDS session timeout, so releasing at 2 s gets
    # the radar back promptly rather than after a further long wait.
    DEFAULT_DEAD_S = 2.0

    def __init__(self, dead_s=None):
        self.t = 0.0
        self.dead_s = (_envf("OP_RX_DEAD_S", self.DEFAULT_DEAD_S)
                       if dead_s is None else float(dead_s))

    def stamp(self, now=None):
        self.t = now if now is not None else time.time()

    def alive(self, now=None):
        if self.dead_s <= 0.0:
            return True                      # check disabled
        if not self.t:
            return False                     # nothing has ever arrived
        return ((now if now is not None else time.time()) - self.t) < self.dead_s

    def age(self, now=None):
        if not self.t:
            return float("inf")
        return (now if now is not None else time.time()) - self.t


def govern_accel(a_model, v_ego, lead_d, lead_prob, a_prev, dt, v_max_kph=None):
    """Clamp the action head's raw acceleration into something drivable.

    Returns (a_cmd, reason). Braking is never limited by the ceiling or the
    speed cap -- only acceleration is. The jerk limit applies in both
    directions, because a step change to hard braking is as bad as one to
    full throttle.

    It does NOT decide following distance. The action head already does, and
    this function overriding it was the brake-checking -- see step 1.
    """
    a = float(a_model)
    why = ""

    # 1. LEAD FOLLOWING -- OFF BY DEFAULT. The model handles the lead itself.
    #
    # This used to impose a fixed 1.8 s time gap on top of the action head, with
    # the premise "the model has no notion of the gap WE want to keep". That
    # premise is wrong for an end-to-end model: it was trained on humans
    # following cars, and `desiredAcceleration` already prices in the lead. So
    # the rule was not clamping the model, it was ARGUING with it -- and the
    # min() meant the fixed formula won every disagreement.
    #
    # MEASURED across this port's drive logs, 708 samples where the rule fired:
    #
    #   417 (59%)  the model asked for ACCELERATION and the rule commanded a
    #              BRAKE instead -- a sign inversion, not a limit. e.g.
    #              a_raw=+1.28 -> a_cmd=-0.71, a_raw=+0.98 -> a_cmd=-1.49
    #   560 (79%)  fired at a gap of 1.45 s or MORE -- distances openpilot's own
    #              standard personality considers fine
    #   median gap at override: 1.60 s
    #
    # Reported from the car as brake-checking and "the distance from the lead
    # feels hardcoded". It was: 1.8 s is further back than upstream's RELAXED
    # personality (1.75), and at 100 km/h it began braking at 50 m and reached
    # the full -1.5 m/s^2 by 25 m regardless of what the model saw.
    #
    # The safety net that remains is the one that was never in dispute: the
    # accel ceiling, the speed cap and the jerk limit below, none of which
    # depend on the lead. Model braking passes through untouched -- it always
    # did, since nothing here clamps negative accel.
    #
    # Set OP_T_FOLLOW to a positive number to restore the rule.
    if (T_FOLLOW_MIN > 0.0 and lead_prob is not None and lead_d is not None
            and lead_prob > 0.5 and v_ego > 1.0):
        gap = lead_d / max(v_ego, 0.1)
        if gap < T_FOLLOW_MIN:
            # Proportional, not a step: how far inside the target gap we are, 0 at
            # the target and 1 when halved. A cliff at exactly T_FOLLOW would make
            # the car surge and lift repeatedly at the boundary.
            frac = min(1.0, (T_FOLLOW_MIN - gap) / max(T_FOLLOW_MIN * 0.5, 1e-3))
            a = min(a, -LEAD_DECEL_MAX * frac)
            why = "lead %.1fs" % gap

    # 2. speed cap. Also proportional: above the limit, command braking that grows
    #    with the overshoot, so the cap HOLDS on a descent instead of merely
    #    stopping acceleration and letting gravity carry the car past it.
    # v_max_kph is the LIVE cap when OP_SPEED_LIMIT is on -- the posted limit from
    # GPS + OSM plus the mode's over-limit allowance (OP_STD_OVER_MPH /
    # OP_MADS_OVER_MPH). None falls back to the fixed
    # V_MAX_KPH, so behaviour is unchanged when no limit is available.
    #
    # Never ABOVE the fixed ceiling: the posted limit can only ever lower the cap.
    # A bad lookup can make the car aim slower than it needs to; it can never let
    # it exceed a bound that already applied.
    _cap = V_MAX_KPH if v_max_kph is None else min(float(v_max_kph), V_MAX_KPH)
    v_kph = v_ego * 3.6

    # 2b. COAST BAND -- where acceleration stops is NOT where braking starts.
    #
    # Until now those were one number, so the instant the car touched the target
    # the proportional braking in step 2 fired. On any downgrade, or just from the
    # last of the taper's momentum, that is a brake-check: a short deceleration
    # nobody asked for, to hold a speed the car was only a fraction over.
    #
    # Two thresholds instead. _cap is the ACCEL ceiling -- past it the car is
    # never given throttle. _brake_at sits V_COAST_BAND_KPH higher, and only past
    # THAT does the cap command deceleration. Between them the car rolls: no
    # throttle, no brake, whatever the road does. With the defaults, STANDARD
    # settles at the limit +10 mph and does not brake until +15.
    #
    # Only ever above a SPEED-LIMIT cap. V_MAX_KPH is the hard ceiling of this
    # stack and keeps braking exactly at itself -- min() below means the coast
    # band can never carry the car past it.
    _brake_at = _cap if v_max_kph is None else min(_cap + V_COAST_BAND_KPH, V_MAX_KPH)

    if v_kph > _brake_at:
        over = v_kph - _brake_at
        a = min(a, -min(V_CAP_DECEL_MAX, V_CAP_GAIN * over))
        why = why or "v>%.0f (+%.1f)" % (_brake_at, over)
    elif v_kph > _cap:
        # In the band. Cut throttle, command nothing else -- min(a, 0.0) rather
        # than a = 0.0 so a lead-following brake from step 1 still gets through.
        a = min(a, 0.0)
        why = why or "coast +%.0f" % (v_kph - _cap)

    # 3. acceleration ceiling -- speed-dependent, see _max_accel_for
    _amax = _max_accel_for(v_ego)

    # 3b. TAPER THE APPROACH TO THE CAP.
    #
    # Step 2 only reacts AFTER the cap is crossed, so the model accelerated at the
    # full ceiling right up to the limit, overshot because nothing slowed the
    # approach, got braked back, fell under, and accelerated again. That is speed
    # hunting -- "rush to the limit, brake, go over, brake" -- and it is a
    # property of a cap with no approach region, not of the model.
    #
    # Scale the ceiling down linearly over the last V_TAPER_KPH so available
    # acceleration reaches zero exactly AT the cap. The car then settles onto the
    # limit instead of colliding with it, and step 2's braking becomes a backstop
    # for descents rather than the normal way of arriving.
    #
    # Braking is untouched: this only ever lowers a POSITIVE ceiling.
    _head = _cap - v_kph
    if _head < V_TAPER_KPH:
        _amax = min(_amax, _amax * max(0.0, _head) / V_TAPER_KPH)
        if a > _amax:
            why = why or "approach %.0f" % _cap

    if a > _amax:
        a = _amax
        why = why or "accel cap %.2f" % _amax

    # 4. jerk limit, applied LAST so nothing above can step the output
    da = JERK_MAX_CMD * max(dt, 1e-3)
    if a > a_prev + da:
        a = a_prev + da
        why = why or "jerk"
    elif a < a_prev - da:
        a = a_prev - da
        why = why or "jerk"
    return a, why


# How many 0x243 sends pass before can_thread takes the link for a recv. 1 is the
# original one-recv-per-send behaviour and the default, so this changes nothing
# unless asked for. 2 is what makes 100 Hz viable -- see the long note in
# can_thread. Raising it trades cs_can freshness for tx-stream regularity.
RECV_EVERY = max(1, int(os.environ.get("OP_RECV_EVERY", 1)))

# OP_STEER_DELAY is read once at startup and is the ONE place the lateral delay
# is set -- op_stream derives LAT_ACTION_T from the same variable, so the model
# and the controller always describe the same physical lag. A live /tune
# endpoint existed briefly and was removed; set it on the command line.
STEER_DELAY = float(os.environ.get("OP_STEER_DELAY", 0.20))
# op_stream builds LAT_ACTION_T as STEER_DELAY + 0.05 + 0.025; keep the offset
# here so the value passed to the model matches what op_stream would compute.
LAT_ACTION_T_OFFSET = 0.075


def find_panda_port():
    """Resolve the panda by DEVICE IDENTITY, not by enumeration order.

    /dev/ttyACM<n> is assigned in probe order, so plugging in any other CDC-ACM
    device can take ttyACM0 and push the panda to ttyACM1. That failure is
    silent and expensive: SerialPanda opens the wrong device, writes are
    accepted by whatever is there, tx_frames climbs, and every read returns
    nothing -- panda_safety_mode -1, cam_seen 0, eps_seen 0. Worse, the panda
    sits INLINE between the forward camera and the car, so a panda that is
    never driven stops forwarding and the dash raises the front camera sensor
    fault. Observed 2026-08-01 with an Arduino Nano ESP32 on ttyACM0.

    /dev/serial/by-id/ names are built from the USB vendor/product/serial, so
    they follow the hardware rather than the probe order. OP_PANDA_PORT
    overrides for the odd case (a second panda, a bench rig).
    """
    env = os.environ.get("OP_PANDA_PORT")
    if env:
        return env
    import glob as _glob
    for pat in ("/dev/serial/by-id/*STM32_STLink*",
                "/dev/serial/by-id/*comma*", "/dev/serial/by-id/*panda*"):
        hits = sorted(_glob.glob(pat))
        if hits:
            dev = os.path.realpath(hits[0])
            print("panda: %s -> %s" % (os.path.basename(hits[0]), dev))
            return dev
    print("panda: no by-id match, falling back to /dev/ttyACM0 -- if nothing is "
          "received (cam_seen 0, eps_seen 0, panda_safety_mode -1) this is why")
    return "/dev/ttyACM0"


SAFETY_MAZDA = 13         # CarParams.SafetyModel.mazda -- enforced in panda firmware
# The seven frames the radar sends besides 0x21b/0x21c. _STATIC ones are expected
# to be constant; DISTANCE/TURN carry real obstacle data and must NOT be
# fabricated -- faking those would let the car believe AEB is working on invented
# measurements, which is worse than the warning it currently shows.
RADAR_IDS = (0x361, 0x362, 0x363, 0x364, 0x365, 0x366, 0x499)
# Where the panda role publishes its own status for the model role's web UI.
# See the writer at the 1 Hz diagnostic tick. Not /tmp: systemd-tmpfiles wipes
# it at boot, and a status file that vanishes is worse than one that is stale,
# because a missing file reads as 'no panda role' rather than 'panda role from
# an earlier boot'.
PANDA_STATUS_FILE = os.environ.get(
    "OP_PANDA_STATUS_FILE", "/home/tran/drivelogs/panda_status.json")
# Of those seven, these five carry a 4-bit rolling counter and the other two do
# not. Verified against mazda_2017.dbc: 0x361 is CTR 56|4@1+ (Intel) and
# 0x362-0x365 are CTR 59|4@0+ (Motorola) -- different byte orders that both
# resolve to the LOW NIBBLE OF BYTE 7, so one rule covers all five. 0x366 has no
# CTR signal at all and 0x499 has no signals defined, so both replay verbatim.
#
# None of the seven carries a checksum. That is the single fact that makes replay
# worth attempting: there is no reverse-engineered formula to get wrong, only a
# counter to keep moving.
RADAR_CTR_IDS = (0x361, 0x362, 0x363, 0x364, 0x365)
# Of the seven, the two the DBC defines no varying signals for. Replaying these
# indefinitely asserts nothing that can go stale, so the shadow age gate skips
# them; every other address is treated as carrying road content that expires.
RADAR_STATIC_IDS = (0x366, 0x499)
# How long a captured radar frame may keep being replayed. Long enough to cover
# the arming sequence and a normal pull-away, short enough that a frozen lead
# does not survive into the drive it would be contradicting. Override to 0 to
# disable the replay entirely, or to a large number for the old behaviour.
SHADOW_MAX_AGE_S = float(os.environ.get("OP_SHADOW_MAX_AGE_S", "20"))
# The addresses the suppression census reports on: the three longitudinal
# frames, the two the panda gates engagement on, and the radar's other seven.
# One line per second answering "what did suppression actually silence".
CENSUS_IDS = (0x21b, 0x21c, 0x21f, 0x165, 0x09d,
              0x361, 0x362, 0x363, 0x364, 0x365, 0x366, 0x499)
# Read ONCE at import so long_tx_thread and main() cannot disagree: the thread is
# defined long before main() computes its own copy, and a closure over a
# later-assigned local would raise rather than read False.
_radar_local_shadow = os.environ.get("OP_ALPHA_LOCAL_SHADOW") == "1"
RADAR_NAMES = {0x361: "DISTANCE", 0x362: "TURN", 0x363: "363", 0x364: "364",
               0x365: "365", 0x366: "366_STATIC", 0x499: "499_STATIC"}
# See the block in carstate_thread() where this is applied.
STEER_ANGLE_SIGN = float(os.environ.get("OP_STEER_ANGLE_SIGN", "1"))
CAM_LKAS_ADDR = 0x243     # Mazda GEN1 steering command frame, bus 0
CAM_LANEINFO_ADDR = 0x440 # Mazda lane/HUD frame -- FORWARDED BY FIRMWARE, see below

# CAM_LKAS.STEERING_ANGLE is a raw 12-bit field in the DBC (factor 1, offset
# -2048) with no documented unit. The car's own STEER (0x82) message uses
# 0.05 deg/bit, so that is the assumed scale here. It is NOT verified against
# this camera -- see --steer-angle in the arg parser before enabling it.
LKAS_ANGLE_DEG_PER_BIT = 0.05
LKAS_ANGLE_RAW_LIMIT = 2047

def check_safety_limits(lkas_hz, params):
    """Prove the sender's limits agree with the panda's BEFORE transmitting.

    Three constants are duplicated across Python and C -- STEER_MAX/max_torque,
    STEER_DELTA_UP/max_rate_up, STEER_DELTA_DOWN/max_rate_down -- plus max_rt_delta
    which only exists on the C side but bounds what max_rate_up may be. If the
    sender's ceiling or ramp is the higher of any pair, the panda rejects the
    frame, zeroes desired_torque_last, and every following frame fails the rate
    check too: 0x243 stops reaching the bus entirely and the EPS raises the front
    LKAS fault. That failure is total and it looks like a hardware problem, so it
    is worth ten lines to catch it at startup instead of at 60 kph.

    Returns a list of complaint strings, empty if consistent. Reads the mazda.h
    SOURCE, which is what the firmware is built from -- it cannot see what is
    actually flashed, so it catches drift in the tree, not a stale flash.
    """
    import re
    try:
        import opendbc
        h = os.path.join(os.path.dirname(opendbc.__file__), "safety", "modes", "mazda.h")
        with open(h) as f:
            src = f.read()
    except Exception as e:
        return ["could not read mazda.h to verify panda limits (%s)" % e]

    fw = {}
    for key in ("max_torque", "max_rate_up", "max_rate_down", "max_rt_delta"):
        m = re.search(r"\.%s\s*=\s*(-?\d+)" % key, src)
        if m:
            fw[key] = int(m.group(1))
    missing = [k for k in ("max_torque", "max_rate_up", "max_rate_down", "max_rt_delta")
               if k not in fw]
    if missing:
        return ["could not parse %s from mazda.h" % ", ".join(missing)]

    out = []
    for py_name, fw_name in (("STEER_MAX", "max_torque"),
                             ("STEER_DELTA_UP", "max_rate_up"),
                             ("STEER_DELTA_DOWN", "max_rate_down")):
        py_val = getattr(params, py_name)
        if py_val != fw[fw_name]:
            out.append("%s=%d but mazda.h .%s=%d -- panda will REJECT every frame "
                       "above the lower of the two" % (py_name, py_val, fw_name, fw[fw_name]))
    # max_rt_delta is a cap per 250 ms window (MAX_RT_INTERVAL) whatever the rate,
    # so it is the bound that decides how fast the ramp may legally be. This is
    # the check that catches "raised --lkas-hz and forgot the firmware".
    msgs = lkas_hz * 0.250
    need = msgs * fw["max_rate_up"]
    if need > fw["max_rt_delta"]:
        out.append("at %.0f Hz the 250ms window holds %.1f messages x %d/msg = %.0f counts, "
                   "over mazda.h .max_rt_delta=%d -- raise max_rt_delta or lower the rate"
                   % (lkas_hz, msgs, fw["max_rate_up"], need, fw["max_rt_delta"]))
    elif need > 0.85 * fw["max_rt_delta"]:
        out.append("at %.0f Hz the ramp uses %.0f of %d max_rt_delta (%.0f%%) -- little "
                   "margin for tx jitter, watch lkas_worst_ms"
                   % (lkas_hz, need, fw["max_rt_delta"], 100.0 * need / fw["max_rt_delta"]))
    return out


STATE = {
    "t": 0.0, "frames": 0, "fps": 0.0, "n_dets": 0, "n_placed": 0,
    "v_ego_kph": 0.0, "rpm": 0,
    # Posted limit and the target derived from it (OP_SPEED_LIMIT=1). None until
    # the first successful GPS+OSM lookup, which is what the HUD draws "--" for.
    # speed_limit_inferred marks a limit guessed from the highway type rather
    # than an explicit maxspeed tag -- ~89% of ways in the extract -- and the HUD
    # shows those differently so a guess never looks like a posting.
    "speed_limit_kph": None, "speed_limit_inferred": False,
    "speed_target_kph": None, "speed_limit_mode": "",
    "cruise_available": False, "cruise_enabled": False,
    # Default so the page renders the stock cruise fields until a panda role that
    # actually knows the mode overwrites it. See the note in the status writer.
    "alpha_long": False,
    "brake": False, "steer_torque": 0.0, "steer_pressed": False,
    "curvature": 0.0, "path_reach": 0.0,
    # Which driving model is loaded, and whether it has an action head. "action
    # head" means desiredCurvature is a trained model output rather than something
    # reconstructed from the plan's yaw columns -- a different quantity with the
    # same name, so it belongs in the telemetry next to the value itself.
    "model": "-", "action_head": False,
    # The head's own two numbers, m/s^2. lat_accel is pre-division by v_ego^2:
    # compare it to 3.0, the lateral accel clip_curvature allows.
    "lat_accel": 0.0, "desired_accel": 0.0, "should_stop": False,
    # camera's lateral position between the ego lane lines, + = right of centre
    "lane_off_m": None, "lane_off_near_m": None, "lane_width_m": None,
    "lane_p_left": None, "lane_p_right": None,
    "calib_rpy": [0.0, 0.0, 0.0], "calib_valid": False,
    "calib_samples": 0, "calib_converged": False, "calib_file": False,
    # the calibrator's own word ("uncalibrated"/"calibrated"/"invalid"), not a
    # re-derivation from valid_blocks -- "invalid" means out of PITCH/YAW_LIMITS,
    # which block count alone cannot tell you.
    "calib_status": "-",
    "op_enabled": False, "op_state": "-", "lat_active": False,
    # events is EVERY onroad event; events_blocking is the subset that actually
    # keeps openpilot from engaging or drops it out (noEntry/soft/immediate).
    "would_command_torque": 0.0, "events": [], "events_blocking": [],
    "moving_seconds": 0.0, "note": "starting up",
    "armed": False, "tx_frames": 0, "applied_torque": 0, "controls_allowed": False,
    # headroom counters, filled by the tx thread (see the overlay's last line)
    "steer_max": 0, "applied_peak": 0, "want_peak": 0,
    "lkas_hz": 0.0, "ramp_cps": 0.0, "cam_trq_peak": 0,
}
HEALTH_FMT = '<IIIIIIIIBBBBBHBBBHfBBHBHHB'   # struct health_t, board/health.h
# seq is bumped every time buf is replaced, so /stream.mjpg can send each frame
# exactly once instead of re-sending whatever is current on a fixed timer.
FRAME_JPEG = {"buf": None, "seq": 0}
# The Tesla-style 3D scene render (tesla_view.py), served on /scene.mjpg. Same
# seq-bump contract as FRAME_JPEG so the MJPEG loop can dedupe it identically.
SCENE_JPEG = {"buf": None, "seq": 0}
# The same render as a raw BGR frame. --display composites this into the window
# directly; JPEG-encoding it there would be pure waste.
SCENE_FRAME = {"img": None, "seq": 0}
DEPTH_FRAME = {"img": None, "raw": None, "seq": 0}
# Top-down occupancy, from occupancy.FreeSpace. Its own buffer rather than a
# corner of the scene image: the 3D view is a perspective render and this is a
# metric plan view, so compositing them would mean one of the two lying about
# its own geometry.
OCC_FRAME = {"img": None, "seq": 0}
LOCK = threading.Lock()

PAGE = b"""<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>dashcam</title><style>
body{margin:0;background:#111;color:#eee;font:14px -apple-system,system-ui,sans-serif}
img{width:100%;display:block;background:#000}
.wrap{max-width:760px;margin:0 auto}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;padding:8px}
.card{background:#1c1c1e;border-radius:10px;padding:10px}
.k{color:#8e8e93;font-size:11px;text-transform:uppercase;letter-spacing:.5px}
.v{font-size:20px;font-weight:600;margin-top:2px;font-variant-numeric:tabular-nums}
.full{grid-column:1/-1}
.ok{color:#30d158}.bad{color:#ff453a}.warn{color:#ffd60a}
.bar{height:6px;background:#333;border-radius:3px;overflow:hidden;margin-top:6px}
.bar>i{display:block;height:100%;background:#30d158;width:0%}
small{color:#8e8e93}
</style></head><body><div class=wrap>
<img src="/stream.mjpg" alt="model view">
<img src="/scene.mjpg" alt="3D scene">
<div class=grid>
 <div class="card full"><div class=k>calibration</div>
   <div class=v id=cal>-</div><div class=bar><i id=calbar></i></div>
   <small id=calnote></small></div>
 <div class=card><div class=k>speed</div><div class=v id=v>-</div></div>
 <div class=card><div class=k>rpm</div><div class=v id=rpm>-</div></div>
 <div class=card><div class=k>cruise</div><div class=v id=crz>-</div></div>
 <div class=card><div class=k>openpilot</div><div class=v id=op>-</div></div>
 <div class=card><div class=k>curvature</div><div class=v id=curv>-</div></div>
 <div class=card><div class=k>would-command torque</div><div class=v id=trq>-</div></div>
 <div class="card full"><div class=k>model</div><div class=v id=mdl>-</div>
   <small id=mdlnote></small></div>
 <div class="card full"><div class=k>transmit</div><div class=v id=tx>-</div>
   <small id=txnote></small></div>
 <div class="card full"><div class=k>events</div><div id=ev><small>-</small></div></div>
 <div class="card full"><div class=k>torque headroom</div><div><small id=hdr>-</small></div></div>
 <div class="card full"><div class=k>EPS response (0x241)</div><div class=v id=eps>-</div>
   <small id=epsnote></small></div>
 <div class="card full"><div class=k>status</div><div id=note><small>-</small></div>
   <small id=stats></small></div>
</div></div><script>
async function tick(){
 try{const s=await (await fetch('/status.json',{cache:'no-store'})).json();
  const rpy=s.calib_rpy.map(x=>(x*57.2958).toFixed(2)).join(', ');
  document.getElementById('cal').textContent=rpy+' deg';
  document.getElementById('cal').className='v '+(s.calib_converged?'ok':(s.calib_samples>0?'warn':'bad'));
  const pct=Math.min(100,100*s.calib_samples/5.0);   // INPUTS_NEEDED=5 blocks
  document.getElementById('calbar').style.width=pct+'%';
  document.getElementById('calnote').textContent=
    s.calib_converged?('converged - saved to disk: '+s.calib_file)
    :(s.calib_samples>0?('learning: '+s.calib_samples+'/5 blocks (100 frames each, needs >15 km/h)')
    :'no blocks yet - only learns above 15 km/h');
  document.getElementById('v').textContent=s.v_ego_kph.toFixed(1)+' kph';
  document.getElementById('v').className='v '+(s.v_ego_kph>1?'ok':'');
  document.getElementById('rpm').textContent=s.rpm;
  // UNDER ALPHA LONG, cruise_available/cruise_enabled DESCRIBE OUR OWN FRAME.
  // They decode from 0x21c on bus 0, and once the radar is suppressed that is the
  // frame WE generate -- so they report what we put in it, not what the driver
  // pressed. Reported as "cruise is always off in the web while i did turn it on"
  // with the panda log showing acc_armed=1, op=enabled, ctrl_allowed=1 at the same
  // instant. acc_armed is the field that follows the button in this mode, so
  // prefer it and say which one is being shown.
  // ...AND UNDER ALPHA LONG cruise_enabled IS NOT MERELY WRONG, IT IS FROZEN.
  //
  // The `_act || s.cruise_enabled` fallback below was the bug behind "why is
  // cruise state stale". Both cruise_available and cruise_enabled decode from
  // 0x21c, the radar's frame. Suppress the radar and NOTHING on bus 0 sends it
  // any more -- our own copies come back tagged (bus + 128) and cs_can.update
  // only decodes bus == 0 -- so the two fields keep whatever value they last
  // saw, for the rest of the drive.
  //
  // MEASURED 2026-08-11 over a full alpha-long run: 175/175 samples read
  // `crz_avail=0 crz_en=1`, which is self-contradictory (active but not
  // available) and is the signature of a field nothing is writing. With
  // cruise_enabled latched at 1, `_act || s.cruise_enabled` is permanently
  // true, so the page showed ENGAGED (long) forever -- while acc_armed and
  // acc_active, the fields that actually follow the car, were both false.
  //
  // So under alpha long use ONLY the PEDALS-derived pair. acc_active is the
  // PCM's own report of stock ACC, acc_armed is MRCC MAIN, and both keep
  // updating with the radar muted -- which is exactly why mazda.h gates alpha
  // long on PEDALS too. Outside alpha long the radar is alive and 0x21c is
  // trustworthy, so nothing changes there.
  var _al = !!s.alpha_long, _armed = !!s.acc_armed, _act = !!s.acc_active;
  var _on  = _al ? _act : s.cruise_enabled;
  var _av  = _al ? _armed : s.cruise_available;
  // Name the source. A frozen field that LOOKS live is what cost the time here,
  // so say which signal the readout is coming from rather than implying the
  // radar-sourced one is still meaningful.
  document.getElementById('crz').textContent =
      (_on?'ENGAGED':(_av?(_al?'armed':'available'):'off')) + (_al?' (PEDALS, radar muted)':'');
  document.getElementById('crz').className='v '+(_on?'ok':(_av?'warn':''));
  document.getElementById('op').textContent=s.op_state+(s.lat_active?' / lat':'');
  document.getElementById('op').className='v '+(s.op_enabled?'ok':'');
  document.getElementById('curv').textContent=s.curvature.toFixed(5);
  document.getElementById('trq').textContent=s.would_command_torque.toFixed(3);
  // The action head, in its own units. lat_accel is what the model asked for
  // before v_ego^2 was divided out of it, so it can be read straight against the
  // 3.0 m/s^2 clip_curvature ceiling -- amber once the command is inside 20% of
  // the limit the controller will silently clip it to.
  document.getElementById('mdl').textContent=
    s.action_head
      ? ('lat '+s.lat_accel.toFixed(2)+' m/s2   long '+s.desired_accel.toFixed(2)
         +' m/s2'+(s.should_stop?'   STOP':''))
      : 'plan-derived (no action head)';
  document.getElementById('mdl').className='v '+(Math.abs(s.lat_accel)>2.4?'warn':'');
  document.getElementById('mdlnote').textContent=
    s.model+(s.action_head?' - curvature is a model output, divided by v_ego^2'
                          :' - curvature reconstructed from the plan yaw');
  const txel=document.getElementById('tx');
  if(!s.armed){txel.textContent='SILENT (not transmitting)';txel.className='v';
    document.getElementById('txnote').textContent='dashcam mode - start with --arm to transmit';}
  else{txel.textContent='ARMED  applied '+s.applied_torque+'  ('+s.tx_frames+' frames sent)';
    txel.className='v '+(s.controls_allowed?'bad':'warn');
    document.getElementById('txnote').textContent=
      s.controls_allowed?'panda controls ALLOWED - steering frames are actuating'
      :'panda controls blocked - engage cruise for torque to pass';}
  document.getElementById('ev').innerHTML='<small>'+(s.events.length?s.events.join(', '):'-')+'</small>';
  document.getElementById('note').innerHTML='<small>'+s.note+'</small>';
  document.getElementById('stats').textContent=
    s.frames+' frames | '+s.fps.toFixed(1)+' fps | moving '+s.moving_seconds.toFixed(0)+'s | '+s.t.toFixed(0)+'s elapsed';
  // Headroom line. peak/STEER_MAX is the question that decides whether raising
  // STEER_MAX does anything: pinned at 100% means the ceiling binds, well under
  // means the ramp or the tune binds and a higher ceiling changes nothing.
  document.getElementById('hdr').textContent=
    'peak applied '+s.applied_peak+'/'+s.steer_max
    +' ('+(s.steer_max?Math.round(100*s.applied_peak/s.steer_max):0)+'%)'
    +' | wanted '+s.want_peak
    +' | tx '+s.lkas_hz.toFixed(1)+' Hz -> ramp '+s.ramp_cps+' counts/s'
    // Regularity, not just rate: worst >> target means the stream is bursty and
    // a higher --lkas-hz is making things worse, not better.
    +' | gap '+s.lkas_worst_ms+'/'+s.lkas_target_ms+' ms worst'
    +' (run '+s.lkas_worst_ms_all+', '+s.lkas_late+' late)'
    +' | stock cam peak '+s.cam_trq_peak;
  // EPS response. eps_effective is the rack's OWN report of the torque it
  // applied; eps_request is what it says it received. Divergence = the EPS is
  // clamping, and that is the one ceiling no software change can lift.
  const epsel=document.getElementById('eps');
  if(!s.eps_seen){epsel.textContent='no 0x241 yet';epsel.className='v';
    document.getElementById('epsnote').textContent='EPS feedback frame not seen - engine off, or not forwarded';}
  else{epsel.textContent='req '+s.eps_request+'  ->  applied '+s.eps_effective
        +'   motor '+s.eps_motor_torque;
    // Only call it a clamp once we have asked for more than the stock camera
    // ever does; below that a low follow ratio says nothing about the ceiling.
    const tested=s.eps_req_peak>s.cam_trq_peak;
    epsel.className='v '+(!tested?'':(s.eps_follow>0.9?'ok':'warn'));
    document.getElementById('epsnote').textContent=
      'peak req '+s.eps_req_peak+' -> applied '+s.eps_eff_peak
      +' (follow '+(s.eps_follow<0?'-':s.eps_follow)+')'
      +(tested?'':'  [never asked past the stock camera peak '+s.cam_trq_peak+' - EPS ceiling UNTESTED]')
      +(s.lkas_block?'  | LKAS_BLOCK':'')+(s.hands_off_5s?'  | HANDS-OFF 5s':'');}
 }catch(e){}
 setTimeout(tick,400);
}tick();
</script></body></html>"""


class ReusableHTTPServer(ThreadingHTTPServer):
    # Without this a restart fails with "Address already in use" for the ~60s the
    # old socket sits in TIME_WAIT -- which on a car you would hit every time you
    # stop and restart the logger.
    allow_reuse_address = True
    daemon_threads = True


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/status.json"):
            with LOCK:
                # skip private keys (_path holds a numpy array, which is not
                # JSON-serialisable and would 500 the status endpoint)
                body = json.dumps({k: v for k, v in STATE.items()
                                   if not k.startswith('_')}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/stream.mjpg") or self.path.startswith("/scene.mjpg"):
            src = SCENE_JPEG if self.path.startswith("/scene.mjpg") else FRAME_JPEG
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            # Send each frame ONCE. The producer runs at ~6fps (time.sleep(0.15)
            # in the preview thread) but this loop used to write whatever was
            # current every 60ms, so ~2 of every 3 frames on the wire were a
            # byte-identical duplicate -- measured 1.46 MB/s where ~0.5 MB/s of
            # real frames existed. That is pure waste on the LAN and enough to
            # stall the stream over a tunnel, where every packet is encrypted
            # and the MTU is 1280.
            #
            # Poll faster than the producer (20ms) so dropping the duplicates
            # does not add latency: a new frame goes out within 20ms of being
            # encoded, vs up to 60ms before.
            KEEPALIVE_S = 2.0
            last_seq = -1
            last_send = time.monotonic()
            try:
                while True:
                    with LOCK:
                        buf, seq = src["buf"], src["seq"]
                    now = time.monotonic()
                    # Resend the last frame if the producer has gone quiet, so an
                    # idle connection is not dropped by the browser or a proxy.
                    # A stalled camera then costs 0.5fps of duplicates, not 16.7.
                    stale = now - last_send >= KEEPALIVE_S
                    if buf is not None and (seq != last_seq or stale):
                        self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: " + str(len(buf)).encode() + b"\r\n\r\n")
                        self.wfile.write(buf)
                        self.wfile.write(b"\r\n")
                        last_seq, last_send = seq, now
                    time.sleep(0.02)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)


# ---- model rendering --------------------------------------------------------
# supercombo emits positions in the CALIBRATED frame: x forward, y right, z down,
# origin at the camera. op_frame builds the model INPUT by warping the camera
# through (intrinsics x view_from_calib); projecting the OUTPUT back is that same
# chain run forwards. So these overlays are not an artist's impression -- a lane
# line drawn on the tarmac means the model placed it on the tarmac, and a lane
# line floating off the road is a calibration error you can now see directly.
#
# The previous version faked this with `x = w/2 - y*12, y = h - 10 - x*3.2`, a
# scaled top-down plot with no perspective at all: correct only at one distance
# and increasingly wrong everywhere else.
sys.path.insert(0, "/home/tran/openpilot_jetson/modeld")
try:
    from op_frame import camera_intrinsics, get_view_frame_from_calib_frame
    from constants import ModelConstants as _MC
    X_IDXS = np.array(_MC.X_IDXS, dtype=np.float32)
    _PROJ_OK = True
except Exception as _e:                                   # pragma: no cover
    _PROJ_OK = False
    print(f"model overlay DISABLED (projection import failed): {_e}")

_K_CACHE = {}
PATH_HALF_W = 0.9          # metres either side of the path centreline

# Fraction of a lane line's near-field vertices that must land on segmented paint
# before the line is drawn in the 3D view. Deliberately low: lane markings are
# DASHED, so even a perfectly placed line only sits on paint for roughly half its
# length, and the 96x96 seg grid loses more. What it has to separate is "some paint
# along this line" from "no paint anywhere along it", which is the difference
# between a real lane boundary and one of supercombo's two spare lines.
SEG_AGREE = 0.22

# Fraction of a detector box that the segmentation must label with the matching
# class before the box is believed. Low on purpose: the box is a rectangle around a
# rotated 3D object, so even a perfect mask fills only part of it, and the 96x96 grid
# loses the edges. This separates "the seg sees something here" from "the seg sees
# nothing here at all", which is what a false positive looks like.
SEG_BOX_MIN = 0.12

# Most objects a frame will have their heading measured from depth. See the note
# at the call site: it is a per-object cost on the same CPU that draws the scene.
YAW_MAX_OBJ = 8

# Depth-scale residual above which the voxel field refuses to place anything. The
# free-space carve uses 25%; voxels get the same number for the same reason -- both
# are absolute positions, and a bad scale stands a tree at the wrong distance
# rather than merely mis-sizing it.
VOXEL_MAX_RESID = 0.25


def _intrinsics(draw_w, draw_h, src_w, src_h):
    """K for a frame that was CAPTURED at src and is being DRAWN at draw size.

    The focal must come from the capture mode and then be scaled by the resize --
    calling camera_intrinsics(960, 540) instead would miss the mode table and fall
    back to scaling the full-sensor focal by width, giving 503 px against the true
    531, i.e. a 5% error that bends every projected lane line outward."""
    key = (draw_w, draw_h, src_w, src_h)
    K = _K_CACHE.get(key)
    if K is None:
        K = camera_intrinsics(src_w, src_h).copy()
        s = draw_w / float(src_w)
        K[:2, :] *= s
        _K_CACHE[key] = K
    return K


def _project(pts_calib, w, h, rpy, src_wh=None):
    """(N,3) calibrated-frame metres -> (N,2) pixels, plus a validity mask."""
    src_w, src_h = src_wh if src_wh else (w, h)
    K = _intrinsics(w, h, src_w, src_h)
    # height=0: the model's points are already relative to the camera origin, so
    # the road-plane offset that get_view_frame_from_calib_frame can add here
    # would double-count the camera height.
    P = K @ get_view_frame_from_calib_frame(rpy[0], rpy[1], rpy[2], 0.0)
    n = len(pts_calib)
    uvw = np.hstack([pts_calib, np.ones((n, 1), np.float32)]) @ P.T
    z = uvw[:, 2]
    ok = z > 0.5                                    # strictly in front of the lens
    uv = np.zeros((n, 2), np.float32)
    uv[ok] = uvw[ok, :2] / z[ok, None]
    # in-front is not enough: a point just past the lens projects to a coordinate
    # thousands of pixels off-frame, and cv2.line will happily rasterise that.
    ok &= (np.abs(uv[:, 0]) < 4 * w) & (np.abs(uv[:, 1]) < 4 * h)
    return uv, ok


def _blit_pip(dst, src, frac=0.34, margin=24, corner="br", label=None):
    """Drop an inset into a corner of the camera frame.

    Two insets now share the frame -- the 3D scene bottom-right, depth bottom-left --
    so the corner is a parameter. Both stay clear of the HUD text along the top.
    """
    h, w = dst.shape[:2]
    tw = int(w * frac)
    th = int(tw * src.shape[0] / src.shape[1])
    if tw < 8 or th < 8 or th + margin >= h or tw + margin >= w:
        return
    x0 = margin if corner.endswith("l") else w - tw - margin
    y0 = margin if corner.startswith("t") else h - th - margin
    dst[y0:y0 + th, x0:x0 + tw] = cv2.resize(src, (tw, th), interpolation=cv2.INTER_AREA)
    cv2.rectangle(dst, (x0 - 1, y0 - 1), (x0 + tw, y0 + th), (90, 90, 96), 2)
    if label:
        cv2.putText(dst, label, (x0 + 6, y0 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(dst, label, (x0 + 6, y0 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1, cv2.LINE_AA)


def _to_view(xyz, z_road):
    """openpilot calibrated frame -> tesla_view frame.

    The model works in comma's device frame: x FORWARD, y RIGHT, z DOWN, origin
    at the CAMERA. (device_frame_from_view_frame in op_frame maps camera-right to
    device y and camera-down to device z; probing _project confirms it -- y=+3
    lands right of centre, and the road only appears below the horizon at z=+1.25.)

    tesla_view draws in the natural graphics frame: y LEFT, z UP, origin on the
    ROAD, because that is what its meshes are built in. So flip y, flip z, and
    shift by the road height. z_road is measured from the lane points themselves
    rather than hardcoded, so it tracks mount height and load with no constant.
    """
    out = np.empty_like(xyz, dtype=np.float32)
    out[:, 0] = xyz[:, 0]
    out[:, 1] = -xyz[:, 1]
    out[:, 2] = z_road - xyz[:, 2]
    return out


def _road_z(lanes_xyz):
    """Camera height above the road, from the road lines' own z."""
    zs = [np.asarray(a, np.float32)[:, 2] for a in lanes_xyz if a is not None and len(a)]
    return float(np.median(np.concatenate(zs))) if zs else 1.25


class _AsPlacement:
    """A tracked object presented as a lane_place.Placement.

    occupancy.footprint_yaw wants the three fields it uses off a Placement -- the
    range it scales the lateral ruler by, the lateral offset it measures from, and
    the tangent it breaks the 180-degree ambiguity with. A confirmed track has all
    three and is the better source, because its position is filtered and its box is
    the one actually being drawn. Constructing this beats widening footprint_yaw's
    signature to take three loose floats and losing the name of what they are.
    """
    __slots__ = ('x', 'y', 'yaw')

    def __init__(self, x, y, yaw):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)


def _path_yaw_at(path, x):
    """Heading of the planned path at distance x, in radians.

    A lead is a point -- the model gives no orientation for it -- so the 3D model
    has to be aimed at something. The path tangent is the right choice for an
    in-path vehicle: it is by definition travelling the way the road goes.
    """
    if path is None or len(path) < 2:
        return 0.0
    p = np.asarray(path, np.float32)
    i = int(np.clip(np.searchsorted(p[:, 0], x), 1, len(p) - 1))
    dx = float(p[i, 0] - p[i - 1, 0])
    return float(np.arctan2(float(p[i, 1] - p[i - 1, 1]), dx)) if abs(dx) > 1e-6 else 0.0


def _yz_to_xyz(yz):
    """lane_lines/road_edges are (33,2) of (y,z); x is the fixed X_IDXS grid."""
    out = np.empty((len(yz), 3), np.float32)
    out[:, 0] = X_IDXS[:len(yz)]
    out[:, 1] = yz[:, 0]
    out[:, 2] = yz[:, 1]
    return out


# lane_lines index 1 / 2 are the two lines of the EGO lane (0 and 3 are the
# adjacent lanes' far sides). lane_lines_prob packs two values per line and
# openpilot reads [1::2], same as draw_model above.
LL_LEFT, LL_RIGHT = 1, 2
NEAR_IDX = 6            # X_IDXS[6] = 6.8 m -- close enough to be "here", far
                        # enough that the lines are observed rather than
                        # extrapolated off the bottom of the frame.


def lane_offset(lanes, probs):
    """Where the CAMERA sits between the ego lane lines, in metres.

    Returns (offset_0m, offset_near, width, p_left, p_right) or None.
    Sign follows the calibrated frame: y is positive to the RIGHT, so a positive
    offset means the camera is right of lane centre. The car's centre is that
    plus however far the camera is mounted off the vehicle centreline -- which
    nothing in this stack models, so it has to be measured with a tape.

    This is the number that decides WHY the car is off centre. The model steers
    on its own plan, and its plan is centred on the camera; if this reads ~0
    while the car is visibly not centred, the model is doing its job and the
    error is in the camera geometry (mount offset, or an unlearned yaw). If it
    reads off centre, the model itself is placing the car there.
    """
    if lanes is None or len(lanes) <= LL_RIGHT:
        return None
    p = np.asarray(probs)[1::2] if probs is not None and len(probs) >= 2 * len(lanes) else None
    yl = float(np.asarray(lanes[LL_LEFT])[:, 0][0])
    yr = float(np.asarray(lanes[LL_RIGHT])[:, 0][0])
    yln = float(np.asarray(lanes[LL_LEFT])[:, 0][NEAR_IDX])
    yrn = float(np.asarray(lanes[LL_RIGHT])[:, 0][NEAR_IDX])
    return (-0.5 * (yl + yr), -0.5 * (yln + yrn), yrn - yln,
            float(p[LL_LEFT]) if p is not None else -1.0,
            float(p[LL_RIGHT]) if p is not None else -1.0)


def draw_model(img, st, src_wh=None):
    """comma-style render: filled path band, lane lines, road edges."""
    m = st.get("_model")
    if not _PROJ_OK or not m:
        return img
    path = m.get("path")
    h, w = img.shape[:2]
    rpy = st.get("calib_rpy") or (0.0, 0.0, 0.0)
    lay = img.copy()
    # uint8 alpha, not float32. The mask is written by cv2 draw calls and read by
    # one blend; doing either in float32 costs ~50 ms/frame at 1080p on this board
    # against ~12 ms for the cv2 integer path, for a max difference of 1 count.
    amask = np.zeros((h, w), np.uint8)
    bbox = []

    def _note(pts):
        bbox.append((pts[:, 0].min(), pts[:, 0].max(),
                     pts[:, 1].min(), pts[:, 1].max()))

    # --- driving path: a tapered band, brightest and most opaque up close ---
    if path is not None and len(path) > 1:
        left = path.astype(np.float32).copy(); left[:, 1] -= PATH_HALF_W
        right = path.astype(np.float32).copy(); right[:, 1] += PATH_HALF_W
        luv, lok = _project(left, w, h, rpy, src_wh)
        ruv, rok = _project(right, w, h, rpy, src_wh)
        for i in range(len(path) - 1):
            if not (lok[i] and lok[i + 1] and rok[i] and rok[i + 1]):
                continue
            quad = np.array([luv[i], luv[i + 1], ruv[i + 1], ruv[i]], np.int32)
            f = i / (len(path) - 1.0)                      # 0 near -> 1 far
            cv2.fillPoly(lay, [quad], (int(40 + 150 * f), 235, int(30 + 60 * f)))
            cv2.fillPoly(amask, [quad], int(255 * 0.50 * (1.0 - 0.80 * f)))
            _note(quad)

    # --- lane lines: opacity IS the model's confidence, so a line the model is
    # unsure about looks unsure rather than identical to one it is certain of ---
    lanes, probs = m.get("lanes"), m.get("lane_prob")
    if probs is not None and lanes is not None and len(probs) >= 2 * len(lanes):
        # lane_lines_prob packs TWO values per line (slice is 8 long for 4 lines).
        # openpilot's fill_model_msg takes [1::2] -- laneLineProbs = prob[0,1::2].
        # Reading it as one-per-line would colour each line by its neighbour's
        # confidence, which looks plausible and is wrong.
        probs = np.asarray(probs)[1::2]
    if lanes is not None:
        for j, ln in enumerate(lanes):
            p = float(probs[j]) if probs is not None and j < len(probs) else 1.0
            if p < 0.15:
                continue
            uv, ok = _project(_yz_to_xyz(np.asarray(ln, np.float32)), w, h, rpy, src_wh)
            for i in range(len(uv) - 1):
                if not (ok[i] and ok[i + 1]):
                    continue
                a, b = uv[i].astype(int), uv[i + 1].astype(int)
                cv2.line(lay, tuple(a), tuple(b), (255, 255, 255), 3, cv2.LINE_AA)
                cv2.line(amask, tuple(a), tuple(b), int(255 * np.clip(p, 0, 1)), 3, cv2.LINE_AA)
                _note(np.array([a, b]))

    # --- road edges: red, always solid; they bound where the car may go ---
    edges = m.get("edges")
    if edges is not None:
        for ed in edges:
            uv, ok = _project(_yz_to_xyz(np.asarray(ed, np.float32)), w, h, rpy, src_wh)
            for i in range(len(uv) - 1):
                if not (ok[i] and ok[i + 1]):
                    continue
                a, b = uv[i].astype(int), uv[i + 1].astype(int)
                cv2.line(lay, tuple(a), tuple(b), (60, 60, 255), 3, cv2.LINE_AA)
                cv2.line(amask, tuple(a), tuple(b), 217, 3, cv2.LINE_AA)
                _note(np.array([a, b]))

    if not bbox:
        return img
    # Blend only the region actually drawn. A full-frame 1080p alpha blend is
    # ~6M float ops per frame on the preview thread, which is real time on this
    # board; the geometry never covers more than the lower part of the frame.
    xs0 = max(0, int(min(b[0] for b in bbox)));  xs1 = min(w, int(max(b[1] for b in bbox)) + 1)
    ys0 = max(0, int(min(b[2] for b in bbox)));  ys1 = min(h, int(max(b[3] for b in bbox)) + 1)
    if xs1 <= xs0 or ys1 <= ys0:
        return img
    a3 = cv2.cvtColor(amask[ys0:ys1, xs0:xs1], cv2.COLOR_GRAY2BGR)
    img[ys0:ys1, xs0:xs1] = cv2.add(
        cv2.multiply(img[ys0:ys1, xs0:xs1], cv2.bitwise_not(a3), scale=1 / 255.0),
        cv2.multiply(lay[ys0:ys1, xs0:xs1], a3, scale=1 / 255.0))
    return img


def _hud_minimal(img, st):
    """Small bottom-left HUD: calibration state + what is blocking engagement.

    The full top-left readout is for the phone dashboard, where there is a whole
    page of context around it. On the car's own monitor the frame IS the display,
    so this keeps it to the two things you cannot infer by looking out of the
    windscreen: how far calibration has got, and why it will not engage.
    """
    h = img.shape[0]
    cal_ok = bool(st.get("calib_converged"))
    status = st.get("calib_status") or "-"
    lines = [(f"cal {status} {st.get('calib_samples', 0)}/5",
              (0, 255, 120) if cal_ok else (0, 200, 255))]
    if st.get("calib_clipped"):
        # The clipped value is a clamp, not a measurement -- without this the HUD
        # would show a plausible rpy that stops responding to physical aiming.
        raw = st.get("calib_rpy_raw_deg")
        lines.append((f"  CLIPPED raw {' '.join(f'{x:+.1f}' for x in raw)} deg"
                      if raw else "  CLIPPED", (0, 165, 255)))

    blk = st.get("events_blocking") or []
    if blk:
        # Names only, and only the blocking ones -- a scrolling wall of every
        # onroad event is exactly what this HUD is trying not to be.
        for name in blk[:3]:
            lines.append((f"blocked: {name}", (0, 80, 255)))
    elif st.get("lat_active"):
        lines.append(("lat ACTIVE", (0, 255, 120)))
    else:
        lines.append((f"idle  {st.get('op_state', '-')}", (200, 200, 200)))

    # bottom-left, growing upward so the last line always sits at a fixed margin
    y = h - 12 - 18 * (len(lines) - 1)
    for text, colour in lines:
        cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    colour, 1, cv2.LINE_AA)
        y += 18
    return img


DET_COLOUR = {'car': (80, 220, 255), 'truck': (80, 220, 255), 'bus': (80, 220, 255),
              'person': (236, 178, 120), 'cycle': (236, 178, 120),
              'traffic light': (120, 255, 160), 'stop sign': (80, 80, 255)}


def draw_dets(img, st, src_wh=None):
    """YOLO26 boxes, in SOURCE pixels, scaled to whatever we are drawing on.

    Stage 2 is deliberately 2D only: these boxes are not placed in the ego frame
    yet. Trusting a range derived from a detector we have not eyeballed would
    just move a detection bug into the geometry.
    """
    # Prefer the TRACKED boxes: they are the same ones the 3D meshes come from, so
    # the box and the mesh move together. Falling back to the raw detections keeps
    # --no-smooth working and covers the first frames before any track is confirmed.
    dets = st.get("_dets_smooth") or st.get("_dets")
    if not dets:
        return
    h, w = img.shape[:2]
    sx = w / float(src_wh[0]) if src_wh else 1.0
    sy = h / float(src_wh[1]) if src_wh else 1.0
    for d in dets:
        x1, y1, x2, y2 = d["box"]
        p1 = (int(x1 * sx), int(y1 * sy)); p2 = (int(x2 * sx), int(y2 * sy))
        col = DET_COLOUR.get(d["cls"], (200, 200, 200))
        cv2.rectangle(img, p1, p2, col, 2 if not d["trunc"] else 1)
        # the ground-contact point stage 3 will range from -- drawn so a wrong
        # one is visible now rather than as a mystery offset later
        cv2.circle(img, (int(0.5 * (p1[0] + p2[0])), p2[1]), 3, col, -1)
        cv2.putText(img, "%s %.2f%s" % (d["cls"], d["conf"], " T" if d["trunc"] else ""),
                    (p1[0], max(p1[1] - 5, 11)), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    col, 1, cv2.LINE_AA)


HOOD = None            # hood.HoodMask, loaded at startup; see hood.py
SEG_OVERLAY = {"on": False}
# Cached colour layer for draw_seg, keyed on the segmentation's sequence number.
# The seg changes 3 times a second and the display draws 30, so rebuilding per
# frame was 90% waste -- see the comment in draw_seg.
_SEG_LAYER = {"seq": None, "wh": None, "layer": None}


def draw_seg(img, st):
    """Tint the raw b2 class map onto the camera frame, in IMAGE space.

    This is the only way to judge the segmentation itself. Everywhere else the
    classes are already through the lane-relative map, so a bad-looking 3D view
    could be the seg being wrong OR the placement being wrong OR there being no
    road to place against -- three problems with three different fixes and no way
    to tell them apart. Drawn here, before any geometry, the question is just
    "did it label the pixels correctly".

    Nearest-neighbour upscale on purpose: it shows the true 96x96 grid rather than
    smoothing it into something that looks better resolved than it is.
    """
    cls = st.get("_seg")
    if cls is None:
        return
    from road_seg import GROUND, VERTICAL, LABELS
    h, w = img.shape[:2]
    # A DIAGNOSTIC palette, not the display one. The 3D view's greys are tuned so
    # drivable is near-white and everything else is a shade of grey; tinting an image
    # with that just washes it out -- vegetation alone can be half the frame. Here
    # the only job is telling classes apart, so use saturated hues.
    #
    # DERIVED from the taxonomy, not hardcoded: the ids moved when the segmenter was
    # swapped (Vistas 16 -> Cityscapes 19), and a hardcoded table would have silently
    # mislabelled every class while still looking plausible. Hues are spread evenly
    # and then a few are pinned by NAME so the important ones stay recognisable
    # whatever taxonomy is loaded.
    n = len(LABELS)
    hsv = np.zeros((1, n, 3), np.uint8)
    hsv[0, :, 0] = (np.arange(n) * (180 // max(n, 1)) + 7) % 180
    hsv[0, :, 1] = 235
    hsv[0, :, 2] = 255
    pal = np.zeros((256, 3), np.uint8)
    pal[:n] = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0]
    keep = np.zeros(256, bool)
    keep[:n] = True
    PIN = (('road', (255, 90, 0)), ('car-road', (255, 90, 0)),
           ('sidewalk', (255, 0, 190)), ('terrain', (60, 160, 60)),
           ('vegetation', (30, 90, 30)), ('building', (0, 130, 255)),
           ('road-marking', (255, 255, 255)), ('crosswalk', (0, 240, 255)),
           ('curb', (255, 200, 0)),
           ('car', (0, 0, 235)), ('vehicle', (0, 0, 235)), ('truck', (0, 60, 200)),
           ('bus', (0, 90, 170)), ('person', (200, 0, 255)), ('rider', (200, 0, 255)),
           ('traffic light', (0, 255, 255)), ('traffic sign', (0, 200, 255)),
           ('sky', (150, 148, 146)))
    for name, bgr in PIN:
        if name in LABELS:
            pal[LABELS.index(name)] = bgr
    drawn3d = set(GROUND) | set(VERTICAL)

    # Two things make this cheap enough to run under the display loop. The first
    # version cost 331 ms a frame -- ten cores at 30 fps -- because it gathered a
    # boolean mask over 2 M pixels into float arrays, and it rebuilt the whole layer
    # every displayed frame even though the segmentation only changes 3 times a
    # second.
    #   1. Palette-map at 96x96 and resize the RESULT, not the other way round.
    #   2. Cache that layer against the seg's sequence number, so the per-frame cost
    #      is one addWeighted in C.
    # Every class gets a colour, including sky, so no per-pixel mask is needed at
    # all -- the mask was most of the cost and sky is never the thing in question.
    seq = st.get("_seg_seq")
    c = _SEG_LAYER
    if c.get("seq") != seq or c.get("wh") != (w, h):
        c["layer"] = cv2.resize(pal[cls], (w, h), interpolation=cv2.INTER_NEAREST)
        c["seq"], c["wh"] = seq, (w, h)
    cv2.addWeighted(img, 0.62, c["layer"], 0.38, 0.0, dst=img)
    # legend: only the classes actually present, with their share of the frame
    frac = np.bincount(cls.ravel(), minlength=256) / float(cls.size)
    y = h - 12
    for cid in np.argsort(-frac):
        if frac[cid] < 0.01 or cid >= len(LABELS):
            continue
        # flag classes the 3D view will NOT show, so a class that is correct here
        # but missing there is obvious rather than confusing
        tag = "%s %.0f%%%s" % (LABELS[cid], frac[cid] * 100,
                               "" if cid in drawn3d else "  [not in 3D]")
        cv2.putText(img, tag, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, tag, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    tuple(int(v) for v in pal[cid]) if keep[cid] else (200, 200, 200),
                    2, cv2.LINE_AA)
        y -= 22
        if y < 60:
            break


def draw_speed_limit_sign(img, st, hud):
    """US-style speed-limit plate, top-right of the HUD.

    Drawn as a SIGN rather than another text row on purpose: the limit is the one
    number you check at a glance while driving, and a white plate in the corner is
    found by shape without reading. --display serves no web page, so this is the
    only place the posted limit is visible in the mode used on the road.

    An INFERRED limit -- guessed from the highway type because the way carries no
    maxspeed tag, which is ~89% of the New York extract -- is drawn amber with a
    '?' so it can never be mistaken for a real posting. SpeedTarget already
    discounts those; this makes the discount visible.
    """
    lim = st.get("speed_limit_kph")
    h, w = img.shape[:2]
    scale = max(0.7, min(1.6, hud * 0.55))
    pw, ph = int(150 * scale), int(190 * scale)
    x1, y1 = w - pw - int(16 * scale), int(16 * scale)
    x2, y2 = x1 + pw, y1 + ph

    inferred = bool(st.get("speed_limit_inferred"))
    border = (0, 170, 220) if inferred else (40, 40, 40)

    cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 255), -1)
    cv2.rectangle(img, (x1, y1), (x2, y2), border, max(2, int(5 * scale)))

    def _centre(text, cy, fs, thick, col):
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fs, thick)
        cv2.putText(img, text, (x1 + (pw - tw) // 2, cy + th // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, fs, col, thick, cv2.LINE_AA)

    _centre("SPEED", y1 + int(30 * scale), 0.5 * scale, max(1, int(2 * scale)), (20, 20, 20))
    _centre("LIMIT", y1 + int(58 * scale), 0.5 * scale, max(1, int(2 * scale)), (20, 20, 20))
    if lim is None:
        _centre("--", y1 + int(120 * scale), 1.7 * scale, max(2, int(4 * scale)), (90, 90, 90))
    else:
        # mph on the plate: this is a US sign and the extract is New York.
        _centre("%.0f" % (lim * 0.621371), y1 + int(122 * scale),
                1.9 * scale, max(2, int(5 * scale)), (10, 10, 10))
        if inferred:
            _centre("?", y1 + int(172 * scale), 0.7 * scale,
                    max(1, int(2 * scale)), (0, 140, 200))
    # What the car is actually aiming for, under the plate -- both modes add
    # an over-limit allowance on top, so target and limit are not the same
    # number and showing only one of them would be misleading.
    tgt = st.get("speed_target_kph")
    if tgt is not None:
        lbl = "tgt %.0f" % (tgt * 0.621371)
        if st.get("speed_limit_mode") == "MADS":
            lbl += " +"
        (tw, th), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.55 * scale, 2)
        tx = x1 + (pw - tw) // 2
        ty = y2 + int(26 * scale)
        cv2.putText(img, lbl, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55 * scale,
                    (0, 0, 0), max(3, int(4 * scale)), cv2.LINE_AA)
        cv2.putText(img, lbl, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55 * scale,
                    (0, 255, 120), max(1, int(2 * scale)), cv2.LINE_AA)
    return img


def overlay(frame, st, src_wh=None, minimal=False):
    """Draw the model's path and key state onto the frame the phone sees."""
    img = frame.copy()
    h, w = img.shape[:2]
    if SEG_OVERLAY["on"]:
        draw_seg(img, st)            # under the lanes/boxes: it is context, not the point
    draw_model(img, st, src_wh)
    draw_dets(img, st, src_wh)
    if minimal:
        # DISPLAY MODE ONLY. minimal=True is the --display path (the car's own
        # monitor); the web preview takes the full text block below and already
        # has the limit among its fields. The sign is drawn here rather than in
        # both so it does not clutter the phone page, which was the ask.
        img = _hud_minimal(img, st)
        if os.environ.get("OP_SPEED_LIMIT") == "1":
            img = draw_speed_limit_sign(img, st, float(os.environ.get("OP_HUD_SCALE", "2.2")))
        return img
    txt = [
        f"{st['v_ego_kph']:.1f} kph   {st['rpm']} rpm",
        f"calib {' '.join(f'{x*57.2958:+.2f}' for x in st['calib_rpy'])} deg  n={st['calib_samples']}",
        f"curv {st['curvature']:+.5f}   torque {st['would_command_torque']:+.3f}",
        # + = camera is RIGHT of lane centre. Blank until the model sees two lines.
        #
        # SPLIT ACROSS TWO LINES so the HUD can be scaled up. As one line this was
        # 939 px wide at scale 1.4 and set the ceiling for the whole overlay --
        # everything else had to stay small so this one line would fit. Two short
        # lines cost one row of height and let the text grow until the SHORTER
        # lines bind instead, which is the right trade on a dash monitor read at
        # arm's length.
        (f"lane off {st['lane_off_m']:+.2f}m @0m  {st['lane_off_near_m']:+.2f}m @6.8m"
         if st.get("lane_off_m") is not None else "lane off  --  (no lane lines)"),
        (f"   width {st['lane_width_m']:.2f}m  p {st['lane_p_left']:.2f}/{st['lane_p_right']:.2f}"
         if st.get("lane_off_m") is not None else ""),
        f"{st['op_state']}  cruise={'ENG' if st['cruise_enabled'] else ('avail' if st['cruise_available'] else 'off')}",
        # Headroom. --display serves no web page, so without this line the peak
        # counters are invisible in the one mode you actually watch while driving.
        # applied/STEER_MAX pinned at 100% means the ceiling is what binds and
        # raising it did something; well short of it means the ramp or the tune
        # binds instead and the higher ceiling is dead weight.
        # Split for the same reason as the lane line: as one string this was the
        # widest row on the HUD and capped how large everything could be drawn.
        (f"trq {st.get('applied_torque', 0):+5d} peak {st.get('applied_peak', 0)}"
         f"/{st.get('steer_max', 0)} want {st.get('want_peak', 0)}"),
        (f"   {st.get('lkas_hz', 0.0):.0f}Hz={st.get('ramp_cps', 0):.0f}c/s"
         f"  cam {st.get('cam_trq_peak', 0)}"),
    ]
    # HUD SIZE. 0.6 was sized for a phone held close on the web preview; on the
    # dash monitor at arm's length while driving it is unreadable, which defeats
    # the point of the mode -- warnings you cannot read at a glance are warnings
    # you do not act on.
    #
    # Everything scales together: glyph size, both stroke widths and the line
    # pitch. Scaling only the font makes the lines collide, and scaling only the
    # pitch leaves the text small with gaps. The outline stroke has to grow too or
    # thin green text disappears against a bright road.
    # 2.2 puts the glyphs at ~30 px against the original 14, which is what a dash
    # monitor read at arm's length needs. It only fits because the two longest
    # rows were split above: as single lines they measured 939 px of a 960 px
    # frame at scale 1.4 and capped the whole overlay. Split, the widest row is
    # ~852 px and the block ~422 px of 540 at this scale, so both dimensions still
    # have margin. 2.4 is about the ceiling before rows start clipping.
    _hud = float(os.environ.get("OP_HUD_SCALE", "2.2"))
    _fs = 0.6 * _hud
    _out = max(4, int(round(4 * _hud)))       # black outline, drawn first
    _thk = max(1, int(round(1.6 * _hud)))     # the green glyph itself
    _pitch = int(round(24 * _hud))
    y = _pitch
    for line in txt:
        cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, _fs, (0, 0, 0), _out, cv2.LINE_AA)
        cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, _fs, (0, 255, 120), _thk, cv2.LINE_AA)
        y += _pitch
    return img


def pipeline(args):
    # Which half of the split this process is. Read first because the panda is
    # opened long before the other flags are consulted, and only ONE process may
    # hold that USB handle -- a second claim raises USBErrorBusy and takes the
    # CAN link down with it. See --role.
    role = getattr(args, "role", "both")
    owns_panda = role != "model"
    import openpilot.cereal.messaging as messaging
    from cereal import log
    from opendbc.car.structs import car as car_struct
    from opendbc.car.mazda.values import CAR
    from opendbc.car.mazda.interface import CarInterface
    from openpilot.common.params import Params
    from curvature_lib import path_to_curvature, T_IDXS
    from usb_panda import UsbPanda
    from dashcam import CarStateFromCAN, SAFETY_SILENT
    SAFETY_NOOUTPUT = 19   # like SILENT for tx, but CAN stays LIVE so
                           # cam<->car forwarding and the camera's ACK
                           # survive -- our software stand-in for the
                           # harness relay this build does not have.
    from opendbc.car.lateral import apply_driver_steer_torque_limits
    from opendbc.car.mazda.values import CarControllerParams
    from opendbc.car.mazda import mazdacan
    from opendbc.can import CANPacker
    try:
        from openpilot.cereal.services import SERVICE_LIST
    except ModuleNotFoundError:
        from cereal.services import SERVICE_LIST
    import subprocess, signal

    armed = getattr(args, "arm", False) or os.environ.get("DASHCAM_ARM") == "1"
    # Arming is a panda operation end to end -- it sets SAFETY_MAZDA, starts the
    # 0x243 tx thread and, under alpha long, opens a UDS session that really does
    # change the car's state. None of that is the model role's to do, and it has
    # no panda handle to do it with. Folded in at the definition rather than at
    # the `if armed:` site so every downstream banner and warning agrees.
    if not owns_panda:
        armed = False
    mads = getattr(args, "mads", False) or os.environ.get("DASHCAM_MADS") == "1"
    alpha_long = getattr(args, "alpha_long", False) or os.environ.get("DASHCAM_ALPHA_LONG") == "1"
    # Inject 0x21b/0x21c WITHOUT suppressing the radar. See the long note where the
    # tx threads are started: the radar reaches the bus 14 ms after wake and will
    # not accept a diagnostic session for 163 ms, so it can never be silenced before
    # the car hears it. Coexisting leaves FCW/AEB/SBS alive and gives the cluster
    # nothing to report.
    coexist = os.environ.get("OP_ALPHA_COEXIST") == "1"
    if alpha_long and mads:
        # These used to be refused as mutually exclusive, and the reason was real:
        # MADS gated engagement inside the CRZ_CTRL rx handler, and alpha long
        # suppresses CRZ_CTRL, so MADS was silently inert. Both sides now read
        # MRCC state from PEDALS instead -- the PCM's own report, which stays
        # truthful with the radar silent -- so the combination works.
        #
        # It is NOT the usual MADS. controls_allowed is a single authority flag
        # in mazda.h, gating the LKAS torque checks and CRZ_CTRL's CRZ_ACTIVE
        # bit alike, so engaging on MAIN grants longitudinal at the same instant.
        print("!" * 64)
        print("!!  MADS + ALPHA LONG: openpilot takes steering AND throttle/brake")
        print("!!  as soon as MRCC MAIN is on. There is NO set-cruise step.")
        print("!!  Torque/rate limits, the +-2000 accel clip and the brake-edge")
        print("!!  drop still apply -- the requirement that you ASK first does not.")
        print("!!  The firmware must be built with -DMAZDA_MADS for this to work.")
        print("!" * 64)
    steer_angle_inject = (getattr(args, "steer_angle", False)
                          or os.environ.get("DASHCAM_STEER_ANGLE") == "1")
    # 0x243 tx rate. Stock is 100; default stays 50 because this process's GIL
    # contention, not the link, is what makes 100 jittery here -- see the note in
    # the tx thread. Clamped so a typo cannot stall or flood the EPS.
    lkas_hz = float(getattr(args, "lkas_hz", 70.0) or 70.0)
    lkas_hz = max(10.0, min(100.0, lkas_hz))

    # alpha_long is the third positional of get_params (get_non_essential_params
    # hardcodes it False). It is what sets CP.openpilotLongitudinalControl, which
    # in turn selects the panda's longitudinal tx table via the safety param AND
    # gates controlsd's whole long path -- controlsd.py:102 ANDs CC.longActive
    # with it, so with it False nothing below is reachable no matter what this
    # process publishes.
    if alpha_long:
        from opendbc.car import gen_empty_fingerprint
        CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(),
                                     [], True, False, False)
        if not CP.openpilotLongitudinalControl:
            raise SystemExit(
                "--alpha-long requested but CP.openpilotLongitudinalControl is False. "
                "The opendbc alpha-long port is not installed: see "
                "jetson_port/opendbc_patches/alpha_long/README.md")
        print("ALPHA LONG: openpilot commands throttle and brake. Radar is suppressed, "
              "so FCW/AEB/SBS are OFF and the dash will show malfunctions.")
    else:
        CP = CarInterface.get_non_essential_params(CAR.MAZDA_CX5_2022)
    # --- longitudinal MPC init ------------------------------------------------
    _mpc = None
    _mpc_state = {"v": 0.0, "a": 0.0, "v_cruise": V_MAX_KPH / 3.6}
    # Set only when OP_SPEED_LIMIT=1 and the OSM index loaded; stays None
    # otherwise so every consumer can test it without a hasattr dance.
    _speed_target = None
    _sl_index = None
    _T_IDXS_MPC = None
    _CTRL_T = None
    _MPC_ACTION_T = 0.05
    get_accel_from_plan = None
    if MPC_ENABLED:
        try:
            from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
                LongitudinalMpc, T_IDXS as _T_IDXS_MPC_SRC)
            from openpilot.selfdrive.controls.lib.drive_helpers import (
                get_accel_from_plan, CONTROL_N as _CONTROL_N)
            from openpilot.selfdrive.modeld.constants import ModelConstants as _MCmpc
            # CONTROL_N_T_IDX is defined in longitudinal_planner, which this board
            # does not run -- rebuild it from the same two constants it uses.
            _CTRL_T_SRC = _MCmpc.T_IDXS[:_CONTROL_N]
            _mpc = LongitudinalMpc()
            _T_IDXS_MPC = np.array(_T_IDXS_MPC_SRC)
            _CTRL_T = np.array(_CTRL_T_SRC)
            _mpc_state["v_cruise"] = float(os.environ.get("OP_V_TARGET_KPH", V_MAX_KPH)) / 3.6
            # SPEED-LIMIT-AWARE TARGET, opt-in via OP_SPEED_LIMIT=1.
            #
            # Off by default and deliberately so: with it off this line is the
            # only thing that sets v_cruise and the behaviour is exactly what it
            # was. With it on, speed_target.SpeedTarget recomputes v_cruise from
            # the GPS position against an offline OSM index, once a second.
            #
            #   STANDARD  target = posted limit
            #   MADS      target = posted limit + OP_MADS_OVER_MPH (default 15 mph)
            #
            # The mode follows the --mads flag, so the same switch that changes
            # how openpilot engages also changes what it aims for.
            #
            # THIS SETS AN AIM, NOT AN AUTHORITY. govern_accel still clamps after
            # the MPC, V_MAX_KPH is still the ceiling, and the panda's limits are
            # untouched. A wrong lookup makes the car aim for the wrong speed; it
            # cannot make the car exceed a cap that already bounded it.
            #
            # COVERAGE, MEASURED on the New York extract: only 10.9% of drivable
            # ways carry an explicit maxspeed tag. The rest fall back to a
            # highway-type default and come back flagged `inferred`, which
            # SpeedTarget discounts by OP_LIMIT_INFERRED_SCALE. Treat an inferred
            # limit as a hint, not a posting.
            if os.environ.get("OP_SPEED_LIMIT") == "1":
                try:
                    sys.path.insert(0, os.path.join(os.path.dirname(
                        os.path.dirname(os.path.abspath(__file__)))))
                    from gps_reader import GpsReader as _GpsReader
                    from speed_limits import SpeedLimitIndex as _SLIndex
                    from speed_target import SpeedTarget as _SpeedTarget
                    _sl_index = _SLIndex()
                    if not _sl_index.ok:
                        print("SPEED LIMIT: index unavailable (%s) -- target stays "
                              "at the fixed value" % _sl_index.error)
                    else:
                        # SPEED-TARGET MODE IS ITS OWN SWITCH.
                        #
                        # It used to follow --mads, which conflated two unrelated
                        # things under one flag:
                        #   SpeedTarget mode   STANDARD = limit + OP_STD_OVER_MPH
                        #                      MADS     = limit + OP_MADS_OVER_MPH
                        #   LATERAL MADS       DisengageOnAccelerator=False (1437)
                        #                      OP_DISENGAGE_ON_BRAKE=0      (1872)
                        #
                        # Asking for the higher speed target therefore also stopped
                        # openpilot disengaging on the gas AND on the brake. That is
                        # a substantial change to override behaviour, bought by
                        # accident while reaching for a cruise-target offset.
                        #
                        # OP_SPEED_LIMIT_MODE selects the target mode alone.
                        # Unset, it still follows --mads so existing invocations
                        # behave as before.
                        _slm = os.environ.get("OP_SPEED_LIMIT_MODE", "").strip().lower()
                        _sl_mads = bool(mads) if _slm == "" else (_slm == "mads")
                        _speed_target = _SpeedTarget(
                            mads=_sl_mads, v_max_kph=V_MAX_KPH,
                            index=_sl_index, gps=_GpsReader().start())
                        print("SPEED LIMIT: %s mode, %d segments, GPS on %s"
                              % (_speed_target.status()["mode"],
                                 _sl_index.status()["segments"],
                                 os.environ.get("OP_GPS_PORT", "/dev/ttyTHS1")))
                except Exception as _sle:
                    print("SPEED LIMIT: disabled --", _sle)
            # action_t is how far ahead the plan is sampled; it must carry the
            # SAME longitudinal delay the car has, or the MPC's smooth profile is
            # applied at the wrong moment. CP.longitudinalActuatorDelay is 0.36 for
            # this port and is NOT measured on this car -- see interface.py.
            _MPC_ACTION_T = float(getattr(CP, "longitudinalActuatorDelay", 0.36)) + 0.05
            print("MPC: longitudinal MPC ACTIVE (cruise target %.0f kph, action_t %.2fs)"
                  % (_mpc_state["v_cruise"] * 3.6, _MPC_ACTION_T))
        except Exception as _e:
            _mpc = None
            print("MPC: unavailable (%s: %s) -- falling back to govern_accel" % (type(_e).__name__, _e))
    else:
        print("MPC: off (govern_accel clamp only); set OP_MPC=1 to enable")
        # SPEED LIMIT ON THE e2e PATH.
        #
        # The initialisation above lives inside `if MPC_ENABLED:`, which coupled
        # two unrelated things. A posted limit is just a CAP, and govern_accel
        # already HAS a speed cap -- it was applying the constant V_MAX_KPH where
        # it could apply the live limit. So with the MPC off (the e2e path, which
        # is the configuration that actually drives this car well) the GPS and the
        # 8.3M-segment OSM index went unused and the only ceiling was
        # OP_V_MAX_MPH's fixed 72 mph.
        #
        # Same construction, run again here so BOTH paths get the target: the MPC
        # consumes it as a cruise target, govern_accel as a dynamic cap.
        if os.environ.get("OP_SPEED_LIMIT") == "1":
            try:
                sys.path.insert(0, os.path.join(os.path.dirname(
                    os.path.dirname(os.path.abspath(__file__)))))
                from gps_reader import GpsReader as _GpsReader
                from speed_limits import SpeedLimitIndex as _SLIndex
                from speed_target import SpeedTarget as _SpeedTarget
                _sl_index = _SLIndex()
                if not _sl_index.ok:
                    print("SPEED LIMIT: index unavailable (%s) -- cap stays at "
                          "the fixed V_MAX_KPH" % _sl_index.error)
                else:
                    _slm = os.environ.get("OP_SPEED_LIMIT_MODE", "").strip().lower()
                    _sl_mads = bool(mads) if _slm == "" else (_slm == "mads")
                    _speed_target = _SpeedTarget(
                        mads=_sl_mads, v_max_kph=V_MAX_KPH,
                        index=_sl_index, gps=_GpsReader().start())
                    print("SPEED LIMIT: %s mode, %d segments, GPS on %s "
                          "(e2e path -- feeds govern_accel's cap)"
                          % (_speed_target.status()["mode"],
                             _sl_index.status()["segments"],
                             os.environ.get("OP_GPS_PORT", "/dev/ttyTHS1")))
            except Exception as _sle:
                print("SPEED LIMIT: disabled --", _sle)

    p_ = Params()
    p_.put("CarParams", CP.to_bytes())
    p_.put_bool("OpenpilotEnabledToggle", True)
    # The whole point of MADS here is that the driver works the pedals, so the
    # accelerator must not be a disengage. Left at the stock True the lateral
    # controller would drop out every time the throttle is touched -- i.e. on
    # every turn you were trying to take slowly. Brake is NOT touched: braking
    # still disengages, which is the behaviour worth keeping.
    p_.put_bool("DisengageOnAccelerator", not mads)

    # USB, not serial. The board is an STM32F407 on native USB since the move off
    # the Nucleo-F446 -- there is no /dev/serial/by-id and no /dev/ttyACM*, so
    # SerialPanda(find_panda_port()) raised at startup no matter what flags were
    # passed. Same four methods, so nothing below changes. See jetson_port/usb_panda.py.
    # PANDA LIFECYCLE -- panda role only.
    #
    # Exactly one process may hold the USB handle; a second claim raises
    # USBErrorBusy and takes the CAN link down with it. In the model role the
    # panda is owned by the other process and carState arrives over msgq, so
    # everything below is skipped and `panda` stays None. Every later use is
    # guarded on owns_panda for the same reason.
    if owns_panda:
        panda = UsbPanda()
        # SAFETY_SILENT does NOT just stop transmitting -- it stops FORWARDING.
        # nooutput_init() in opendbc/safety/modes/defaults.h returns
        #     (safety_config){NULL, 0, NULL, 0, true}   // disable_forwarding = true
        # and safety_fwd_hook() checks that first, returning -1 for every address.
        #
        # This build has NO harness relay (nucleo_harness_config), so the panda is the
        # ONLY path between the forward camera on bus 2 and the car on bus 0. Silent
        # mode therefore SEVERS them: the car sees no camera at all and raises a front
        # camera sensor fault within seconds. A stock panda survives this because its
        # relay re-bridges cam<->car whenever openpilot is not intercepting; we have no
        # such fallback.
        #
        # So silent is the safe default only when the car is ASLEEP. With CAN ignition
        # present, go straight to Mazda safety: it forwards and ACKs, and it commands
        # nothing on its own -- the tx thread is what sends frames, and that only runs
        # when armed. MEASURED 2026-08-07: the camera also needs the ACK. Left unacked
        # it retransmits CAM_EMPTY at line rate (3656 Hz) and never reaches its real
        # messages, because the panda is its only bus partner on that segment.
        panda.control_write(0xdc, SAFETY_NOOUTPUT, 0)
        print("panda: startup safety = SAFETY_NOOUTPUT "
              "(no transmit; cam<->car passthrough + ACK kept alive)")
        time.sleep(0.3)

        # --- CAN-live gate -------------------------------------------------------
        # bxCAN needs 11 consecutive RECESSIVE bits to leave initialisation. Start
        # this stack while the car is off and CAN1 never gets them: it stays in init
        # FOREVER. Turning the ignition on afterwards does not retrigger init, and
        # neither does re-setting the safety mode (can_init_all cannot help a core
        # that is already wedged) -- only a device reset does.
        #
        # The failure is silent and total, and it does NOT look like a bus problem:
        #   MEASURED 2026-08-09, 543 s run -- CAN1 total_rx_cnt frozen at 5,263,945
        #   (delta 0/s with the ignition on), TEC=0, REC=0, bus_off=0, last_error
        #   "No error". Zero traffic AND zero errors, because a core in init neither
        #   receives nor error-counts. Every carState field held its startup value
        #   for the whole run, so v/rpm/angle read 0.0 and openpilot simply never
        #   engaged with nothing in the log to say why.
        #
        # Worse, reception is not the only casualty. Forwarding runs inside can_rx(),
        # so a wedged core also stops bridging cam<->bus0 -- and with no harness
        # relay the panda is the only path between them. The cluster raised a front
        # camera sensor fault, which looks exactly like the alpha-long radar
        # suppression fault and sent me chasing the wrong cause.
        #
        # So probe before trusting the link, and reset if it is dead. 0xd8 is
        # NVIC_SystemReset -- a firmware reboot, NOT a flash; it re-runs can_init
        # against whatever the bus looks like now.
        def _bus0_rate(pnd, dur=1.0):
            pnd._buf = b''
            t_end, n = time.monotonic() + dur, 0
            while time.monotonic() < t_end:
                for _a, _d, _b in pnd.can_recv():
                    if _b == 0:
                        n += 1
            return n / dur

        def _hw_vs_host(pnd, dur=1.0):
            """(frames the SILICON received, frames that reached us) over dur.

            THE FAILURE THIS CATCHES, and it cost most of a debugging session.
            The panda's CAN cores keep receiving at full rate while the USB
            stream to the host wedges, so can_recv returns a trickle -- 2, 10,
            22 frames/s against a bus carrying 2800. Every symptom then looks
            like the CAR: v=0, rpm=0, ignition MISSING, cam_seen=0, cruise
            unavailable. It is indistinguishable from a sleeping car unless you
            compare against the hardware counter, which is the one number that
            keeps counting through it.
            MEASURED 2026-08-10: CAN1 total_rx_cnt climbing +2330/s while the
            host saw 4/s. That was read as "car asleep" repeatedly, and produced
            a dead-camera diagnosis, a connector hunt and a cruise-refusal
            theory -- all of them artefacts of this one fault.
            The gate below only asked "is the host seeing anything?", so a
            trickle passed it. Ask the silicon too.
            """
            try:
                h0 = pnd.p.can_health(0)["total_rx_cnt"]
            except Exception:
                return None, _bus0_rate(pnd, dur)
            host = _bus0_rate(pnd, dur)
            try:
                h1 = pnd.p.can_health(0)["total_rx_cnt"]
            except Exception:
                return None, host
            return (h1 - h0) / dur, host

        hw_rate, rate = _hw_vs_host(panda)
        # Hardware busy but nothing reaching us -> the link is wedged, not the bus.
        # A board reset is the only thing that reliably clears it; can_reset_
        # communications alone has been observed not to.
        if hw_rate is not None and hw_rate > 200.0 and rate < 0.25 * hw_rate:
            print("panda: LINK WEDGED -- silicon is receiving %.0f frames/s but only "
                  "%.0f/s reach the host. Resetting the board." % (hw_rate, rate))
            try:
                panda.control_write(0xd8, 0, 0)
            except Exception:
                pass
            try:
                panda.close()
            except Exception:
                pass
            time.sleep(4.0)
            panda = UsbPanda()
            panda.control_write(0xdc, SAFETY_NOOUTPUT, 0)
            time.sleep(0.5)
            hw_rate, rate = _hw_vs_host(panda)
            print("panda: after reset -- silicon %.0f/s, host %.0f/s"
                  % (hw_rate if hw_rate is not None else -1, rate))

        if rate == 0.0:
            print("panda: bus 0 SILENT at startup -- CAN1 may be stuck in init; "
                  "resetting the board (0xd8) and retrying")
            try:
                panda.control_write(0xd8, 0, 0)
            except Exception:
                pass          # the reset tears the USB link down mid-transfer
            try:
                panda.close()
            except Exception:
                pass
            time.sleep(4.0)
            panda = UsbPanda()
            panda.control_write(0xdc, SAFETY_NOOUTPUT, 0)
            time.sleep(0.5)
            rate = _bus0_rate(panda)

        if rate == 0.0:
            # A reset against a genuinely dead bus leaves the core wedged again, so
            # this is not recoverable from here -- it needs the ignition on first.
            print("panda: *** bus 0 STILL SILENT (%.0f frames/s) ***" % rate)
            print("       The car is asleep, or the harness is not connected.")
            print("       Turn the ignition ON, then restart this stack -- CAN1")
            print("       cannot leave init against a dead bus, and starting now")
            print("       means carState stays frozen and cam<->car forwarding")
            print("       never runs (the cluster then reports a camera fault).")
        else:
            print("panda: bus 0 live at %.0f frames/s" % rate)
    else:
        panda = None
        print("panda: not opened (--role model); carState comes from the panda "
              "role over msgq")

    # --- transmit path (only actually sends when armed) ---------------------
    packer = CANPacker('mazda_2017')
    tx_state = {"torque_norm": 0.0, "lat_active": False, "op_enabled": False,
                "apply_last": 0, "tx_frames": 0, "applied": 0,
                "curvature": 0.0, "angle_deg": 0.0, "angle_raw": 0,
                # panda-side rejection tracking, see the resync block in tx_thread
                "blocked_last": 0, "blocked_seen": 0, "resyncs": 0,
                # --- panda safety-mode watchdog -----------------------------
                # SAFETY_MAZDA was written exactly once, at arm time. Anything
                # that puts the panda back in SAFETY_SILENT -- a reset on the
                # cranking brownout, a USB re-enumeration, a lost heartbeat --
                # was permanent, because nothing ever re-asserted it. See the
                # watchdog in tx_thread for why that is the whole ballgame.
                "remodes": 0, "panda_resets": 0, "uptime_last": 0,
                # The safety PARAM the panda is supposed to be holding, stashed
                # at arm time. Bit 0 is MAZDA_PARAM_LONGITUDINAL: with it clear
                # the firmware selects the 3-entry stock tx table and rejects
                # EVERY 0x21b/0x21c/0x764 frame, while can_send still succeeds
                # and long_tx_frames still climbs. The watchdog below used to
                # re-assert the mode with a hardcoded 0, which silently turned
                # alpha long off after any board reset and looked exactly like
                # "the car ignores our acceleration".
                "safety_param": 0, "param_fixups": 0,
                # --- headroom telemetry -------------------------------------
                # applied_peak is what actually went on the wire AFTER the rate
                # limiter; want_peak is what the controller asked for BEFORE it.
                # The gap between them is the ramp limit, the gap between
                # applied_peak and STEER_MAX is the amplitude limit. Raising
                # STEER_MAX only helps if applied_peak is pinned AT STEER_MAX --
                # otherwise the ceiling is not the thing binding, and raising it
                # buys nothing (see the reverted MAX_LATERAL_ACCEL change).
                "applied_peak": 0, "want_peak": 0.0,
                # measured tx rate: the --lkas-hz target is a request, not a
                # fact. GIL contention here has dragged a 100 Hz target down to
                # ~28 Hz before, and the EPS faults on IRREGULAR 0x243, so the
                # achieved rate has to be read, not assumed.
                "hz": 0.0, "hz_t0": 0.0, "hz_n0": 0,
                # tx interval jitter -- see the note at the measurement site.
                # The mean rate above cannot distinguish a steady stream from a
                # bursty one, and the EPS only cares about the latter.
                "last_tx_t": 0.0, "worst_ms": 0.0, "worst_ms_run": 0.0,
                "worst_ms_all": 0.0, "late_frames": 0}
    # Live camera CAM_LKAS (0x243) state bits, read off bus 2. The real Mazda
    # carcontroller COPIES these into the steering frame; BIT_1 is the "LKAS active"
    # bit the EPS requires -- injecting 0 makes the EPS ignore our torque. Decoded
    # from the DBC: BIT_1 @bit29 (byte3 b5), ERR_BIT_1 @bit16 (byte2 b0),
    # ERR_BIT_2 @bit30 (byte3 b6).
    cam_state = {"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0, "seen": 0, "last_t": 0.0,
                 "lane_seen": 0, "lane_t": 0.0,
                 "ck_ok": 0, "ck_bad": 0, "ck_skip": 0, "ck_angle_seen": 0,
                 # Peak |LKAS_REQUEST| the STOCK camera has ever commanded on bus 2.
                 # This is the only empirical evidence of a torque level the EPS
                 # provably accepts, so it is the number to size STEER_MAX against
                 # -- the 2047 the CAN field allows is just the field width, not a
                 # statement about what the rack will do. Only counts frames that
                 # passed the checksum check, so a misparse cannot inflate it.
                 "trq_peak": 0,
                 # CAM_LKAS.CTR continuity -- see the note where it is decoded.
                 # ctr_last < 0 means "no frame yet", so the first one does not
                 # invent a delta against a counter we never saw.
                 "ctr_last": -1, "ctr_d": {}}

    # Peak |LKAS_REQUEST| and the |LKAS_EFFECTIVE| that the EPS paired with it.
    # Tracked at the REQUEST peak rather than independently: the question is
    # "when we asked for the most, how much did the rack deliver", and two
    # independent maxima taken at different instants cannot answer that.
    eps_peak = {"req": 0, "eff": 0}

    def _validate_cam_checksum(d):
        """Prove mazdacan's 0x243 packing+checksum against the REAL camera.

        The CAM_LKAS checksum in mazdacan.py is reverse-engineered (note the bare
        `if ahi == 1: csum += 15` fudge), and with the stock steering_angle of 0
        the angle nibbles only ever take one value. Before trusting a model-derived
        STEERING_ANGLE we need evidence the formula holds for other values.

        So: decode each camera frame, re-pack it with our own code path, and compare
        all 8 bytes. Only frames create_steering_control can reproduce exactly are
        comparable -- it hardcodes LINE_NOT_VISIBLE/LDW/ANGLE_ENABLED to 0 -- the
        rest are counted as skipped rather than failed. ck_angle_seen counts the
        comparable frames that carried a NON-ZERO angle: if it stays 0, this camera
        never exercises the unfitted part of the formula and --steer-angle is
        still unproven no matter how high ck_ok climbs.
        """
        lnv = (d[2] >> 3) & 1
        ldw = (d[2] >> 7) & 1
        b2 = (d[6] >> 4) & 1
        # With OP_LKAS_COPY_CAM_LINES set, create_steering_control reproduces the
        # camera's LINE_NOT_VISIBLE and LDW instead of forcing them to 0, so those
        # frames become comparable and MUST be compared -- they are the only
        # evidence that the checksum's (lnv << 3) and (ldw << 7) terms are right.
        #
        # They have never been checked. The skip below is why: every frame that
        # would exercise them was counted as skipped rather than failed, so ck_ok
        # could climb forever while those two terms stayed unproven. The formula
        # is reverse-engineered and fitted to captures, and a neighbouring term
        # looks suspect on inspection -- ERR_BIT_2 sits at DBC bit 30 (byte 3,
        # bit 6) yet the formula uses (er2 << 4) -- which survives only because
        # error bits are normally 0. Treat the lnv/ldw terms with the same
        # suspicion until ck_bad proves otherwise.
        #
        # ck_skip is therefore the cheap pre-test: run a drive with the flag OFF
        # and if ck_skip stays 0, this camera never sets these bits at all, the
        # copy would be a no-op, and the front camera fault has another cause.
        # WHICH bit forces the skip, counted separately.
        #
        # ck_skip lumps three signals together, and MEASURED 2026-08-09 it was
        # 1869 out of 1869 -- every camera frame skipped, so ck_ok was 0 and our
        # 0x243 packing went completely unvalidated for the whole run. "All of
        # them" is useless on its own: OP_LKAS_COPY_CAM_LINES copies
        # LINE_NOT_VISIBLE and LDW but NOT ANGLE_ENABLED, so if b2 is the bit
        # actually being set the flag cannot help and enabling it would prove
        # nothing. Split the counter so the next run says which.
        if lnv:
            cam_state["ck_lnv"] = cam_state.get("ck_lnv", 0) + 1
        if ldw:
            cam_state["ck_ldw"] = cam_state.get("ck_ldw", 0) + 1
        if b2:
            cam_state["ck_ang"] = cam_state.get("ck_ang", 0) + 1
        _cmp_lines = getattr(mazdacan, "_COPY_CAM_LINES", False)
        if b2 or ((lnv or ldw) and not _cmp_lines):
            cam_state["ck_skip"] += 1
            return
        # STEERING_ANGLE: DBC 33|12@0+ -> byte4 bits 1..0, byte5, byte6 bits 7..6
        ang = (((d[4] & 0x03) << 10) | (d[5] << 2) | ((d[6] >> 6) & 0x03)) - 2048
        # LINE_NOT_VISIBLE/LDW are read by create_steering_control only when
        # OP_LKAS_COPY_CAM_LINES is set, but they are supplied unconditionally:
        # the call below is wrapped in a bare `except Exception: return`, so a
        # missing key would not raise, it would silently stop validating and
        # leave ck_ok/ck_bad frozen while everything looked fine.
        bits = {"BIT_1": (d[3] >> 5) & 1, "ERR_BIT_1": d[2] & 1, "ERR_BIT_2": (d[3] >> 6) & 1,
                "LINE_NOT_VISIBLE": lnv, "LDW": ldw}
        torque = (((d[0] & 0x0F) << 8) | d[1]) - 2048
        try:
            ref = mazdacan.create_steering_control(packer, CP, d[0] >> 4, torque, bits, ang)
        except Exception:
            return
        ck_ok = bytes(ref[1]) == bytes(d)
        cam_state["ck_ok" if ck_ok else "ck_bad"] += 1
        # KEEP THE FRAMES THAT FAIL, not just a count.
        #
        # MEASURED 2026-08-09: ck = 7056/43/10155 -- 43 of 7099 comparable frames
        # (0.6%) did NOT reproduce byte-for-byte. That is not a validation
        # curiosity: create_steering_control is the SAME code that builds what we
        # transmit, so whatever input it mis-packs here, it also mis-packs on the
        # wire, and the EPS rejects those frames. It lines up with lkas_block
        # flapping and tx_blocked climbing.
        #
        # A count cannot say WHICH input breaks it. The prime suspect is the
        # STEERING_ANGLE path -- the checksum carries a bare `if ahi == 1:
        # csum += 15` fudge that this file's own docstring admits was only ever
        # fitted against captures, and ahi is always 2 when the angle is 0. So
        # record the raw frame, our attempt, and the decoded fields, and let the
        # difference name itself.
        if not ck_ok and len(cam_state.setdefault("ck_bad_frames", [])) < 40:
            _tmp = ang + 2048
            cam_state["ck_bad_frames"].append({
                "cam": bytes(d).hex(),
                "ours": bytes(ref[1]).hex(),
                "ang": ang,
                "ahi": _tmp >> 10,          # the nibble the fudge keys on
                "torque": torque,
                "ctr": d[0] >> 4,
                "csum_cam": d[7] if len(d) > 7 else None,
            })
        if ang != 0:
            cam_state["ck_angle_seen"] += 1
        # Only trust the torque from a frame we can reproduce byte-for-byte --
        # a bad checksum means we decoded the layout wrong, and a garbage peak
        # here would be sizing a safety limit off a parse error.
        if ck_ok:
            cam_state["trq_peak"] = max(cam_state["trq_peak"], abs(torque))

    # Model -> steering-wheel angle. controlsd's LatControlTorque returns 0.0 for
    # its `lateral_output`, so carControl.actuators.steeringAngleDeg is ALWAYS 0 on
    # this car -- it cannot be used. actuators.curvature is the real signal: it is
    # controlsd's post-clip_curvature desired curvature (the one the lateral
    # controller actually targets), so it already has the lateral accel/jerk limits
    # applied. Invert it through the same VehicleModel controlsd uses. The negation
    # matches controlsd: `self.curvature = -VM.calc_curvature(sa, ...)`.
    from opendbc.car.vehicle_model import VehicleModel
    VM = VehicleModel(CP)

    def model_steer_angle_deg(curvature, v_ego):
        try:
            return math.degrees(VM.get_steer_from_curvature(-float(curvature), max(float(v_ego), 0.0), 0.0))
        except Exception:
            return 0.0

    # Road-metered AE: Argus's own fast AE runs unlocked in the ISP, and a slow
    # outer loop re-aims it using the ROAD band only (exposurecompensation). Argus
    # meters the whole frame, so a bright sky drags the road down -- measured sky
    # p50 184 vs road p50 69 in one frame, with the whole-frame mean looking
    # perfectly fine at 101. No autolevel: that was a workaround for a sensor
    # pinned at the wrong exposure, and stretching a correctly-exposed frame just
    # moves the model's input off its training distribution.
    # --can-only skips the camera and the model entirely. Measured 2026-07-30 on
    # this box with --no-detect --no-scene --no-depth --no-roadseg already set:
    # the process still burned ~155% CPU, nvargus-daemon another ~27%, and the
    # two openpilot daemons ~45% between them. None of that serves the 0x243
    # stream, and all of it competes with the tx thread for the GIL -- which is
    # the one thing that puts holes in the frame the EPS faults on.
    can_only = bool(getattr(args, "can_only", False))
    cam = runner = None
    if not can_only:
        from op_camera_ae import CameraAE
        cam = CameraAE(auto_exposure=True)
        from op_stream import SupercomboRunner
        runner = SupercomboRunner()

    # CPU isolation (fixes selfdrivedLagging). selfdrived's loop is message-driven
    # with no sleep; on this shared 6-core Jetson it was descheduled behind the
    # model/publish load and averaged <90 Hz even at ~8% CPU (its inputs arrive at
    # ~290 Hz, so it was starved of CPU, not of messages). Pin this process (model
    # inference + 250 Hz publisher) to cores 0-3 and give each RT daemon its own
    # core: selfdrived -> 4, controlsd -> 5.
    #
    # ROLE-AWARE SINCE THE TWO-PROCESS SPLIT. The allocation above was written for
    # the single-process build, where cores 0-3 held camera + model + panda and the
    # two RT daemons got 4 and 5. After the split this same line ran in BOTH roles,
    # so the panda role pinned itself to 0-3 alongside a model role measured at
    # 156% CPU, and the one loop with a hard deadline was the only one with no core
    # of its own.
    #
    # That deadline is real and it is short. can_rx_q holds 2048 frames and bus 0
    # carries ~2873 frames/s, so the host has 0.71 s to get back to bulkRead before
    # the queue overflows -- and overflow does not degrade gracefully, it tears the
    # USB framing and ends in a silently frozen carState (v/angle pinned, cam_age
    # climbing, nothing logged). MEASURED 2026-08-11 with both roles on 0-3 and
    # loadavg 7.57 on 6 cores: reception froze for 450 s with angle decoding as
    # -1395.20 deg from a torn frame.
    #
    # This is NOT a bandwidth problem, and the ID list is not the lever. At 14 wire
    # bytes per frame the feed is 39 KB/s against ~1000 KB/s of USB FS bulk, and
    # the host unpack loop benchmarks at 380k frames/s -- 132x headroom. Trimming
    # MAZDA_HOST_IDS only lengthens the overflow window; it never fixes the stall
    # that consumes it. Give the panda role cores of its own instead, and the radar
    # shadow IDs fit with room to spare.
    ncpu = os.cpu_count() or 6
    try:
        if ncpu >= 6:
            # The daemons keep 4 and 5 (see daemon_core below) and total ~34%, so
            # the panda role shares that pair rather than taking a core outright:
            # its 10 threads at ~18% combined are better served by two cores at
            # ~57% load than by one at ~40% where they would queue behind
            # each other.
            os.sched_setaffinity(0, {4, 5} if role == "panda" else {0, 1, 2, 3})
    except Exception as e:
        print("could not pin main process:", e)

    procs = []
    if not args.no_daemons:
        env = dict(os.environ)
        env["PYTHONPATH"] = os.pathsep.join([os.path.join(_CX5_ROOT, "msgq"), os.path.join(_CX5_ROOT, "car"),
                             _CX5_ROOT, os.path.join(_CX5_ROOT, "openpilot")])
        # This board has no camerad, no sensord, no locationd: supercombo is fed
        # straight from the IMX477 in-process and modelV2 is published from here.
        # Without this flag selfdrived raises four NO_ENTRY events for daemons that
        # will never exist in this architecture -- cameraMalfunction (camera state
        # topics never published), posenetInvalid + locationdTemporaryError
        # (livePose never published), sensorDataInvalid (no IMU packets) -- and
        # engagement is blocked permanently. See selfdrived.py:87. This suppresses
        # ONLY those liveness checks; calibrationIncomplete still gates engagement,
        # and the panda's own Mazda safety limits are untouched.
        env["JETSON_CAMERA_BYPASS"] = "1"
        # Under MADS the brake must not disengage lateral. Stock gates the
        # accelerator on DisengageOnAccelerator (set above) but gates the brake on
        # nothing, so lateral dropped out on every slowdown -- MEASURED over one
        # 1583 s drive, all TWELVE transitions to disabled had brake=1, and
        # nothing else disengaged the stack at all.
        #
        # It is also what stops the car TURNING: pick_desire only issues
        # turnLeft/turnRight while lat_active is true, and you brake to take a
        # turn, so the desire is suppressed exactly when it is wanted.
        #
        # NOTE this only lifts openpilot's half of the gate. The panda enforces
        # its own -- mazda.h does `controls_allowed = cruise_engaged &&
        # !brake_pressed` -- so until that build changes, the firmware still
        # refuses torque while the brake is down and lateral will still drop.
        if mads:
            env["OP_DISENGAGE_ON_BRAKE"] = "0"
        daemon_core = {"openpilot.selfdrive.selfdrived.selfdrived": "4",
                       "openpilot.selfdrive.controls.controlsd": "5"}
        for mod in ("openpilot.selfdrive.selfdrived.selfdrived",
                    "openpilot.selfdrive.controls.controlsd"):
            base = ["python3", "-m", mod]
            # taskset pins the daemon to its own core so its real-time loop is not
            # bounced behind the model/publish threads.
            cmd = (["taskset", "-c", daemon_core[mod]] + base) if ncpu >= 6 else base
            procs.append(subprocess.Popen(cmd,
                                          cwd=os.path.join(_CX5_ROOT, "openpilot"), env=env,
                                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        time.sleep(4.0)

    pub = ['deviceState', 'pandaStates', 'peripheralState', 'modelV2', 'liveCalibration',
           'carOutput', 'driverMonitoringState', 'longitudinalPlan', 'livePose', 'liveDelay',
           'managerState', 'liveParameters', 'radarState', 'liveTorqueParameters',
           'driverAssistance', 'alertDebug', 'lateralManeuverPlan', 'carState']
    LIST_TOPICS = {'pandaStates'}
    avail = []
    for t in pub:
        try:
            messaging.new_message(t, 0) if t in LIST_TOPICS else messaging.new_message(t)
            avail.append(t)
        except Exception:
            pass
    # carState is published from its OWN thread (below), not the heavy multi-topic
    # loop: selfdrived's data_sample() does a blocking recv_one(carState, 20ms), so
    # its whole loop rate == the carState publish rate. The shared publish loop only
    # sustains ~83 Hz under GIL contention with the model, which put selfdrived's
    # loop at ~12 ms (>11.1 ms) and raised selfdrivedLagging. A dedicated ~110 Hz
    # carState thread keeps selfdrived's loop under 10 ms.
    # SPLIT THE TOPICS BY ROLE, or the two processes fight over them.
    #
    # The publish loop runs in BOTH roles, and it sends every topic it was given
    # whether or not this process produced the data behind it. Left alone, the
    # panda role would publish an all-defaults modelV2, liveCalibration and
    # livePose alongside the model role's real ones -- two publishers on one
    # topic, with subscribers seeing whichever arrived last. selfdrived would
    # then be reading a model output that flickers between real and empty, and
    # nothing in either log would say why.
    #
    # These are the topics backed by panda-side state: pandaStates comes from
    # health_state, which can_thread fills, and carOutput carries the torque
    # actually applied, which only the tx thread knows. carState is handled
    # separately through pm_cs. Everything else is model-side.
    PANDA_TOPICS = {'pandaStates', 'carOutput'}
    # ...but NOT from the shared publish loop when this process owns the panda.
    #
    # MEASURED 2026-08-10: in the panda role that loop runs at ~52 Hz, not the 250
    # it is paced for -- can_thread, tx_thread and carstate_thread crowd it out.
    # carOutput then publishes at 52.6 Hz against a 100 Hz nominal and pandaStates
    # at 6.1 against 10, both under selfdrived's 0.8x floor, which raises
    # commIssue/commIssueAvgFreq. Those are SOFT-DISABLE events, so lateral
    # engaged for a median of 4.0 seconds and dropped, 145 times in one drive.
    #
    # carstate_thread holds 109.8 Hz in the same process, so publish them there
    # instead. Nothing else changes: carOutput carries no populated fields here
    # (selfdrived polls it for liveness), and pandaStates is built from
    # health_state either way.
    FAST_PANDA_TOPICS = PANDA_TOPICS if owns_panda else set()
    if role == "panda":
        main_pub = [t for t in avail if t in PANDA_TOPICS and t not in FAST_PANDA_TOPICS]
    elif role == "model":
        main_pub = [t for t in avail if t != 'carState' and t not in PANDA_TOPICS]
    else:
        main_pub = [t for t in avail if t != 'carState']
    pm = messaging.PubMaster(main_pub)
    # Only the panda role publishes carState -- it is the process that decodes
    # the bus. Two publishers on one topic would interleave a real carState with
    # an all-defaults one and make engagement flap for reasons invisible in
    # either log.
    pm_cs = (messaging.PubMaster(['carState'])
             if ('carState' in avail and owns_panda) else None)
    # carOutput + pandaStates ride carstate_thread's 110 Hz clock, not the shared
    # publish loop -- see the note at FAST_PANDA_TOPICS.
    _fast_list = [t for t in avail if t in FAST_PANDA_TOPICS]
    pm_fast = messaging.PubMaster(_fast_list) if _fast_list else None
    _sub = ['selfdriveState', 'carControl', 'onroadEvents']
    if role == "model":
        # The model role does not own the panda, so nothing local decodes the bus.
        # carState arrives from the panda role instead.
        _sub.append('carState')
    sm = messaging.SubMaster(_sub)
    time.sleep(0.4)

    class _CarStateFromMsgq:
        """CarStateFromCAN's attribute surface, fed from a subscribed carState.

        WHY A SHIM RATHER THAN A REWRITE. cs_can is read at 45 sites outside the
        CAN threads -- the model's v_ego, the angle-offset learner, the desire
        helper, the display. Presenting the same attribute names means every one
        of those keeps working untouched, so splitting the process cannot quietly
        change what the model is fed.

        Only the fields stock carState actually carries are mapped. The
        Mazda-specific ones (acc_armed, eps_request, eps_effective, lkas_block,
        hands_off_5s, buttons) have no home in the schema and are read ONLY by the
        CAR diagnostic line -- which belongs to the panda role, because that is
        the process holding the state. They keep their constructor defaults here
        rather than being faked, so anything that starts reading them in this role
        reads an obvious zero instead of a plausible lie.
        """

        def __init__(self):
            self.v_ego = 0.0
            self.cruise_available = False
            self.cruise_enabled = False
            self.acc_armed = False
            self.acc_active = False
            self.brake_pressed = False
            self.gas_pressed = False
            self.steering_torque = 0.0
            self.steering_pressed = False
            self.steering_angle = 0.0
            self.steer_angle_rate = 0.0
            self.rpm = 0
            self.buttons = dict(set_p=0, set_m=0, res=0, off=0)
            self.eps_motor_torque = 0.0
            self.eps_request = 0
            self.eps_effective = 0
            self.lkas_block = False
            self.hands_off_5s = False
            self.lkas_track_state = 0
            self.left_blinker = False
            self.right_blinker = False
            self.left_blindspot = False
            self.right_blindspot = False
            self.seen = 0
            self.eps_seen = 0
            self.crz_btns_counter = 0

        def update_from(self, cs):
            self.v_ego = float(cs.vEgo)
            self.steering_angle = float(cs.steeringAngleDeg)
            self.steer_angle_rate = float(cs.steeringRateDeg)
            self.steering_torque = float(cs.steeringTorque)
            self.steering_pressed = bool(cs.steeringPressed)
            self.brake_pressed = bool(cs.brakePressed)
            self.gas_pressed = bool(cs.gasPressed)
            self.cruise_available = bool(cs.cruiseState.available)
            self.cruise_enabled = bool(cs.cruiseState.enabled)
            self.left_blinker = bool(cs.leftBlinker)
            self.right_blinker = bool(cs.rightBlinker)
            self.left_blindspot = bool(cs.leftBlindspot)
            self.right_blindspot = bool(cs.rightBlindspot)
            self.seen += 1

    cs_can = _CarStateFromMsgq() if role == "model" else CarStateFromCAN()
    # engagement edge tracker; see the EDGE print in the publish loop
    edge = {"prev": None, "n": 0, "last": "-"}
    diag = {"t": 0.0}   # 1 Hz CAR/EVT diagnostic tick
    radar_shadow = {}   # addr -> {n, d, t, t0}, see RADAR_IDS
    # The radar's own 0x21b/0x21c as captured BEFORE suppression, so what we send
    # can be diffed against what the PCM demonstrably obeys.
    radar_pre = {}      # addr -> {n, d, t0}
    # Follow-distance setting last seen from the radar, so the log only reports
    # changes rather than 40 lines a second.
    _last_ds = {"v": None}
    from opendbc.car.mazda import longitudinal as _mazda_long
    radar_shadow_state = {"sent": 0}   # frames replayed, surfaced in the log
    # ENGAGE-FIRST takeover gate. Open by default: every mode other than
    # OP_ALPHA_ENGAGE_FIRST transmits from the moment its thread starts, exactly
    # as before. See the takeover watcher at the arm site.
    longtx_gate = {"on": True}
    # Tester-present gate. False lets the radar's UDS session lapse so it comes
    # back with FCW/AEB/SBS. Only engage-first's re-arm loop ever drops it; every
    # other mode holds the session for the whole run exactly as before.
    radar_hold = {"on": True}

    # ---- SUPPRESSION DEAD-MAN'S SWITCH ---------------------------------------
    #
    # THE RADAR MUTE MUST FAIL OPEN. Everything else in this stack already fails
    # safe when the host loses the bus -- the panda drops to SAFETY_NOOUTPUT,
    # openpilot disengages itself -- but the radar mute was held by a TX-only
    # timer with nothing checking that the host could still see the car. So a
    # dead RX link left the car with no FCW/AEB/SBS for as long as the process
    # ran, and no input could end it, because the release that ends it is
    # triggered by RX.
    #
    # MEASURED ON THE CAR 2026-08-13 (drivelogs/cb3_panda.log):
    #
    #   t=115.5s  handover 1, radar suppressed
    #   t=134.6s  RELEASE: ACC ended, tester-present stopped
    #   t=140.6s  RELEASE COMPLETE: radar alive again after 6.0 s   <- worked
    #   t=588.5s  handover 2 at 80 kph, radar suppressed again
    #   t=676.7s  the USB feed dies: 21f=0 165=0 09d=0, whole bus gone
    #   t=940.3s  STILL suppressing. tp=11059 tester-present frames sent blind.
    #
    # 264 seconds of suppression that no brake, cancel or MAIN-off could clear,
    # and the car logged fault codes in the camera/cluster/PCM the whole time.
    #
    # THE MECHANISM. The release waits on `cs_can.acc_active`, decoded from
    # PEDALS 0x165. That is a plain assignment (dashcam.py:293) with no expiry,
    # so when the frames stopped it held True forever and the wait never ended.
    # Exactly the same shape as the set_speed_raw latch fixed earlier this
    # session: a decoded field used as a liveness condition without one.
    #
    # The rule here: a decoded VALUE can never establish liveness. Only an
    # arrival TIME can. rx_alive() is that time, and every consumer of the
    # suppression asks it.
    rx_live = RxLiveness()
    rx_alive = rx_live.alive

    # addr -> how many frames WE have put on the bus at that address. can_census
    # counts every frame can_thread sees, including the echo of our own transmits,
    # so any "is the car still sending this?" question has to subtract this first.
    _ourtx = {}
    ang_off = {"v": 0.0, "n": 0}   # learned steering angle offset, deg
    import collections as _c
    lat_trace = {"on": os.environ.get("OP_LAT_TRACE") == "1", "t": 0.0,
                 # ~220 Hz, so 60000 rows held only the last 4.5 minutes and the
                 # deque silently discarded the rest. Both lane-change attempts of
                 # the 2026-08-09 drive fell out of the window before the file
                 # could be read, leaving no way to tell whether the wheel actually
                 # responded. 900k rows is ~68 min at ~40 MB -- cheap next to
                 # losing the one event the trace exists to capture.
                 #
                 # FLUSHED INCREMENTALLY, NOT REWRITTEN. This deque now holds only
                 # the rows not yet on disk; it is drained on every flush, so the
                 # write cost is proportional to the last 10 s and not to the whole
                 # session. The maxlen is a burst guard, nothing more.
                 #
                 # It used to be the whole session's history, rewritten from
                 # scratch every 10 s with a %-format and a write() per row. That
                 # is O(n) forever on a buffer that only grows, and it runs on the
                 # main loop -- so it holds the GIL for the entire write.
                 #
                 # MEASURED 2026-08-11 in the model role at 293,002 rows / 18.7 MB:
                 # modelV2 published at its nominal 19.2 Hz (p50 52.2 ms, p95
                 # 55.1 ms) and then FROZE FOR 3.9-4.1 s, once every 10 s, six times
                 # in sixty seconds. modelV2, livePose, longitudinalPlan and
                 # liveCalibration all stalled together (same thread); carState,
                 # published from the panda role, never exceeded 20.9 ms.
                 #
                 # Downstream that reads as twelve services not_alive -> commIssue
                 # [NS] -> softDisable, which is why engagement died after a few
                 # seconds and openpilot "gave up mid curve". The giveaway was that
                 # it got worse the longer the stack ran, with nothing retuned:
                 # early in a drive the buffer is short and the write is quick, and
                 # it crosses the aliveness timeout only once enough rows pile up.
                 # At the 900k maxlen the old code would have stalled ~12 s.
                 "buf": _c.deque(maxlen=int(os.environ.get("OP_LAT_TRACE_ROWS", "900000"))),
                 # Header goes down once, when the file is created.
                 "hdr": False,
                 # NOT /tmp. The Jetson clears it on boot, and the 2026-08-09 drive
                 # -- the one carrying the first plan_y capture -- was gone before it
                 # could be read. A trace that does not survive a reboot cannot
                 # answer a question about last night.
                 # ROLE-SUFFIXED. Both roles run this same code and both were
                 # launched with OP_LAT_TRACE=1, so with one shared default path
                 # they interleaved rows into a single file -- and now that the
                 # first flush truncates, they would each throw away the other's
                 # trace on startup. An explicit OP_LAT_TRACE_PATH still wins, so
                 # a deliberate single-file capture is still possible.
                 "path": os.environ.get(
                     "OP_LAT_TRACE_PATH",
                     "/home/tran/drivelogs/lat_trace_%s.csv" % role)}
    panda_lock = threading.Lock()
    # The 0x243 stream owns the serial link. can_thread's can_recv() is a ~15.5 ms
    # round-trip and it loops with NO sleep, so it reacquires panda_lock almost
    # immediately after releasing it -- the tx thread, which needs the same lock
    # every 20 ms, was losing the race repeatedly. Measured 2026-07-30: 65% of
    # one-second windows contained a 30-39 ms gap, worst 95.5 ms, in a frame the
    # EPS expects every 10 ms. Zero panda rejections, so this is pure link
    # contention, not safety.
    #
    # tx_due is a one-way yield: the tx thread raises it before asking for the
    # lock, can_thread refuses to START a new transaction while it is up. An
    # in-flight can_recv still cannot be preempted, so the worst case becomes one
    # can_recv (~15.5 ms) instead of an unbounded pile-up of them. can_send is
    # ~1-2 ms per send, so at 70 Hz this costs can_thread ~10-15% of the link.
    tx_due = threading.Event()
    # --- deadline-aware sharing of the serial link ---------------------------
    # tx_due alone is a one-way yield: can_thread will not START a transaction
    # while the tx thread is waiting, but a can_recv already in flight cannot be
    # preempted, and it costs ~15.5 ms against a 20 ms tx period. That is exactly
    # the measured residual -- 38% of one-second windows still carry a 30-39 ms
    # hole in a stream the EPS wants every 10 ms (20 + 15.5 = 35.5).
    #
    # So publish the tx thread's next deadline and let can_thread decide whether
    # a recv FITS before it, using its own measured cost rather than a constant.
    # This is what makes a higher --lkas-hz possible at all: at 100 Hz the whole
    # period is shorter than one recv, so "yield when asked" can never be enough
    # -- the recv has to be placed, not just deferred.
    tx_sched = {
        "next_t": 0.0,      # tx thread's next send deadline (0 = not transmitting)
        "period_ms": 1000.0 / lkas_hz,
        "recv_ms": 16.0,    # EWMA of observed can_recv cost, seeded near the
                            # profiled 15.5 ms so the first decisions are sane
        "last_recv_t": 0.0,
        # 0.5 ms ticks spent waiting for a send to complete. Divided by the recv
        # count this is the mean phase wait; if it approaches the period, the
        # recv is finishing so late that the next send is already due.
        "waits": 0,
    }
    import shutil as _sh
    _du = _sh.disk_usage('/')
    free_pct = 100.0 * _du.free / _du.total
    # health_t layout, board/health.h
    health_state = {"v": None, "t": 0.0}
    # Per-CAN counters from control_read(0xc2) -- the only place the FIRMWARE's
    # own view is visible. `fwd` is the one that matters: it counts frames the
    # RX ISR forwarded to the other bus, so a flat fwd on CAN2 means the camera
    # is NOT reaching the car no matter how healthy everything on this side
    # looks. rx_lost is the panda's RX FIFO overflow, i.e. frames the serial
    # link was too slow to drain.
    can_health_state = {"v": {}, "t": 0.0}
    # Address census, per bus. cs_can decodes six addresses and cam_state two;
    # everything else the camera and the car send has been invisible. Without
    # this there is no way to answer "what does the real camera actually put on
    # bus 2, and at what rate" -- which is the reference our 0x243 stream is
    # supposed to imitate.
    can_census = {0: {}, 1: {}, 2: {}}
    # UDS response tap for the alpha-long radar suppression handshake. can_thread
    # owns the serial link -- calling panda.can_recv() from a second thread to run
    # an ISO-TP query would steal frames out of THIS loop and put holes in the
    # 0x243 stream. So the query reads from here instead: while "on", matching
    # frames are copied into the deque and the query's can_recv drains it.
    # Inert (one set-membership test per frame) whenever the handshake is not
    # running, which is all of the time except a few hundred ms at arming.
    # "seen" is a running total that the query does NOT consume, unlike "q".
    # It separates "can_thread never delivered the reply" from "it did, and
    # IsoTpParallelQuery missed it" -- two very different bugs that look
    # identical from the handshake's return value.
    uds_sniff = {"on": False, "addrs": frozenset(), "q": collections.deque(maxlen=512),
                 "seen": 0, "log": collections.deque(maxlen=16)}
    # NOT /tmp: tmpfiles.d wipes it on boot, and this box loses power in the car
    # rather than shutting down, so a drive log in /tmp never survives to be read.
    _logdir = os.path.expanduser("~/drivelogs")
    os.makedirs(_logdir, exist_ok=True)
    logf = open(os.environ.get("DASHCAM_LOG",
                               "%s/dashcam_%d.jsonl" % (_logdir, int(time.time()))), "w")
    t0 = time.time()
    frames = 0
    moving = 0.0
    last = t0
    out = {"path_xyz": None}
    curv = 0.0

    # --- decouple capture and preview from the model loop -------------------
    # Running capture -> autolevel -> inference -> 18 cereal publishes -> JPEG
    # encode -> JSONL serially pinned the pipeline at ~4.3fps, even though the
    # camera alone does 30fps and the model alone does 35fps.
    #
    # That is not just slow, it BREAKS CALIBRATION. supercombo's pose output is
    # frame-to-frame odometry expressed as m/s *at the model's native 20Hz*. At
    # 4.3fps a true 16.7m/s reads as ~3.6m/s, which fails calibrationd's
    # `trans[0] > MIN_SPEED_FILTER` (4.167) gate, so every sample is discarded --
    # 216 records above 20km/h in the last drive produced 0 calibration blocks.
    #
    # So: capture in its own thread (always holding the freshest frame), JPEG
    # encode for the phone in another at a low rate, and leave the model loop to
    # do inference on the latest frame only.
    cap = {"frame": None, "n": 0}
    cap_lock = threading.Lock()
    stop_flag = {"v": False}

    def capture_thread():
        while not stop_flag["v"]:
            fr = cam.read()
            if fr is not None:
                with cap_lock:
                    cap["frame"] = fr
                    cap["n"] += 1
            else:
                time.sleep(0.005)

    def can_thread():
        # can_recv() is a full framed round-trip over the 1.5Mbaud VCP and costs
        # ~15.5ms whether or not anything is waiting -- profiling showed it
        # returning ~0 frames per call while eating 25% of the loop budget. On the
        # critical path that alone kept the model under 20Hz. cs_can is only ever
        # written here and read (field at a time) by the model loop, so a lock is
        # not needed for correctness of individual scalars.
        # --- phase the recv to just after a send -----------------------------
        # MEASURED 2026-07-30 at 70 Hz: recv_ms 12.0 against a 14.3 ms period.
        # A recv and a send together (13.5 ms) fit inside the period with room
        # to spare, yet the first version of this scheduler produced a WORSE
        # stream than 50 Hz -- 61% of one-second windows in the 30-39 ms band
        # and 168 starvation escapes in a minute.
        #
        # The reason is that "does it fit before the next deadline" is the wrong
        # question. By the time can_thread asks, the send has already consumed
        # part of the period, so the remaining slack (12.8 ms) is smaller than a
        # recv plus any margin -- and it only ever shrinks as the deadline
        # approaches. The condition can never become true, so the recv waits out
        # the 250 ms escape and lands on top of a send anyway.
        #
        # The right question is WHEN to start, and the answer does not depend on
        # any measurement: immediately after a send is the moment of maximum
        # slack, always. So wait for the send, then go. One recv per send, rx
        # rate equals tx rate, no starvation possible while tx is running, and
        # the interval the EPS sees is period or (send + recv), whichever is
        # larger -- either way the SAME every cycle, which is the property that
        # matters. Nothing here needs recv_ms; it is kept purely as telemetry.
        while not stop_flag["v"]:
            try:
                while tx_due.is_set():
                    time.sleep(0.0005)
                # Only phase against a live tx deadline. next_t == 0 means we are
                # not armed (dashcam mode: recv freely); a deadline more than half
                # a second in the past means the tx thread is gone or wedged, and
                # waiting on it would stop rx forever.
                _after = tx_sched["next_t"]
                if _after > 0.0:
                    _sends = 0
                    while not stop_flag["v"]:
                        now_r = time.time()
                        nxt = tx_sched["next_t"]
                        # next_t is advanced only after can_send returns, so a
                        # CHANGE in it is the signal that a send just completed
                        # and the whole gap to the next one is ours.
                        if nxt <= 0.0 or (now_r - nxt) > 0.5:
                            break
                        if nxt != _after:
                            # Wait for RECV_EVERY sends, not one. One-per-send caps
                            # the achievable tx rate: a cycle costs send (~1.5 ms)
                            # plus a full recv, and MEASURED 2026-08-01 the recv is
                            # ~5.3 ms whether it collects 10 frames or 15 -- it is
                            # dominated by fixed USB transaction latency, not data.
                            # So at 100 Hz one-per-send is 6.8 ms of a 10.2 ms
                            # period = 67% link occupancy, and queueing delay goes
                            # as rho/(1-rho): 2.0 at 0.67 against 1.0 at 0.50. That
                            # is why 100 Hz produced 22 ms gaps and the EPS refused
                            # LKAS 98.7% of the time, while 70 Hz (50%) works.
                            #
                            # Recving every 2nd send at 100 Hz is 41% occupancy --
                            # better than 70 Hz is today. Nothing is lost: the panda
                            # buffers between reads and can_rx_lost0 is 0 in every
                            # log so far, so the batch just gets bigger, which is
                            # nearly free. The cost is cs_can staleness, up to
                            # RECV_EVERY periods instead of one.
                            _sends += 1
                            _after = nxt
                            if _sends >= RECV_EVERY:
                                break
                        tx_sched["waits"] += 1
                        time.sleep(0.0005)
                    # The wait above ends the instant the deadline advances, which
                    # is not necessarily after the tx thread has let go. Re-check
                    # the yield flag so we never take the lock out from under a
                    # send that is already asking for it.
                    while tx_due.is_set():
                        time.sleep(0.0005)
                _t_recv = time.time()
                with panda_lock:
                    msgs = panda.can_recv()
                # Measure what a recv actually costs on THIS link instead of
                # trusting the profiled 15.5 ms -- it moves with the frame count
                # in the batch, and the decision above is only as good as it.
                # Weighted toward the recent worst: underestimating the cost is
                # what puts a hole in the 0x243 stream, overestimating only
                # delays rx a little.
                _d_ms = (time.time() - _t_recv) * 1000.0
                tx_sched["recv_ms"] = (_d_ms if _d_ms > tx_sched["recv_ms"]
                                       else 0.85 * tx_sched["recv_ms"] + 0.15 * _d_ms)
                tx_sched["last_recv_t"] = time.time()
                if msgs:
                    # RX LIVENESS, for the suppression dead-man's switch.
                    #
                    # One timestamp, updated wherever frames actually arrive, so
                    # every consumer asks the same question: "has the bus said
                    # anything lately?" Decoded fields cannot answer it -- they
                    # are plain assignments that HOLD their last value when the
                    # frames stop, which is indistinguishable from the car still
                    # reporting that value. See rx_alive().
                    rx_live.stamp()
                for addr, d, bus in msgs:
                    # Census BEFORE the dispatch below, which only looks at the
                    # handful of addresses this stack decodes. The question this
                    # answers is what the real camera puts on bus 2 and at what
                    # rate, so our 0x243 can be compared against its source
                    # rather than against an assumption.
                    if bus in can_census:
                        can_census[bus][addr] = can_census[bus].get(addr, 0) + 1
                    # RADAR SHADOW CAPTURE. Suppressing the radar over UDS silences
                    # every frame it sends, not just 0x21b/0x21c, and these are
                    # normally forwarded bus 0 -> bus 2 to the camera. Record the
                    # last payload and a count for each so we can (a) confirm they
                    # really do go silent under suppression and (b) replay the
                    # static ones to keep the camera satisfied. See RADAR_IDS.
                    # THE RADAR'S OWN 0x21b/0x21c, kept for comparison against ours.
                    #
                    # These are the two frames alpha long replaces, and the PCM obeys
                    # the radar's copies while ignoring ours -- acc_active never
                    # reached 1 in any run, so ACCEL_CMD was never acted on at any
                    # speed. The obvious question is how our bytes differ from the
                    # radar's, and there has never been a capture to answer it: the
                    # shadow records the OTHER seven frames and not these two.
                    #
                    # Captured before suppression only (afterwards 0x21b on bus 0 is
                    # our own echo). "pre" is the last radar-authored payload.
                    if addr in (0x21B, 0x21C) and len(d) == 8 and not tx_state.get("long_tx_frames"):
                        _p = radar_pre.setdefault(addr, {"n": 0, "d": b"", "t0": time.time()})
                        _p["n"] += 1
                        _p["d"] = bytes(d)
                        # Learn the driver's follow-distance setting from the
                        # radar's own CRZ_CTRL, so our replacement frame carries
                        # the setting the cluster is already displaying instead of
                        # the template's hardcoded 2. MEASURED: that single field
                        # was the ONLY difference between the radar's 0x21c and
                        # ours (0x13 vs 0x0b), and the front camera warning began
                        # flickering exactly when our frame took over.
                        if addr == 0x21B:
                            # Same reasoning as 0x21c below: CRZ_INFO_TEMPLATE is a
                            # hardcoded guess whose static bytes (2, 5, 6) do not
                            # match this car, and every frame we send carries them.
                            try:
                                _mazda_long.set_observed_crz_info(bytes(d))
                            except Exception:
                                pass
                        if addr == 0x21C:
                            try:
                                # Remember the radar's byte 2 for whichever state
                                # it is reporting, so we can reproduce the right
                                # one instead of a static template.
                                _mazda_long.set_observed_crz_ctrl(bytes(d))
                                _ds = _mazda_long.distance_setting_from_frame(bytes(d))
                                # ZERO IS NOT A SETTING. MEASURED 2026-08-11: the
                                # radar reports DISTANCE_SETTING=0 with cruise off
                                # and 4 once it is set, so the field tracks cruise
                                # state, not just the driver's bars. Adopting the
                                # last value seen would hand the cluster a 0 -- a
                                # different wrong answer than the template's 2.
                                # Keep the highest real value observed; that is the
                                # driver's actual selection.
                                if not _ds:
                                    _ds = _last_ds["v"]
                                elif _last_ds["v"] and _ds < _last_ds["v"]:
                                    _ds = _last_ds["v"]
                                if _ds and _ds != _last_ds["v"]:
                                    _last_ds["v"] = _ds
                                    _mazda_long.set_observed_distance_setting(_ds)
                                    print("radar DISTANCE_SETTING observed = %s "
                                          "(our CRZ_CTRL will match it)" % _ds,
                                          flush=True)
                            except Exception:
                                pass
                    if addr in RADAR_IDS and len(d) == 8:
                        # t0 is first-seen, so the replay can derive each frame's
                        # real rate as (n-1)/(t-t0) rather than guessing. Guessing
                        # is not harmless: send these faster than the radar did and
                        # the bus gains load that was never there; slower, and a
                        # consumer watching for a timeout still sees one.
                        _now_r = time.time()
                        _r = radar_shadow.setdefault(
                            addr, {"n": 0, "d": b"", "t": 0.0, "t0": _now_r})
                        _r["n"] += 1; _r["d"] = bytes(d); _r["t"] = _now_r
                    if uds_sniff["on"] and addr in uds_sniff["addrs"]:
                        uds_sniff["q"].append((addr, bytes(d), bus))
                        uds_sniff["seen"] += 1
                        uds_sniff["log"].append((round(time.time(), 3), addr, bytes(d).hex()))
                    if bus == 0:
                        cs_can.update(addr, d)
                        # Sample the EPS pair at the moment of the request peak,
                        # so eff is the value that ACCOMPANIED the biggest ask.
                        if addr == 0x241 and abs(cs_can.eps_request) > eps_peak["req"]:
                            eps_peak["req"] = abs(cs_can.eps_request)
                            eps_peak["eff"] = abs(cs_can.eps_effective)
                    elif bus == 2 and addr == CAM_LKAS_ADDR and len(d) == 8:
                        # live camera LKAS state bits -> copied into our tx frame
                        cam_state["BIT_1"] = (d[3] >> 5) & 1
                        cam_state["ERR_BIT_1"] = d[2] & 1
                        cam_state["ERR_BIT_2"] = (d[3] >> 6) & 1
                        # Lane bits, captured for OP_LKAS_COPY_CAM_LINES. Same bit
                        # positions the ck validator uses. Without these the tx dict
                        # is missing the keys create_steering_control reads under
                        # that flag, and tx_thread dies on frame one -- see the note
                        # where cam_bits is built.
                        cam_state["LINE_NOT_VISIBLE"] = (d[2] >> 3) & 1
                        cam_state["LDW"] = (d[2] >> 7) & 1
                        cam_state["seen"] += 1
                        cam_state["last_t"] = time.time()
                        # --- is 0x243 arriving whole, or are we seeing a slice?
                        # The camera is observed at ~16 Hz here while we imitate
                        # it at 50, and the two readings of that are opposite:
                        # either the camera really runs at 16 Hz (so OUR stream
                        # is 3x too fast, counter and all) or we are dropping 5
                        # of every 6 frames somewhere between the CAN core and
                        # this thread (so the rate is fine and the reading is an
                        # artefact). CTR settles it without any new plumbing: it
                        # is the camera's OWN 4-bit sequence number, so a delta
                        # of 1 per received frame means we have them all, and a
                        # delta of ~6 means we are looking at every sixth one.
                        _c = d[0] >> 4
                        if cam_state["ctr_last"] >= 0:
                            _dc = (_c - cam_state["ctr_last"]) & 0xF
                            cam_state["ctr_d"][_dc] = cam_state["ctr_d"].get(_dc, 0) + 1
                        cam_state["ctr_last"] = _c
                        _validate_cam_checksum(d)
                    elif bus == 2 and addr == CAM_LANEINFO_ADDR and len(d) == 8:
                        # CAM_LANEINFO (lane state / LKAS HUD). NOT relayed here any
                        # more -- the panda firmware forwards it cam->car itself, via
                        # .disable_static_blocking on the 0x440 entry in the Mazda
                        # safety mode's tx table (opendbc/safety/modes/mazda.h, behind
                        # PANDA_NUCLEO). Doing it in the RX ISR instead of over the
                        # serial link removes ~16 ms of latency, stops the batch
                        # coalescing that dropped every 0x440 but the last in a poll,
                        # keeps the camera's native cadence, and frees the panda_lock
                        # contention that was jittering the 0x243 stream.
                        # This branch now only observes it for the dashboard.
                        cam_state["lane_seen"] = cam_state.get("lane_seen", 0) + 1
                        cam_state["lane_t"] = time.time()
                # Refresh panda health ~1Hz so pandaStates can be published with
                # real values instead of an empty message (an empty one is what
                # made selfdrived raise usbError).
                now_h = time.time()
                # Timed off can_health_state["t"], NOT health_state["t"]: once the
                # tx thread is running it refreshes health_state at 10 Hz for its
                # own rejection check, so `now - health_state["t"] > 1.0` is never
                # true again and everything nested under it runs exactly once.
                # That is what left can_fwd2 / rx_lost frozen at their first
                # sample for a whole run -- the counters were there and the
                # differences were all zero.
                if now_h - can_health_state["t"] > 1.0:
                    can_health_state["t"] = now_h
                    while tx_due.is_set():
                        time.sleep(0.0005)
                    with panda_lock:
                        h = panda.control_read(0xd2, 0, 0, 64)
                    if h and len(h) >= struct.calcsize(HEALTH_FMT):
                        health_state["v"] = struct.unpack(HEALTH_FMT, h[:struct.calcsize(HEALTH_FMT)])
                        health_state["t"] = now_h
                    # Per-CAN counters, same 1 Hz slot. NOT panda.can_health():
                    # that retries with reset_input_buffer(), which would flush
                    # CAN frames already queued on the link, and sleeps 20 ms
                    # inside the lock the tx stream is waiting on. One read, no
                    # retry, no side effects -- a miss just leaves the previous
                    # sample standing for another second.
                    for _bus, _n in ((0, 0), (2, 1)):
                        while tx_due.is_set():
                            time.sleep(0.0005)
                        with panda_lock:
                            _d = panda.control_read(0xc2, _n, 0, 64)
                        if _d and len(_d) >= 64:
                            _f = struct.unpack("<BIBBBBBBBBIIIIIIIHHBBBIIII", _d[:64])
                            can_health_state["v"][_bus] = {
                                "bus_off": _f[0], "rx_err": _f[8], "tx_err": _f[9],
                                "total_err": _f[10], "tx_lost": _f[11],
                                # rx FIFO overflow: frames the CAN core received
                                # and the serial link was too slow to drain.
                                "rx_lost": _f[12],
                                "tx": _f[13], "rx": _f[14],
                                # frames the RX ISR relayed to the other bus.
                                # On CAN2 this IS the camera -> car path.
                                "fwd": _f[15]}

                # RATE-LIMIT THE POLL -- this loop used to be self-limiting.
                #
                # The no-sleep design above is correct for the link it was written
                # for: on the 1.5 Mbaud ST-Link VCP can_recv() was a ~15.5 ms framed
                # round-trip, so the loop could only run ~65 times a second no matter
                # how hard it spun. The move to native USB deleted that limiter
                # without anyone noticing -- MEASURED 2026-08-09: can_recv() p50 is
                # 0.18 ms, so the identical loop now runs ~5000 times a second,
                # holding the GIL continuously in pure Python.
                #
                # That is what stops the 0x243 stream from keeping 100 Hz, and it is
                # not the panda: a 10 ms deadline is held to +-0.2 ms (p99 10.02 ms)
                # against ONE competing CPU-bound thread, and collapses to p99 442 ms
                # / max 770 ms against THREE. can_recv, frame parsing (2.75 us per
                # frame) and capnp message building together account for under 4% of
                # a 10 ms period, so the link has nothing to do with it.
                #
                # Sleeping 2 ms between polls turns ~5000 Hz into ~500 Hz. At the
                # ~1400 frame/s this bus actually carries that is ~3 frames per call
                # and ~8 us of parsing, and 500 polls/s x 1170 frames per 16 KiB read
                # is orders of magnitude more capacity than the bus can produce --
                # so nothing is dropped, the thread stops holding the GIL, and the
                # rx_buffer_overflow headroom is unaffected.
                #
                # Skip the sleep when a real backlog shows up (a burst after a
                # scheduling hiccup, or the queue filling) so draining stays fast --
                # falling behind is what overflows can_rx_q, and that failure mode
                # freezes carState.
                if len(msgs) < 8:
                    time.sleep(0.002)
            except Exception:
                time.sleep(0.01)

    def preview_thread():
        # The phone does not need 30fps. Encoding every frame was pure overhead
        # on the critical path.
        while not stop_flag["v"]:
            with cap_lock:
                fr = cap["frame"]
            if fr is not None:
                with LOCK:
                    snap = dict(STATE)
                    snap["_path"] = STATE.get("_path")
                    snap["_model"] = STATE.get("_model")
                    snap["_dets"] = STATE.get("_dets")
                try:
                    # Halve before drawing AND encoding. A phone does not resolve
                    # 1080p, and this thread shares the GIL with the model thread --
                    # quartering the pixel count takes the overlay blend from ~12 ms
                    # to ~3 ms and shrinks the JPEG by the same factor. The model
                    # still runs on the full-resolution frame; this is display only.
                    sh, sw = fr.shape[0] // 2, fr.shape[1] // 2
                    small = cv2.resize(fr, (sw, sh), interpolation=cv2.INTER_AREA)
                    ok, jpg = cv2.imencode(".jpg",
                                           overlay(small, snap, (fr.shape[1], fr.shape[0])),
                                           [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                    if ok:
                        with LOCK:
                            FRAME_JPEG["buf"] = jpg.tobytes()
                            FRAME_JPEG["seq"] += 1
                except Exception:
                    pass
            time.sleep(0.15)      # ~6fps preview

    def det_thread():
        """YOLO26n at ~5 Hz on the freshest camera frame.

        Measured on this board: 11.9 ms standalone, and interleaved 4:1 with
        supercombo it costs ~2.8 ms per model step (31.3 vs 28.5 ms), leaving
        ~19 ms of the 50 ms budget at 20 Hz. Detection only -- boxes stay in
        image space until stage 3 brackets them against the lane lines.
        """
        try:
            from yolo_trt import YoloRunner
            det = YoloRunner()
        except Exception as e:
            print("yolo_trt unavailable, detection disabled:", e)
            return
        print("detector ready:", getattr(det, "conf", "?"), "conf threshold")
        while not stop_flag["v"]:
            t0 = time.time()
            with cap_lock:
                fr = cap["frame"]
            if fr is not None:
                try:
                    ds = det.step(fr)
                    packed = [{"cls": d.cls_name, "conf": round(d.conf, 3),
                               "box": [round(v, 1) for v in d.box],
                               "trunc": d.truncated} for d in ds]
                    with LOCK:
                        STATE["_dets"] = packed
                        STATE["_dets_seq"] = STATE.get("_dets_seq", 0) + 1
                        STATE["n_dets"] = len(packed)
                except Exception:
                    pass
            time.sleep(max(0.0, 0.20 - (time.time() - t0)))    # ~5 Hz

    def seg_thread():
        """b2-vistas road segmentation, slow and on its own thread.

        44 ms a frame -- 3.5x YOLO -- so it runs at 3 Hz, not 5. It can afford to:
        the road is ground-fixed, so unlike a moving car the layer does not have to
        be fresh to be right, only recent. The class map is published in IMAGE
        space; turning it into ground geometry is scene_thread's job, and happens
        at the scene rate against that frame's lane lines.

        The staleness is real and not hidden: at 3 Hz and 20 m/s the road texture
        steps ~7 m between updates. It shows most at speed on a straight and least
        in town, which is where the crosswalks and markings actually matter.
        """
        try:
            from road_seg import SegRunner, LABELS as _SEGL
            from temporal import ClassEMA
            seg = SegRunner()
            # Smooth at the SEG rate, not the render rate: this filters successive
            # network outputs, and stepping it more often than the network produces
            # would just decay the history between real observations.
            csm = None if getattr(args, "no_smooth", False) else ClassEMA(len(_SEGL))
        except Exception as e:
            print("road_seg unavailable, ground layer disabled:", e)
            return
        period = 1.0 / max(float(getattr(args, "seg_hz", 3.0)), 0.2)
        print("road segmentation ready: %dx%d -> %d classes at %.1f Hz"
              % (seg.oh, seg.ow, seg.n_cls, 1.0 / period))
        while not stop_flag["v"]:
            t0 = time.time()
            with cap_lock:
                fr = cap["frame"]
            if fr is not None:
                try:
                    cls = seg.step(fr)
                    if HOOD is not None:
                        # before smoothing: the bonnet is not evidence about anything,
                        # so it must not enter the class history either
                        cls = HOOD.blank(cls, (fr.shape[1], fr.shape[0]), len(_SEGL))
                    if csm is not None:
                        cls = csm.step(cls)
                    with LOCK:
                        STATE["_seg"] = cls
                        STATE["_seg_seq"] = STATE.get("_seg_seq", 0) + 1
                        STATE["seg_ms"] = round((time.time() - t0) * 1000.0, 1)
                except Exception:
                    pass
            time.sleep(max(0.0, period - (time.time() - t0)))

    def depth_thread():
        """Depth Anything V2 metric, on its own thread, deliberately slow.

        A DISPLAY pane and nothing else -- see depth_trt's docstring for why it is
        not fused into the geometry. It is the fourth network on this GPU, so it runs
        at the lowest rate of any of them and is the first thing to turn off.
        """
        try:
            from depth_trt import DepthRunner
            dr = DepthRunner()
        except Exception as e:
            print("depth unavailable, depth pane disabled:", e)
            return
        period = 1.0 / max(float(getattr(args, "depth_hz", 2.0)), 0.2)
        print("depth ready: %dx%d metric, max %.0f m, at %.1f Hz"
              % (dr.ow, dr.oh, 80.0, 1.0 / period))
        while not stop_flag["v"]:
            t0 = time.time()
            with cap_lock:
                fr = cap["frame"]
            if fr is not None:
                try:
                    m = dr.step(fr)
                    vis = dr.colourise(m)
                    with LOCK:
                        DEPTH_FRAME["img"] = vis
                        DEPTH_FRAME["raw"] = m          # scene_thread fits scale on this
                        DEPTH_FRAME["seq"] += 1
                        STATE["depth_ms"] = round((time.time() - t0) * 1000.0, 1)
                        # bonnet excluded: it reads as road 2x too far and would
                        # dominate any near-field statistic
                        v = HOOD.valid(m.shape, (fr.shape[1], fr.shape[0])) \
                            if HOOD is not None else None
                        mm = m[v] if v is not None else m
                        STATE["depth_near_m"] = round(float(np.percentile(mm, 2)), 1)
                except Exception:
                    pass
            time.sleep(max(0.0, period - (time.time() - t0)))

    def scene_thread():
        """Tesla-style 3D view: model road geometry + placed detections.

        Stage 1 drew the model's own output. Stage 3 adds every vehicle YOLO sees,
        placed in the ego frame by bracketing each box against the projected lane
        lines (lane_place.RoadFrame) rather than by back-projecting through the
        intrinsics -- so a car between two lane lines in the image renders between
        those same lines here, and nothing floats.

        Display only. Nothing downstream reads any of this.
        """
        try:
            from tesla_view import TeslaView, Obj
            from lane_place import RoadFrame, build_lines, fit_ground
            from road_seg import (line_agreement, BOUNDARY, box_support, SUPPORT,
                                  GROUND as SEG_GROUND)
            from yolo_trt import MESH_FOR
        except Exception as e:
            print("tesla_view/lane_place unavailable, /scene.mjpg disabled:", e)
            return
        # --display scene fills a 16:9 window, so render 16:9 and upscale; the
        # web pane and the PiP inset are small enough to stay at 640x400.
        sw, sh = (960, 540) if getattr(args, "display_view", "pip") == "scene" \
                 and use_display else (640, 400)
        view = TeslaView(w=sw, h=sh, ss=1)       # ss=1: ~2x cheaper, slightly jaggier
        ground_cache = {"v": None}
        smooth = not getattr(args, "no_smooth", False)
        pe_lane = pe_edge = pe_path = trk = None
        if smooth:
            from temporal import PolyEMA, Tracker
            pe_lane, pe_edge, pe_path = PolyEMA(), PolyEMA(), PolyEMA()
            trk = Tracker()
        try:
            from depth_trt import ScaleFit
            dscale = ScaleFit()
        except Exception:
            dscale = None
        # what counts as "the road surface" for a depth anchor: every ground class,
        # not just the drivable ones. A lane vertex landing on the verge or the
        # kerb is still ON the ground plane at the distance it claims, which is all
        # the fit needs -- restricting to drivable would throw away good anchors.
        DRIVABLE_IDS = tuple(SEG_GROUND.keys())
        last_det_seq = [None]
        last_upd = [time.time()]
        gr = None
        if not getattr(args, "no_roadseg", False):
            try:
                from road_seg import GroundRaster
                gr = GroundRaster()
            except Exception as e:
                print("GroundRaster unavailable, ground layer disabled:", e)

        # --- free space and orientation (occupancy.py).
        # Built on the SAME GroundRaster grid the surface is drawn from, so the
        # cells that clip the path are the cells you can see. Disabled without a
        # ground raster because there would be nothing to build it on.
        fspace = ygate = footprint_yaw = vfield = None
        if gr is not None and not getattr(args, "no_freespace", False):
            try:
                from occupancy import FreeSpace, YawGate, footprint_yaw
                fspace = FreeSpace(gr)
                ygate = YawGate()
                print("free space ready: %dx%d cells at %.2f m, path clipping on"
                      % (gr.nr, gr.nc, gr.res))
            except Exception as e:
                print("occupancy unavailable, path clipping disabled:", e)
        if not getattr(args, "no_voxels", False):
            try:
                from vertical import VoxelField
                vfield = VoxelField()
                print("voxel field ready: %dx%d cells at %.2f m, %.2f m tall steps"
                      % (vfield.nx, vfield.ny, vfield.res_xy, vfield.res_z))
            except Exception as e:
                print("vertical unavailable, voxel field disabled:", e)
        last_scene = [time.time()]

        while not stop_flag["v"]:
            t0 = time.time()
            with LOCK:
                m, dets = STATE.get("_model"), STATE.get("_dets")
                det_seq = STATE.get("_dets_seq")
            with cap_lock:
                fr = cap["frame"]
            if m and fr is not None:
                try:
                    src_h, src_w = fr.shape[:2]
                    rpy = STATE.get("calib_rpy") or (0.0, 0.0, 0.0)
                    lanes = m.get("lanes")
                    edges = m.get("edges")
                    probs = m.get("lane_prob")
                    if probs is not None and lanes is not None and len(probs) >= 2 * len(lanes):
                        # lane_lines_prob packs TWO values per line; openpilot's
                        # fill_model_msg reads [1::2]. Same as draw_model().
                        probs = np.asarray(probs)[1::2]

                    lanes_xyz = [_yz_to_xyz(np.asarray(l, np.float32)) for l in lanes] if lanes is not None else []
                    edges_xyz = [_yz_to_xyz(np.asarray(e, np.float32)) for e in edges] if edges is not None else []
                    if smooth:
                        # Filter the geometry BEFORE it is used, so the placement
                        # ruler, the ground layer and the drawn lines all agree --
                        # smoothing only the drawn output would leave cars bracketed
                        # against unsmoothed lines and reintroduce the jitter.
                        lanes_xyz = pe_lane.step(lanes_xyz) or []
                        edges_xyz = pe_edge.step(edges_xyz) or []
                    z_road = _road_z(lanes_xyz + edges_xyz)

                    # --- placement frame, in SOURCE pixels (where the boxes are) ---
                    objs = []
                    meas = []              # raw placements, before the tracker
                    ground = None
                    # default when there is no segmentation to consult: every line,
                    # exactly the old behaviour
                    draw_lanes, draw_probs = lanes_xyz, probs
                    edge_ok = None
                    seg_gate = "no seg"
                    n_unsupported = 0
                    n_hood_rejected = 0
                    n_lanes_drawn = len(lanes_xyz)
                    # seg_cls is read by the detection loop below but was only ever
                    # ASSIGNED inside `if gr is not None`. With the ground raster
                    # disabled -- --no-roadseg, or a GroundRaster that failed to
                    # construct -- the first detection raised NameError into this
                    # thread's bare except, so the scene silently stopped updating
                    # and looked like a frozen renderer rather than a missing name.
                    seg_cls = None
                    # Same trap as seg_cls above, and worth stating once for all
                    # three: everything inside `if _PROJ_OK and (lanes or edges)`
                    # is read by the tracker and voxel blocks that sit OUTSIDE it.
                    # Without lanes -- an unpaved road, a car park, a failed
                    # projection -- those reads raise NameError into this thread's
                    # bare except and the scene silently freezes. Initialise here.
                    rf = None
                    dcorr = None
                    # Where detections go when they do not become objects. n_placed
                    # was reading 0 against n_dets 1 with every existing counter also
                    # at 0, which narrowed nothing down: the four ways to lose a
                    # detection were all invisible.
                    n_nomesh = n_trunc = n_unplaced = n_yaw = 0
                    if _PROJ_OK and (lanes_xyz or edges_xyz):
                        def project(xyz):
                            return _project(np.asarray(xyz, np.float32), src_w, src_h,
                                            rpy, (src_w, src_h))
                        lines = build_lines(lanes_xyz, probs, edges_xyz, project)
                        g = fit_ground(lanes_xyz + edges_xyz)
                        if g is not None:
                            K = _intrinsics(src_w, src_h, src_w, src_h)
                            V = get_view_frame_from_calib_frame(rpy[0], rpy[1], rpy[2], 0.0)
                            ground_cache["v"] = (g[0], g[1], np.asarray(V[:, :3], np.float64),
                                                 np.linalg.inv(K))
                        rf = RoadFrame(lines, ground=ground_cache["v"])

                        # --- depth scale, fitted on the lane anchors.
                        # The model's absolute scale is ~2x out on this camera, which
                        # is fatal for the absolute comparison free-space carving
                        # needs. The lane vertices have known distances, so they are
                        # free anchors -- and they are also why this lives here rather
                        # than in depth_thread, which has no geometry.
                        if dscale is not None:
                            with LOCK:
                                draw = DEPTH_FRAME.get("raw")
                            if draw is not None:
                                # seg_cls is read here BEFORE the ground-raster
                                # block assigns it, so take it directly: the anchor
                                # filter is the difference between a 50% residual
                                # and a 5% one, and it must not depend on where in
                                # this function the raster happens to be built.
                                with LOCK:
                                    _sc = STATE.get("_seg")
                                fit = dscale.step(draw, rf.lines, (src_w, src_h),
                                                  z_road, hood=HOOD, seg=_sc,
                                                  ground=DRIVABLE_IDS)
                                with LOCK:
                                    STATE["depth_scale"] = (
                                        "%s a=%.3f b=%.4f resid=%.0f%% n=%d"
                                        % (dscale.mode, dscale.a, dscale.b,
                                           100 * dscale.resid, dscale.n)
                                        if fit else "no fit (n=%d, needs lane anchors)"
                                        % dscale.n)

                        # --- road surface, on the SAME lane-line ruler as the cars.
                        # Built here rather than in seg_thread so it is bracketed
                        # against THIS frame's lane lines: the texture and the
                        # vehicles then share one geometry and cannot slide apart.
                        if gr is not None:
                            with LOCK:
                                seg_cls = STATE.get("_seg")
                            if seg_cls is not None:
                                # Agreement FIRST, because it decides whether the
                                # ground layer may be built at all.
                                ag = line_agreement(rf.lines, seg_cls, (src_w, src_h))
                                eg_lines = [l for l in rf.lines if l.kind == 'edge']
                                eg = line_agreement(eg_lines, seg_cls, (src_w, src_h),
                                                    want=BOUNDARY)
                                # per-line corroboration, lanes vs paint and edges vs
                                # boundary, each judged against what it should sit on
                                verdict = {}
                                for l, (sc, n_) in zip(rf.lines, ag):
                                    if l.kind == 'lane':
                                        verdict[(l.kind, l.idx)] = (sc, n_)
                                for l, (sc, n_) in zip(eg_lines, eg):
                                    verdict[(l.kind, l.idx)] = (sc, n_)
                                scorable = sum(1 for sc, n_ in verdict.values() if n_ >= 4)
                                agreed = sum(1 for sc, n_ in verdict.values()
                                             if n_ >= 4 and sc >= SEG_AGREE)

                                # THE GATE. The layer's whole claim is that it is
                                # placed on the model's road geometry; that claim is
                                # only worth anything if the geometry is real. Two
                                # corroborated lines is the minimum because _lane_map
                                # needs two to bracket a cell -- with fewer, every
                                # cell is either bracketed against a fabrication or
                                # extrapolated off a pair of them.
                                #
                                # This is the hole that produced the parking-lot
                                # render: every lane line correctly dropped out at
                                # lane_prob 0.10, but build_lines hardcodes road
                                # edges to prob 1.0, so two meaningless edges sailed
                                # through and became the sole basis for the surface,
                                # the 35 stray quads and every car position. Judging
                                # edges by agreement instead of by that constant is
                                # the fix.
                                #
                                # scorable >= 2 guards it: if the seg had nothing to
                                # say about any line, that is silence, not
                                # disagreement, and we keep the old behaviour.
                                # Switch the METHOD, never the layer. An earlier
                                # version set ground=None here, which threw away a
                                # perfectly good segmentation -- a parking lot really
                                # is car-road -- in order to hide the fact that its
                                # POSITION came from meaningless lane lines. Fall back
                                # to IPM instead, exactly as place() does for cars.
                                use_lanes = not (scorable >= 2 and agreed < 2)
                                ground = gr.build(seg_cls, rf, project, z_road,
                                                  (src_w, src_h), use_lanes=use_lanes)
                                seg_gate = "%s %d/%d agreed" % (
                                    "lanes" if use_lanes else "IPM", agreed, scorable)
                                # --- which lane lines are actually THERE.
                                # supercombo always emits four, so on a two-lane road
                                # two of them are fabrications. The segmentation looks
                                # at the pixels, so it is the primary source: a model
                                # line is drawn only where the two agree. Silence
                                # (line out of frame or too far to resolve) falls back
                                # to lane_prob rather than counting as disagreement.
                                #
                                # This filters DRAWING only. rf keeps every line,
                                # because the placement ruler wants all the geometry
                                # it can get -- an unpainted line is still a correct
                                # lane boundary to bracket a car against.
                                def _draw_line(kind, j, p):
                                    sc, n_ = verdict.get((kind, j), (0.0, 0))
                                    if n_ < 4:
                                        return p >= 0.30      # seg said nothing
                                    return sc >= SEG_AGREE
                                keep_lane = [j for j in range(len(lanes_xyz))
                                             if _draw_line('lane', j, float(probs[j])
                                                           if probs is not None
                                                           and j < len(probs) else 1.0)]
                                draw_lanes = [lanes_xyz[j] for j in keep_lane]
                                draw_probs = ([float(probs[j]) for j in keep_lane]
                                              if probs is not None else None)
                                n_lanes_drawn = len(keep_lane)
                                # RAILS only. Do not filter the edges themselves --
                                # tesla_view uses them to bound the drivable fill,
                                # and dropping them deletes the road surface.
                                edge_ok = [bool(n_ < 4 or sc >= SEG_AGREE)
                                           for sc, n_ in eg]

                        # the depth map, corrected onto the lane ruler, for yaw
                        dcorr = None
                        if footprint_yaw is not None and dscale is not None \
                                and dscale.a is not None:
                            with LOCK:
                                _raw = DEPTH_FRAME.get("raw")
                            if _raw is not None:
                                dcorr = dscale.apply(_raw)

                        for d in (dets or []):
                            mesh = MESH_FOR.get(d["cls"])
                            if mesh is None:
                                # traffic light / stop sign: nothing to draw in 3D
                                n_nomesh += 1
                                continue
                            if d["trunc"]:
                                # the box is cut off by the frame edge, so its bottom
                                # is not the ground contact point and the range would
                                # be wrong
                                n_trunc += 1
                                continue
                            x1, y1, x2, y2 = d["box"]
                            v_contact = y2
                            if HOOD is not None and HOOD.contains(0.5 * (x1 + x2), y2,
                                                                  (src_w, src_h)):
                                # the bonnet IS a car, so YOLO boxes it and the
                                # segmentation confirms it -- two sources agreeing on
                                # a phantom obstacle at 0 m. Geometry is the only
                                # thing that can reject it.
                                n_hood_rejected += 1
                                continue

                            # --- second opinion from the segmentation.
                            # The seg cannot make vehicle meshes -- one `vehicle`
                            # class, no instances, so a row of parked cars is a
                            # single blob and its pixels are not on the road plane.
                            # It can still do two useful things for a box YOLO
                            # already found, and ignoring them was waste.
                            if seg_cls is not None:
                                want = SUPPORT.get(d["cls"])
                                if want is not None:
                                    frac, v_seg, n_cells = box_support(
                                        seg_cls, (x1, y1, x2, y2), (src_w, src_h), want)
                                    if n_cells >= 4:
                                        if frac < SEG_BOX_MIN:
                                            # nothing of that class inside the box
                                            n_unsupported += 1
                                            continue
                                        if v_seg is not None:
                                            # the bottom of the MASK beats the bottom
                                            # of the BOX: a box includes bumper
                                            # overhang and shadow below the tyres, and
                                            # in Stage 3 vertical pixels are range
                                            v_contact = v_seg
                                    # n_cells < 4: the box is smaller than the seg
                                    # grid resolves (~20x11 px per cell, so anything
                                    # past ~40 m). No opinion, not disagreement.

                            pl = rf.place(0.5 * (x1 + x2), v_contact)
                            if pl is None:
                                n_unplaced += 1
                                continue
                            # model frame -> view frame: y and yaw both flip sign.
                            # pl.yaw is the LANE TANGENT; the depth-measured heading
                            # is applied after the tracker, which is the only place a
                            # stable identity to gate it on exists.
                            meas.append((mesh, pl.x, -pl.y, -pl.yaw,
                                         (x1, y1, x2, y2)))

                    # --- temporal filter on the placed objects.
                    # predict every render frame, correct only when the detector has
                    # actually produced something new; see Tracker.predict.
                    if trk is not None:
                        now = time.time()
                        trk.predict(0.125)
                        if det_seq != last_det_seq[0]:
                            trk.update(meas, max(now - last_upd[0], 1e-3))
                            last_det_seq[0] = det_seq
                            last_upd[0] = now
                        shown = trk.drawn()

                        # --- heading from the depth footprint, per TRACK.
                        # Here rather than in the detection loop because the gate
                        # needs a stable identity and the tracker is what supplies
                        # one. Keying on a quantised position instead looked simpler
                        # and was wrong twice over: the mesh class and the detector
                        # class are not the same string (MESH_FOR maps bicycle ->
                        # cycle), and a car crossing a bucket boundary would lose its
                        # accumulated agreement and fall back to the tangent.
                        #
                        # t.yaw is the lane tangent, carried through the tracker. It
                        # is the right answer for a vehicle travelling along the road
                        # and wrong for the cases worth seeing -- turning, parked
                        # askew, crossing -- which is the whole reason to measure.
                        yaws = {}
                        if dcorr is not None and seg_cls is not None:
                            # 3.4 ms each, measured. Capped and nearest-first so a
                            # crowded frame degrades by measuring fewer cars rather
                            # than by blowing the 125 ms render budget -- and the
                            # ones dropped are the far ones, which are the ones the
                            # fit would refuse anyway.
                            for t in sorted(shown, key=lambda t: abs(t.x))[:YAW_MAX_OBJ]:
                                want = SUPPORT.get(t.cls) or SUPPORT.get('car')
                                pl_ = _AsPlacement(t.x, -t.y, -t.yaw)
                                fy = footprint_yaw(dcorr, t.box, seg_cls,
                                                   (src_w, src_h), pl_, rf,
                                                   mode=dscale.mode, hood=HOOD,
                                                   want=want)
                                # measure in MODEL sense, use in VIEW sense
                                y_, used = ygate.step(
                                    t.id, fy[0] if fy else None, fallback=-t.yaw)
                                yaws[t.id] = -y_
                                if used:
                                    n_yaw += 1
                            ygate.drop({t.id for t in shown})
                        objs = [Obj(t.cls, t.x, t.y, yaws.get(t.id, t.yaw))
                                for t in shown]
                        with LOCK:
                            # the 2D overlay draws the SMOOTHED boxes too, so the box
                            # and the mesh cannot disagree on screen
                            STATE["_dets_smooth"] = [
                                {"cls": t.cls, "conf": 1.0, "trunc": False,
                                 "box": [round(float(b), 1) for b in t.box]}
                                for t in shown]
                    else:
                        objs = [Obj(c, x, y, yw) for c, x, y, yw, _ in meas]

                    # --- leads, only where the detector found nothing near them ---
                    lead, lprob = m.get("lead"), m.get("lead_prob")
                    if lead is not None and lprob is not None:
                        for i in range(min(2, len(lead))):
                            if float(lprob[i]) <= 0.5:
                                continue
                            lx, ly = float(lead[i][0][0]), float(lead[i][0][1])
                            if not (2.0 < lx < 160.0):
                                continue
                            vy = -ly
                            if any(abs(o.x - lx) < 6.0 and abs(o.y - vy) < 2.0 for o in objs):
                                continue          # the detector already drew this car
                            objs.append(Obj('car', lx, vy, -_path_yaw_at(m.get("path"), lx)))

                    path = m.get("path")
                    if smooth and path is not None:
                        sp = pe_path.step([np.asarray(path, np.float32)])
                        path = sp[0] if sp else path
                    path_v = _to_view(np.asarray(path, np.float32), z_road) \
                        if path is not None else None

                    # --- free space, and the path clipped against it.
                    # Run AFTER the tracker so the objects stamped into the grid are
                    # the smoothed ones being drawn -- stamping raw placements would
                    # put a footprint where no mesh is.
                    clip_x = None
                    if fspace is not None:
                        now_ = time.time()
                        dt_ = min(max(now_ - last_scene[0], 1e-3), 1.0)
                        last_scene[0] = now_
                        try:
                            with LOCK:
                                draw_ = DEPTH_FRAME.get("raw")
                            fspace.step(ground, depth=draw_,
                                        scale=dscale, objects=objs,
                                        src_wh=(src_w, src_h), hood=HOOD,
                                        v_ego=STATE.get("v_ego_kph", 0.0) / 3.6,
                                        dt=dt_)
                            if path_v is not None:
                                path_v, clip_x = fspace.clip_path(path_v, dt=dt_)
                        except Exception as e:
                            # Free space is a display refinement. It must never be
                            # the reason the scene stops drawing, so it fails to the
                            # unclipped path rather than out of the try block that
                            # wraps the whole render.
                            with LOCK:
                                STATE["freespace"] = "error: %s" % e

                    # --- voxel occupancy for vertical structure.
                    # Needs the SCALE-CORRECTED depth, so it is gated on the same
                    # fit the carve is: a voxel is a position, and an uncorrected
                    # 2x scale would stand every tree at twice its distance.
                    vox = None
                    if vfield is not None and dcorr is not None and seg_cls is not None \
                            and dscale is not None and dscale.resid is not None \
                            and dscale.resid <= VOXEL_MAX_RESID:
                        try:
                            vox = vfield.build(dcorr, seg_cls, rf, z_road,
                                               (src_w, src_h), mode=dscale.mode,
                                               hood=HOOD,
                                               v_ego=STATE.get("v_ego_kph", 0.0) / 3.6,
                                               dt=dt_ if fspace is not None else 0.125)
                        except Exception as e:
                            with LOCK:
                                STATE["voxels"] = "error: %s" % e
                    if vox is not None and ground is not None:
                        # the nominal-height wall and the measured volume are two
                        # answers to the same question; drawing both puts a 6 m
                        # slab through the middle of a 3 m hedge
                        ground.walls = None

                    img = view.render(
                        objects=objs,
                        path_xyz=path_v,
                        voxels=vox,
                        lane_lines=[_to_view(l, z_road) for l in draw_lanes] or None,
                        road_edges=[_to_view(e, z_road) for e in edges_xyz] or None,
                        lane_probs=draw_probs, ground=ground, edge_ok=edge_ok,
                        path_clip=clip_x)
                    if fspace is not None and use_display:
                        with LOCK:
                            OCC_FRAME["img"] = fspace.render(
                                w=260, h=380, path_view=path_v, clip=clip_x)
                            OCC_FRAME["seq"] += 1
                    with LOCK:
                        SCENE_FRAME["img"] = img
                        SCENE_FRAME["seq"] += 1
                        STATE["n_placed"] = len(objs)
                        STATE["n_lanes"] = n_lanes_drawn
                        STATE["seg_gate"] = seg_gate
                        STATE["n_unsupported"] = n_unsupported
                        STATE["n_hood_rejected"] = n_hood_rejected
                        STATE["n_dropped"] = ("%d no-mesh / %d truncated / %d unplaceable"
                                              % (n_nomesh, n_trunc, n_unplaced))
                        STATE["n_yaw_measured"] = n_yaw
                        if vfield is not None:
                            vs = vfield.stats
                            STATE["voxels"] = (
                                "%d pts -> %d cells -> %d quads%s"
                                % (vs.get("pts", 0), vs.get("cells", 0),
                                   vs.get("quads", 0),
                                   "  CAPPED" if vs.get("capped") else "")
                                + ("  [%s]" % vs["why"] if vs.get("why") else ""))
                        if fspace is not None:
                            s2 = fspace.stats
                            STATE["freespace"] = (
                                "%d free / %d blocked cells | carve %s (%d cells) | "
                                "%d obj cells" % (s2.get("free", 0), s2.get("blocked", 0),
                                                  s2.get("carve_state", "?"),
                                                  s2.get("carve", 0), s2.get("obj", 0)))
                            STATE["path_clip"] = ("%.1f m" % clip_x) if clip_x is not None \
                                else "none (path clear)"
                        if ground is not None:
                            s_ = ground.stats
                            # lane vs ipm is the number worth watching: a layer that
                            # is mostly ipm is not on the model's ruler any more.
                            # inferred = cells carried into the near field the camera
                            # cannot see (structure only, drawn at reduced alpha).
                            STATE["seg_cells"] = (
                                "%d ground + %d wall quads / %d lane / %d ipm / %d inferred"
                                % (s_["ground"], s_["wall"], s_["lane"], s_["ipm"],
                                   s_["inferred"]))
                            # where vertical structure is lost on its way to a quad
                            if "vert_cells" in s_:
                                STATE["wall_funnel"] = (
                                    "%d vert cells -> %d cols w/ vert -> %d cols kept "
                                    "-> %d adjacent pairs -> %d drawn"
                                    % (s_["vert_cells"], s_["cols_with_vert"],
                                       s_["cols_kept"], s_["pairs_adjacent"],
                                       s_["pairs_drawn"]))
                    if not use_display:          # only the web pane needs a JPEG
                        ok, jpg = cv2.imencode(".jpg", img,
                                               [int(cv2.IMWRITE_JPEG_QUALITY), 72])
                        if ok:
                            with LOCK:
                                SCENE_JPEG["buf"] = jpg.tobytes()
                                SCENE_JPEG["seq"] += 1
                except Exception:
                    pass
            time.sleep(max(0.0, 0.125 - (time.time() - t0)))     # ~8 fps

    def tx_thread():
        # Mazda LKAS (0x243) must be streamed continuously at 100 Hz. apply_torque
        # is 0 unless controlsd wants lateral control; even then the panda's Mazda
        # safety hooks gate every frame on controls_allowed (car reports cruise
        # engaged) and clamp torque/rate to the Mazda limits. This thread is the
        # ONLY place a CAN frame is transmitted, and it only runs when armed.
        # BIT_1/ERR_BIT_1/ERR_BIT_2 are COPIED live from the camera's 0x243 on bus 2
        # (cam_state, updated in can_thread) -- the real Mazda carcontroller does the
        # same. BIT_1 is the EPS's "LKAS active" gate: injecting 0 while the camera
        # asserts 1 makes the EPS ignore our torque. The checksum is recomputed over
        # whatever bits we send, so it stays consistent.
        ctr = 0
        # Steady 70 Hz (14.3 ms). On this GIL-loaded box a 100 Hz target (10 ms)
        # just produced jitter down to ~28 Hz. What the Mazda EPS cares about is
        # regularity -- it faults LKAS on irregular/dropped 0x243, not on the
        # nominal rate itself -- so the rate is set by what can be held STEADY,
        # and the thing that decides that is whether a can_recv still fits in the
        # gap between sends (see the link scheduler note below).
        #
        # This is still a deviation: stock openpilot's Mazda CarController sends
        # 0x243 EVERY frame at 100 Hz, and the 10-up/25-down rate limit is defined
        # per message, so halving the rate also halves the torque ramp to 500
        # units/s. Both sides (this thread and the panda's safety hook) apply the
        # same per-message limit, so nothing gets BLOCKED -- it just ramps slower.
        #
        # There is NO 52 kph floor on this platform, despite what LKAS_LIMITS
        # suggests: interface.py only sets minSteerSpeed for cars OTHER than
        # MAZDA_CX5_2022, so lkas_allowed_speed is True at all speeds here and the
        # EPS evaluates 0x243 from a standstill. Do not assume low-speed slack.
        #
        # MEASURED 2026-07-26, engine running, full-bus forwarding active: the
        # panda and the serial link sustain the stock 100 Hz cleanly -- 1000
        # frames handed over in 10.0 s, 1000 tx-complete echoes, 0 safety
        # rejections, 0 lost, err 0, bus_off 0, alongside 224/s of cam->car
        # forwarding on the same bus. So the transport is NOT what caps this.
        #
        # The cap is this process: that test was a standalone script with no
        # competing threads, whereas here the model, the camera capture and the
        # web server all contend for the GIL, which is what dragged a 100 Hz
        # target down to ~28 Hz before. Irregularity is the thing the EPS faults
        # on, so a steady rate beats a jittery faster one.
        #
        # DEFAULT 70 Hz (was 50). The binding constraint is one can_recv, which
        # costs ~15.5 ms and cannot be preempted once started: at 50 Hz that fits
        # inside the period and produced the 30-39 ms holes in 25% of one-second
        # windows. The link scheduler in can_thread now places the recv in the
        # gap before the next tx deadline rather than merely deferring it, so the
        # period no longer has to be longer than a recv -- it has to be long
        # enough that a recv still fits somewhere, which at 70 Hz (14.3 ms) it
        # only just does. Above ~70 the scheduler starts hitting its 250 ms
        # starvation escape instead, which puts the holes back.
        #
        # 70 Hz is inside the panda's rate budget with room to spare: the 250 ms
        # window holds 17.5 messages x max_rate_up 30 = 525 counts against
        # mazda.h .max_rt_delta = 1400 (37%). check_safety_limits() re-derives
        # this at startup from the mazda.h source, so it complains if either side
        # moves. Watch lkas_worst_ms and lkas_rx_forced before going higher.
        period = 1.0 / lkas_hz
        next_t = time.time()
        tx_state["hz_t0"] = time.time()
        while not stop_flag["v"]:
            new_torque = 0
            if tx_state["lat_active"]:
                new_torque = int(round(tx_state["torque_norm"] * CarControllerParams.STEER_MAX))
            apply_torque = apply_driver_steer_torque_limits(
                new_torque, tx_state["apply_last"], cs_can.steering_torque, CarControllerParams)
            tx_state["apply_last"] = apply_torque

            # Peaks, sampled before the rejection-resync below can zero them, so
            # a resync does not erase evidence of what the controller wanted.
            if abs(new_torque) > tx_state["want_peak"]:
                tx_state["want_peak"] = abs(new_torque)
            if abs(apply_torque) > tx_state["applied_peak"]:
                tx_state["applied_peak"] = abs(apply_torque)

            # --- resync after a panda-side rejection -------------------------
            # On ANY violation the firmware zeroes its own desired_torque_last
            # (opendbc/safety/lateral.h:144). This process keeps its independent
            # apply_last, so the next frame is checked against 0 and fails the
            # 10-unit rate limit too -- and so does every frame after it, for as
            # long as the model still wants torque. 0x243 then stops reaching the
            # bus ENTIRELY, and the EPS raises the front LKAS module fault on the
            # cluster. One rejected frame becomes a permanent dropout.
            #
            # Zeroing apply_last the moment safety_tx_blocked increments puts both
            # sides back at 0 (which always passes the rate check), so the frame
            # stream resumes on the very next message and then ramps up again at
            # 10/frame. Polled at 10 Hz: fast enough that a stall lasts ~100 ms
            # instead of indefinitely, cheap enough not to disturb the tx loop.
            if ctr % max(1, int(lkas_hz // 10)) == 0:
                try:
                    tx_due.set()
                    with panda_lock:
                        hh = panda.control_read(0xd2, 0, 0, 64)
                    if hh and len(hh) >= struct.calcsize(HEALTH_FMT):
                        _h = struct.unpack(HEALTH_FMT, hh[:struct.calcsize(HEALTH_FMT)])
                        # Publish the whole health tuple, not just the blocked
                        # counter. can_thread refreshes health_state at 1 Hz,
                        # which is far too coarse to catch a controls_allowed
                        # drop-and-relatch -- and that transient is exactly what
                        # leaves torque in flight and produces a resync. This
                        # read is already happening at 10 Hz for `blocked`, so
                        # reusing it costs nothing on the panda_lock, which the
                        # tx stream is sensitive to.
                        health_state["v"] = _h
                        health_state["t"] = time.time()

                        # --- panda safety-mode watchdog ---------------------
                        # SAFETY_MAZDA is written ONCE, at arm time, and was
                        # never re-checked. That is a single point of failure
                        # for the whole camera path, because SAFETY_SILENT is
                        # not just "we stop steering":
                        #
                        #   nooutput_init() returns disable_forwarding = true
                        #   (opendbc/safety/modes/defaults.h:16), and
                        #   safety_fwd_hook blocks EVERYTHING when that is set
                        #   (safety.h:261).
                        #
                        # So in SILENT the panda stops relaying the forward
                        # camera to the car entirely -- not only 0x243, which
                        # we transmit ourselves, but every other FSC frame the
                        # firmware normally passes bus 2 -> bus 0. From the
                        # car's point of view the camera is UNPLUGGED, and a
                        # Mazda raises the front-camera fault on the cluster
                        # and latches it for the ignition cycle.
                        #
                        # Nothing on our side looks wrong when this happens:
                        # can_send still succeeds over the serial link (the
                        # firmware drops the frame afterwards), so tx_frames
                        # keeps climbing at exactly the nominal rate and the dashboard
                        # stays green. safety_tx_blocked does climb, but the
                        # resync above reads that as a rate-limit violation and
                        # zeroes apply_last forever instead of re-arming.
                        #
                        # Anything that resets the MCU puts us here: the
                        # cranking brownout, a USB re-enumeration, the
                        # firmware watchdog. Re-assert instead of trusting the
                        # one write at startup, and count it -- a non-zero
                        # panda_remodes in the log is the proof that this was
                        # the failure, and uptime going backwards says the
                        # board rebooted rather than merely timing out.
                        if _h[0] < tx_state["uptime_last"]:
                            tx_state["panda_resets"] += 1
                        tx_state["uptime_last"] = _h[0]
                        # BOTH halves matter. _h[12] is health_t.safety_mode and
                        # _h[13] is health_t.safety_param -- the watchdog only ever
                        # checked the mode, and re-asserted it with a hardcoded 0,
                        # which CLEARS MAZDA_PARAM_LONGITUDINAL. mazda_init() reads
                        # bit 0 of that param to pick between MAZDA_TX_MSGS (3
                        # entries) and MAZDA_LONG_TX_MSGS (22). Drop it and the tx
                        # hook rejects every 0x21b, 0x21c and 0x764 we send, for the
                        # rest of the drive, with no error anywhere on our side:
                        # can_send still returns success, long_tx_frames still
                        # climbs at the nominal rate, long_tx_err stays 0. The only
                        # visible symptom is that the car never responds -- which is
                        # indistinguishable from "the PCM rejects our frames" and is
                        # precisely the wrong conclusion to draw.
                        #
                        # A mode that is right with a param that is wrong was
                        # entirely invisible before, so check and repair both.
                        _want_param = int(tx_state["safety_param"])
                        if _h[12] != SAFETY_MAZDA or _h[13] != _want_param:
                            _param_only = (_h[12] == SAFETY_MAZDA)
                            panda.control_write(0xdc, SAFETY_MAZDA, _want_param)
                            if _param_only:
                                tx_state["param_fixups"] += 1
                                print("PANDA   safety param was %d, want %d "
                                      "(alpha long was silently OFF) -- re-asserted"
                                      % (_h[13], _want_param), flush=True)
                            else:
                                tx_state["remodes"] += 1
                                print("PANDA   safety mode was %d, want %d -- "
                                      "re-asserted with param %d"
                                      % (_h[12], SAFETY_MAZDA, _want_param), flush=True)
                            # The firmware's desired_torque_last is 0 after a
                            # mode change, so ours must be too or every frame
                            # fails the rate check (same lockout as a resync).
                            tx_state["apply_last"] = 0
                            apply_torque = 0

                        blocked = _h[3]
                        if blocked > tx_state["blocked_last"]:
                            tx_state["blocked_seen"] += blocked - tx_state["blocked_last"]
                            tx_state["resyncs"] += 1
                            tx_state["apply_last"] = 0
                            apply_torque = 0
                        tx_state["blocked_last"] = blocked
                except Exception:
                    pass
                finally:
                    tx_due.clear()

            # --- never ramp down into a closed gate ---------------------------
            # steer_torque_cmd_checks ends with:
            #     if (!controls_allowed && (desired_torque != 0)) violation = true;
            # so while controls_allowed is false the ONLY torque that reaches the
            # bus is exactly 0. apply_driver_steer_torque_limits ramps down at
            # max_rate_down (25/frame), so a drop from -1126 spends 45 frames --
            # 0.64 s at 70 Hz -- emitting non-zero values that are every one of them
            # rejected. 0x243 vanishes from the bus for that whole window, and the
            # cluster raises the front camera fault. Measured on 2026-07-30:
            # 13 such windows, longest 0.94 s, in a single 370 s run.
            #
            # Forcing 0 the moment the panda says the gate is shut keeps the frame
            # STREAM alive (zero-torque frames pass the check) while still
            # commanding no torque. health is polled at 10 Hz just above, so the
            # worst-case blackout becomes one poll interval instead of the full ramp.
            # Defaults to True before the first poll, which is the pre-existing
            # behaviour and safe -- new_torque is 0 until lat_active anyway.
            if health_state["v"] is not None and not health_state["v"][10]:
                apply_torque = 0
                tx_state["apply_last"] = 0

            # LINE_NOT_VISIBLE/LDW BELONG HERE TOO, and their absence was a live
            # trap. create_steering_control reads them only when _COPY_CAM_LINES is
            # set (mazdacan.py:46), so with the flag off the lookup short-circuits
            # and nothing notices they were never in this dict. Turn the flag on and
            # the very first frame raises KeyError inside tx_thread.
            #
            # MEASURED 2026-08-11: that is exactly what happened. tx_thread died on
            # frame one, tx_frames stayed at 0 for the whole run, and because the
            # panda holds SAFETY_MAZDA -- which blocks the camera's own 0x243 from
            # reaching the car -- the cluster received NO LKAS frame from any
            # source and raised the front camera sensor fault. Armed, silent, and
            # faulting the car is the worst of the three states, and the only
            # symptom in the log was txgap=0/0ms.
            #
            # Bit positions match the ck validator above: lnv = byte2 bit3,
            # ldw = byte2 bit7 (DBC CAM_LKAS LINE_NOT_VISIBLE / LDW).
            cam_bits = {"BIT_1": cam_state["BIT_1"], "ERR_BIT_1": cam_state["ERR_BIT_1"],
                        "ERR_BIT_2": cam_state["ERR_BIT_2"],
                        "LINE_NOT_VISIBLE": cam_state.get("LINE_NOT_VISIBLE", 0),
                        "LDW": cam_state.get("LDW", 0)}

            # Steering-wheel angle the MODEL wants, from controlsd's clipped desired
            # curvature through the VehicleModel. Always computed (it is the useful
            # telemetry: compare it against cs_can.steering_angle to see whether the
            # wheel is following), but only PUT ON THE WIRE with --steer-angle.
            # Reason it is opt-in: this field feeds mazdacan's reverse-engineered
            # checksum, whose angle terms are only fitted for the value the stock
            # code emits (0). Getting it wrong corrupts the checksum of 0x243 --
            # the one frame the EPS actually acts on -- so it must be proven with
            # the ck_* counters on the dashboard first. Zero when lateral control
            # is not active, so we never claim an angle we are not steering to.
            angle_deg = (model_steer_angle_deg(tx_state["curvature"], cs_can.v_ego)
                         if tx_state["lat_active"] else 0.0)
            angle_raw = 0
            if steer_angle_inject:
                angle_raw = int(max(-LKAS_ANGLE_RAW_LIMIT,
                                    min(LKAS_ANGLE_RAW_LIMIT,
                                        round(angle_deg / LKAS_ANGLE_DEG_PER_BIT))))
            tx_state["angle_deg"] = angle_deg
            tx_state["angle_raw"] = angle_raw

            msg = mazdacan.create_steering_control(packer, CP, ctr, apply_torque, cam_bits, angle_raw)
            tx_due.set()
            try:
                with panda_lock:
                    panda.can_send(msg[0], bytes(msg[1]), 0)
                    # Heartbeat (0xf3) at 10 Hz. WITHOUT it the panda reverts to
                    # SAFETY_SILENT after 2-5 s. CRITICAL: engaged must be 1 the whole
                    # time controls_allowed is set, or the firmware revokes it after 3 s
                    # of (controls_allowed && !heartbeat_engaged). controls_allowed is
                    # set on the cruise-engage edge, BEFORE lat_active -- so tie engaged
                    # to op_enabled (true the instant openpilot engages), not lat_active,
                    # or we lose the window and controls never come back.
                    if ctr % 10 == 0:
                        panda.control_write(0xf3, 1 if tx_state["op_enabled"] else 0, 0)
                tx_state["tx_frames"] += 1
                tx_state["applied"] = apply_torque
            except Exception:
                time.sleep(0.005)
            finally:
                tx_due.clear()
            ctr += 1

            # Achieved tx rate over a rolling ~1 s window. Frames counted here are
            # frames the panda ACCEPTED over the link, so this is the rate the EPS
            # actually sees -- which is also the rate the 10-counts-per-message
            # ramp limit is denominated in (hz * 10 = counts/s of ramp).
            now_hz = time.time()
            # JITTER, not just mean rate. The EPS faults on IRREGULAR 0x243, and
            # a mean of 100 Hz is perfectly consistent with half the intervals at
            # 5 ms and half at 15 ms -- which is what GIL contention actually
            # produces here. Without this, raising --lkas-hz looks like a clean
            # win in the mean right up until the EPS drops LKAS. worst_ms is the
            # number that decides whether a higher rate is safe to keep.
            if tx_state["last_tx_t"]:
                gap_ms = (now_hz - tx_state["last_tx_t"]) * 1000.0
                if gap_ms > tx_state["worst_ms_run"]:
                    tx_state["worst_ms_run"] = gap_ms
                # intervals more than 1.5x the target are late enough to matter
                if gap_ms > 1.5 * period * 1000.0:
                    tx_state["late_frames"] += 1
            tx_state["last_tx_t"] = now_hz
            if now_hz - tx_state["hz_t0"] >= 1.0:
                tx_state["hz"] = ((tx_state["tx_frames"] - tx_state["hz_n0"])
                                  / (now_hz - tx_state["hz_t0"]))
                tx_state["hz_t0"] = now_hz
                tx_state["hz_n0"] = tx_state["tx_frames"]
                # Report the worst gap in the LAST SECOND, and keep the run-long
                # worst separately: a single 40 ms stall an hour ago should not
                # keep the live readout red, but it must not vanish either.
                tx_state["worst_ms"] = tx_state["worst_ms_run"]
                tx_state["worst_ms_all"] = max(tx_state["worst_ms_all"],
                                               tx_state["worst_ms_run"])
                tx_state["worst_ms_run"] = 0.0

            next_t += period
            # Publish the deadline BEFORE sleeping on it, so can_thread can see
            # the gap it has to fit a recv into. Written after next_t has been
            # advanced (and after the catch-up reset below), so it is always the
            # deadline this loop is actually about to sleep until.
            sleep = next_t - time.time()
            if sleep > 0:
                tx_sched["next_t"] = next_t
                time.sleep(sleep)
            else:
                # Overran the period; the next send is due immediately, so give
                # can_thread a deadline it will correctly refuse to fit into.
                next_t = time.time()
                tx_sched["next_t"] = next_t

    # --- inference in its own thread ---------------------------------------
    # Previously inference ran inline in the publish loop, which had no pacing:
    # the loop spun at ~1000 Hz and re-published EVERY topic each spin, so every
    # service ran ~50x its nominal rate. selfdrived's freq check ([0.8x,1.2x] of
    # nominal) then raised commIssueAvgFreq (NO_ENTRY) and blocked engagement, and
    # controlsd -- flooded on its inputs -- free-ran at ~200 Hz. Decoupling
    # inference here lets the publish loop run at a fixed rate with per-service
    # throttling, so every topic goes out at its real nominal frequency.
    # action/action_raw stay None until the first step with a real v_ego: the
    # command scales with the speed it is divided by, so op_stream returns no
    # action at all rather than one computed against a guess.
    # govern_accel state: last commanded accel (for the jerk limit), the raw
    # action-head value and which clamp bound it, both surfaced in status.json.
    _gov = {"t": 0.0, "a": 0.0, "raw": 0.0, "why": ""}
    mstate = {"out": {"path_xyz": None}, "curv": 0.0, "frames": 0,
              "lead_d": None, "lead_p": None,
              "action": None, "action_raw": None}

    # ---- desire: turn signal -> what the model is asked to DO ---------------
    # supercombo takes a `desire_pulse` (1,25,8) input that this stack has always
    # fed as zeros, so the model has never been asked for anything but "keep
    # lane". These three pieces wire it up.
    from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
    from openpilot.selfdrive.modeld.constants import ModelConstants as _MCd
    DH = DesireHelper()

    # log.Desire ordinals. Hardcoded rather than imported from the capnp enum
    # because this indexes a MODEL TENSOR -- the two only agree because the model
    # was trained against this ordering, and a schema change upstream must not
    # silently re-map the tensor.
    DESIRE_NONE, DESIRE_TURN_L, DESIRE_TURN_R = 0, 1, 2
    DESIRE_LC_L, DESIRE_LC_R = 3, 4

    # DesireHelper ignores the blinker below 20 mph (LANE_CHANGE_SPEED_MIN) --
    # correctly, since a blinker at 10 mph is a turn, not a lane change. That
    # leaves turnLeft/turnRight unused by upstream entirely (they fed Navigate on
    # openpilot, removed in 0.9.7). This fills that gap: under the lane-change
    # speed, a blinker means a TURN.
    #
    # No nudge required here, unlike lane change. The nudge exists so the car
    # cannot wander into an adjacent lane on a stray indicator at speed; at
    # turning speed you are already steering, and requiring a nudge would mean
    # the desire only fires after you have started the turn yourself.
    TURN_SPEED_MAX = 8.94          # m/s, == LANE_CHANGE_SPEED_MIN

    class DesirePulse:
        """Builds the (1,25,8) input from a Desire ordinal.

        The model wants a PULSE on the rising edge, not a level: modeld.py:114
        notes "Model decides when action is completed, so desire input is just a
        pulse triggered on rising edge". Holding the one-hot high instead would
        re-trigger the manoeuvre every frame for as long as the stalk is on.
        """
        def __init__(self):
            self.hist = np.zeros((25, _MCd.DESIRE_LEN), np.float16)
            self.prev = np.zeros(_MCd.DESIRE_LEN, np.float32)

        def step(self, desire):
            vec = np.zeros(_MCd.DESIRE_LEN, np.float32)
            if 0 < desire < _MCd.DESIRE_LEN:      # index 0 is "none": never pulsed
                vec[desire] = 1.0
            pulse = np.where(vec - self.prev > 0.99, vec, 0.0)
            self.prev = vec
            self.hist = np.roll(self.hist, -1, axis=0)
            self.hist[-1] = pulse
            return self.hist[None]                # (1,25,8)

    pulser = DesirePulse()

    class _CSForDesire:
        """The six carState fields DesireHelper.update() touches.

        Built from cs_can rather than round-tripping through the published
        carState message: this runs in the model thread and the cereal copy is
        produced by a different thread at a different rate, so reading it here
        would introduce a lag between the blinker the driver flicked and the
        blinker the state machine sees.
        """
        __slots__ = ('vEgo', 'leftBlinker', 'rightBlinker', 'steeringPressed',
                     'steeringTorque', 'leftBlindspot', 'rightBlindspot')

        def __init__(self, c):
            self.vEgo = c.v_ego
            self.leftBlinker = c.left_blinker
            self.rightBlinker = c.right_blinker
            self.steeringPressed = c.steering_pressed
            self.steeringTorque = c.steering_torque
            self.leftBlindspot = c.left_blindspot
            self.rightBlindspot = c.right_blindspot

    def pick_desire(cs_for_dh, lat_active, lane_change_prob):
        """DesireHelper above the lane-change speed, turn desires below it."""
        DH.update(cs_for_dh, lat_active, lane_change_prob)
        if DH.desire != DESIRE_NONE:
            return DH.desire
        if not lat_active:
            return DESIRE_NONE
        one_blinker = cs_for_dh.leftBlinker != cs_for_dh.rightBlinker
        if one_blinker and cs_for_dh.vEgo < TURN_SPEED_MAX:
            return DESIRE_TURN_L if cs_for_dh.leftBlinker else DESIRE_TURN_R
        return DESIRE_NONE

    def model_thread():
        # Pace inference at supercombo's NATIVE 20 Hz, not at camera rate.
        #
        # The model takes two consecutive frames, and was trained with them 50 ms
        # apart. Feeding it whatever the camera produces (~30 fps here, 33 ms) makes
        # every motion estimate come out scaled by dt/0.05 -- and pose is exactly
        # that estimate. At 30 fps a true 4.17 m/s (15 km/h) reads as ~2.8 m/s, which
        # fails op_calibrate's `trans[0] > MIN_SPEED_FILTER` (4.167) gate, so the
        # calibrator silently collects ZERO blocks no matter how far you drive.
        # The dashboard meanwhile says "needs >15 km/h", so it looks like the
        # calibrator is broken rather than never being handed a valid sample.
        #
        # This is the same failure the 4.3fps note above describes, in the other
        # direction: that fix decoupled the threads and let the rate run past 20 Hz.
        # Pace it instead of chasing the camera. Also cheaper, and keeps the model's
        # temporal input on the distribution it was trained on.
        MODEL_DT = 0.05
        last_mn = -1
        next_t = time.time()
        while not stop_flag["v"]:
            now = time.time()
            if now < next_t:
                time.sleep(min(0.005, next_t - now))
                continue
            with cap_lock:
                f = cap["frame"]; n = cap["n"]
            if f is not None and n != last_mn:
                last_mn = n
                next_t = max(next_t + MODEL_DT, now)   # no burst-catchup after a stall
                # Desire is decided BEFORE the step from the PREVIOUS frame's
                # lane_change_prob, the same ordering modeld uses: the model
                # tells us the manoeuvre is finished, we act on that, and the
                # next inference carries the updated pulse. Deciding after the
                # step would feed a desire derived from the very output it is
                # meant to influence.
                desire = pick_desire(_CSForDesire(cs_can),
                                     bool(tx_state["lat_active"]),
                                     mstate.get("lane_change_prob", 0.0))
                # lat_action_t passed explicitly: its default is bound at import
                # time from OP_STEER_DELAY, so leaving it out would freeze the
                # model's planning horizon and only the controller side would
                # follow /tune. Both must move together or they describe
                # different lags -- the exact split this variable exists to fix.
                o = runner.step(f, v_ego=cs_can.v_ego, desire=pulser.step(desire),
                                lat_action_t=STEER_DELAY + LAT_ACTION_T_OFFSET)
                # STOCK action (see dashcam.py) -- already computed inside step()
                # because v_ego was passed. On "Rebellious Hope" this is the model's
                # own action head; on the older weights it is reconstructed from the
                # plan. Either way all THREE fields are carried forward now, not just
                # the curvature: modeld publishes desiredAcceleration and shouldStop
                # on the same message, and dropping them here left the longitudinal
                # fields of modelV2 at their capnp defaults.
                act = o.get("action")
                if act is not None:
                    mstate["curv"] = act["desiredCurvature"]
                    mstate["action"] = act
                # LANE-CHANGE DIAGNOSTIC: what the model PLANS laterally.
                #
                # We steer from the action head's desiredCurvature alone, so if the
                # model plans a lane change but the head does not express it, the
                # command stays flat and the car never leaves its lane -- which is
                # exactly the measured symptom. Over 14 sustained desire episodes on
                # 2026-08-09 the curvature moved toward the requested lane in only 4,
                # mean -0.000305 against the ~+0.0008 a 3.5 m change needs, while the
                # model acknowledged every one of them with lane_change_prob 1.0.
                #
                # path_xyz is in the calibrated frame, y positive to the RIGHT, so a
                # committed change shows as y walking out to roughly a lane width at
                # the far end of the horizon. If plan_y swings and curv does not, the
                # fault is on our extraction side; if neither moves, the model is
                # declining the manoeuvre and the desire input is the thing to chase.
                _pxyz = o.get("path_xyz")
                if _pxyz is not None:
                    try:
                        mstate["plan_y"] = (float(_pxyz[16][1]), float(_pxyz[24][1]),
                                            float(_pxyz[32][1]))
                    except Exception:
                        mstate["plan_y"] = None
                # The head's raw output, before v_ego is divided out of it:
                # [lateral accel, longitudinal accel] in m/s^2. Recorded because it
                # is the one number that is directly comparable to the 3.0 m/s^2
                # lateral-accel limit clip_curvature enforces downstream.
                mstate["action_raw"] = o.get("action_raw")
                # Nearest lead, for govern_accel's time-gap check. lead[0][0] is
                # [x, y, v, a] for the closest of the three hypotheses; slot 0 is
                # what publishes as leadOne. Only x and the probability are used.
                try:
                    _lead, _lprob = o.get("lead"), o.get("lead_prob")
                    if _lead is not None and _lprob is not None and len(_lead):
                        mstate["lead_d"] = float(_lead[0][0][0])
                        mstate["lead_p"] = float(_lprob[0])
                        # v and a as well: govern_accel only needed distance, but
                        # the MPC models the lead's own motion to decide how early
                        # to lift. LEAD_WIDTH is [x, y, v, a].
                        mstate["lead_v"] = float(_lead[0][0][2])
                        mstate["lead_a"] = float(_lead[0][0][3])
                    else:
                        mstate["lead_d"] = mstate["lead_p"] = None
                        mstate["lead_v"] = mstate["lead_a"] = None
                except Exception:
                    mstate["lead_d"] = mstate["lead_p"] = None
                ds = o.get("desire_state")
                if ds is not None:
                    # DesireHelper leaves laneChangeStarting when this drops
                    # below 0.02 -- the model's own "I am done" signal.
                    mstate["lane_change_prob"] = float(ds[DESIRE_LC_L] + ds[DESIRE_LC_R])
                    mstate["desire_state"] = ds
                mstate["desire"] = int(desire)
                mstate["lc_state"] = int(DH.lane_change_state)
                mstate["lc_dir"] = int(DH.lane_change_direction)
                mstate["out"] = o
                mstate["frames"] += 1
            else:
                time.sleep(0.002)

    def long_tx_thread(tx_enabled: bool = True):
        """ALPHA LONG: transmit the radar's frames in its place.

        WHY THIS IS A SEPARATE THREAD FROM tx_thread. The 0x243 stream is the one
        thing on this board with a hard regularity requirement -- the EPS faults
        LKAS on irregular frames, and tx_thread's rate, its link scheduler and its
        rejection-resync are all tuned around that. Longitudinal runs at 50 Hz with
        no such constraint, so it gets its own loop and tx_thread is left exactly as
        it was rather than being taught a second cadence.

        WHY IT DRIVES THE REAL CarController. The stop-and-go sequence in
        carcontroller.py -- five CRZ_CTRL profiles, the hold latch, the passive-hold
        substate, the resume unlatch phases -- is what took the community from
        "longitudinal works" (2026-03) to "stop and go works" (2026-04). It is
        stateful and the states are not guessable. Reimplementing it here to fit
        this process's data shapes would be rewriting the only part that is known
        good, so instead the real controller runs and only its longitudinal frames
        are taken; the LKAS/HUD frames it also builds are DISCARDED, because
        tx_thread already owns those.
        """
        from opendbc.car.mazda.carcontroller import CarController as MazdaCarController
        from opendbc.car.mazda.longitudinal import CRZ_CTRL_ADDR, CRZ_INFO_ADDR, RADAR_ADDR
        # Heartbeat ids included: create_radar_heartbeat_messages emits
        # 0x361-0x366 and 0x499 on BOTH buses, and without them here the
        # filter below drops the very frames that reach the camera.
        _txlog = {"t": 0.0}     # rate limiter for the LONGTX transmit log
        # SET-BUTTON HANDSHAKE.
        #
        # VERIFIED 2026-08-11 at the transmit point: when openpilot engages we send
        #     0x21b 01 ff e1 80 06 80 0d 0b     0x21c 0a 01 8b 40 00 00 10 00
        # on BOTH buses, counter and checksum tracking, longActive=1, accel=+2.00 --
        # and acc_active (the PCM's own report on PEDALS 0x165) stays 0. The frames
        # are right; the PCM simply does not enter ACC on CRZ_INFO.ACC_ACTIVE alone.
        #
        # In the stock system the driver presses SET, the RADAR observes it on
        # CRZ_BTNS and activates. With the radar suppressed nobody plays that part:
        # alpha long engages on MRCC MAIN (acc_armed), so the car never sees a set
        # event, and we announce "ACC active" for a cruise that was never set.
        #
        # So play it. One short SET press when longActive is true and the PCM still
        # has not activated, retried at a slow cadence and capped -- if this is not
        # the mechanism, it must not turn into a button spammer.
        from opendbc.car.mazda.values import Buttons as _Btns
        _setbtn = {"t": 0.0, "n": 0, "max": int(os.environ.get("OP_SET_HANDSHAKE_MAX", "6")),
                   # OFF by default now. It fired on longActive, which under the
                   # PCM engagement order already implies acc_active=1 -- so it can
                   # never trigger, and a synthetic SET press is the driver's call
                   # to make, not ours. OP_SET_HANDSHAKE=1 to re-enable.
                   "on": os.environ.get("OP_SET_HANDSHAKE", "0") == "1"}
        LONG_ADDRS = {CRZ_INFO_ADDR, CRZ_CTRL_ADDR, RADAR_ADDR,
                      0x361, 0x362, 0x363, 0x364, 0x365, 0x366, 0x499}

        # CarController.__init__ does CANPacker(dbc_names[Bus.pt]); this process
        # only ever built a bare CANPacker('mazda_2017'), so hand it the dict form.
        from opendbc.car import Bus as _Bus
        cc_obj = MazdaCarController({_Bus.pt: 'mazda_2017'}, CP)

        # Minimal stand-ins for the two objects CarController.update() reads. This
        # process never builds opendbc's CarState (it has its own CarStateFromCAN
        # off raw CAN), so the fields carcontroller actually touches are mirrored
        # onto a shim. Anything it reads that is NOT set here would raise on first
        # use rather than silently read a default -- which is the intent.
        class _CruiseState:
            # go-no-malfunction's carcontroller reads CS.out.cruiseState.available to
            # drive crz_available, which pairs CRZ_AVAILABLE in CRZ_CTRL with
            # ACC_SET_ALLOWED in CRZ_INFO -- the synthetic pre-engage state our old
            # static templates could not express.
            #
            # Without it update() raises '_Out' object has no attribute
            # 'cruiseState' on EVERY cycle, so no longitudinal frame is ever built:
            # MEASURED 2026-08-11, tester-present at 129 and long_tx/crz_info at 0.
            # The thread's own error print is what surfaced it -- worth keeping in
            # mind that a shim missing one attribute silences the whole path.
            available = False

        class _Out:
            def __init__(self):
                self.cruiseState = _CruiseState()

        # CarController.update() builds the LKAS and HUD frames unconditionally,
        # outside every openpilotLongitudinalControl gate -- create_steering_control
        # runs EVERY frame and create_alert_command every 50th. Both index camera
        # dicts, so without these three fields update() raises AttributeError on
        # the first call and this thread transmits nothing at all.
        #
        # The frames they produce are dropped by the LONG_ADDRS filter below
        # (tx_thread owns 0x243 and CAM_LANEINFO), so these only have to make
        # construction succeed. cam_lkas points at the live camera state because
        # it already carries the three keys create_steering_control reads; the
        # laneinfo fields are not decoded anywhere in this process, so they are
        # zeros. If CAM_LKAS or CAM_LANEINFO is ever added to LONG_ADDRS, these
        # placeholders become wire-visible and must be sourced for real first.
        CAM_LANEINFO_NEUTRAL = dict.fromkeys(
            ("LINE_VISIBLE", "LINE_NOT_VISIBLE", "LANE_LINES", "BIT1", "BIT2",
             "BIT3", "NO_ERR_BIT", "S1", "S1_HBEAM"), 0)

        class _CS:
            def __init__(self):
                self.out = _Out()
                self.accel_button = 0
                self.crz_btns_counter = 0
                self.cam_lkas = cam_state
                self.cam_laneinfo = CAM_LANEINFO_NEUTRAL
                self.lkas_allowed_speed = False

        shim = _CS()
        period = 1.0 / 100.0          # CarController is written against DT_CTRL
        nxt = time.time()
        # Radar-return watchdog. Suppression is granted once at arming and is NOT
        # guaranteed to hold -- the radar can time out of the programming session
        # or reset, and then it and this thread are both writing 0x21b. Our own
        # frames echo back on bus 0 at that same address, so the radar's share is
        # the census delta MINUS what we sent.
        watch = {"t": time.time(), "census": can_census[0].get(CRZ_INFO_ADDR, 0), "ours": 0}
        tp_last = 0.0
        while not stop_flag["v"]:
            now = time.time()
            # Keep-alive at 10 Hz, independent of the CarController's 2 Hz.
            # can_tx_lost0 climbs ~33 frames/s: the panda's bus-0 tx queue is
            # over capacity from 0x243 plus cam->car forwarding, and it discards
            # silently -- can_send returns fine, the frame never reaches the wire.
            # At 2 Hz a ~10% drop rate can take out enough consecutive frames to
            # expire the radar's ~5 s S3 timeout, which is the lapse we measured
            # at ~14 s. 10 Hz means ~50 chances per timeout window instead of 10.
            # Cheap insurance: one 8-byte frame, and the radar ignores extras.
            # radar_hold gates the keep-alive. Dropping it is how the RADAR IS
            # RELEASED: the session lapses on its own within ~5 s and the radar
            # comes back with FCW/AEB/SBS. FINDINGS establishes that this is the
            # clean exit -- explicitly requesting the default session makes this
            # radar fault. Always true outside engage-first's re-arm loop.
            # DEAD-MAN'S SWITCH. If the bus has gone quiet we have no way to know
            # whether ACC is still on, so we have no business holding the radar
            # muted -- and no way to ever release it, since the release watches
            # RX. Drop the hold; the session lapses on its own in ~5 s and the
            # car returns to stock with FCW/AEB/SBS back. Latched, not momentary:
            # re-suppressing the instant a frame trickles through would flap the
            # radar in and out of a programming session, which is worse than
            # either state. The re-arm loop restores it deliberately, from a
            # fresh handover, once the link is genuinely healthy again.
            if radar_hold["on"] and not rx_alive(now):
                radar_hold["on"] = False
                tx_state["radar_released_rx_dead"] = \
                    tx_state.get("radar_released_rx_dead", 0) + 1
                print("\n>>> RX DEAD (%.1f s with no bus-0 frame): RELEASING THE "
                      "RADAR.\n    The mute is not safe to hold when we cannot see "
                      "the car -- the\n    release is RX-driven, so holding it here "
                      "would be unclearable.\n    Session lapses in ~5 s; "
                      "FCW/AEB/SBS come back."
                      % rx_live.age(now), flush=True)

            if now - tp_last >= 0.1 and radar_hold["on"]:
                tp_last = now
                tx_due.set()
                try:
                    with panda_lock:
                        panda.can_send(RADAR_ADDR, bytes([0x02, 0x3E, 0x80, 0, 0, 0, 0, 0]), 0)
                    tx_state["radar_tp_tx"] = tx_state.get("radar_tp_tx", 0) + 1
                except Exception:
                    tx_state["long_tx_err"] = tx_state.get("long_tx_err", 0) + 1
                finally:
                    tx_due.clear()

            if coexist or (engage_first and not tx_state.get("handover_done")):
                # The radar is deliberately alive in this mode, so its 0x21b is not
                # evidence of a lapsed session -- it is the design. Firing the
                # re-suppression watchdog here would try to silence a radar we chose
                # to keep, and then halt transmit when it failed.
                #
                # engage_first is the same situation until the handover completes:
                # the radar owns 0x21b by design until the PCM has entered ACC. The
                # watchdog becomes correct again the moment handover_done is set.
                #
                # Rebaseline while skipping, or the first window AFTER the handover
                # counts every frame the radar sent during the whole engage-first
                # phase as "foreign" and halts transmit on the spot.
                watch.update(t=now, census=can_census[0].get(CRZ_INFO_ADDR, 0),
                             ours=tx_state.get("crz_info_tx", 0))
            elif now - watch["t"] >= 2.0:
                _c = can_census[0].get(CRZ_INFO_ADDR, 0)
                _o = tx_state.get("crz_info_tx", 0)
                _foreign = (_c - watch["census"]) - (_o - watch["ours"])
                watch.update(t=now, census=_c, ours=_o)
                # The radar runs ~43 Hz, so ~86 frames per window if it is back.
                # 20 (10 Hz) is well clear of rx-loss noise in either direction.
                if _foreign > 20:
                    # The radar leaves the session within ~1 s of controls_allowed
                    # going true, reproducibly, and nothing we send explains it --
                    # tester-present at 10 Hz, no tx loss, CRZ_ACTIVE and the 50 Hz
                    # stream both cleared by probe. The trigger is an ACC state
                    # change inside the car. Re-entry takes ~30 ms and is proven,
                    # so recover instead of guessing further.
                    tries = tx_state.get("radar_resuppress", 0) + 1
                    tx_state["radar_resuppress"] = tries
                    if tries > 3:
                        tx_state["long_halted"] = True
                        print("\n!!! RADAR RETURNED %d times -- giving up.\n"
                              "    Longitudinal transmit STOPPED. Stock radar has "
                              "longitudinal again.\n    Steering unaffected.\n" % tries)
                        return
                    print("\n!!! RADAR RETURNED (%d frames in 2 s). Re-entering the "
                          "programming session [%d/3]..." % (_foreign, tries))
                    # Nothing of ours goes out during the check below, so any 0x21b
                    # counted is the radar's -- the same authoritative silence test
                    # used at arming, not the handshake's own return value.
                    try:
                        with panda_lock:
                            panda.can_send(RADAR_ADDR,
                                           bytes([0x02, 0x10, 0x02, 0, 0, 0, 0, 0]), 0)
                    except Exception:
                        pass
                    _c0 = can_census[0].get(CRZ_INFO_ADDR, 0)
                    time.sleep(1.0)
                    _seen = can_census[0].get(CRZ_INFO_ADDR, 0) - _c0
                    if _seen > 5:
                        tx_state["long_halted"] = True
                        print("    re-entry FAILED (%d frames in 1 s). Transmit "
                              "STOPPED.\n" % _seen)
                        return
                    print("    re-entry OK, radar silent again. Resuming.\n")
                    # Rebaseline, or the frames counted during recovery fire it again.
                    watch.update(t=time.time(),
                                 census=can_census[0].get(CRZ_INFO_ADDR, 0),
                                 ours=tx_state.get("crz_info_tx", 0))
                    continue
            if now < nxt:
                time.sleep(min(0.002, nxt - now))
                continue
            nxt = max(nxt + period, now)

            cc = sm['carControl']
            # CRZ_AVAILABLE must track the car, not our own frame. Under alpha long
            # we generate 0x21c, so reading availability back off it is circular --
            # acc_armed comes from PEDALS, which is the PCM's own report and what
            # the panda gates on.
            # Guarded: the pristine vendored longitudinal.py has no such hook. See
            # the baseline-test note in interface.py.
            if hasattr(_mazda_long, "set_cruise_available"):
                _mazda_long.set_cruise_available(bool(getattr(cs_can, "acc_armed", False)))
            shim.out.cruiseState.available = bool(getattr(cs_can, "acc_armed", False))
            shim.out.vEgo = float(cs_can.v_ego)
            shim.out.standstill = bool(cs_can.v_ego < 0.3)
            shim.out.gasPressed = bool(cs_can.gas_pressed)
            shim.out.brakePressed = bool(cs_can.brake_pressed)
            shim.out.steeringTorque = float(cs_can.steering_torque)
            # Read these straight off cs_can, no hasattr/getattr fallback: a
            # missing field must raise here like every other shim field, not
            # decay to 0. Both defaults were silently wrong -- res_button never
            # existed (RES lives in cs_can.buttons) so physical RES was dead,
            # and a frozen counter made every virtual RES frame a duplicate.
            shim.accel_button = int(cs_can.buttons["res"])
            shim.crz_btns_counter = int(cs_can.crz_btns_counter)

            try:
                sends = cc_obj.update(cc, shim, int(now * 1e9))[1]
            except Exception as e:
                # Loud and rate-limited: a shim field the controller wants but this
                # process does not provide shows up here, not as silent no-accel.
                if int(now) % 5 == 0:
                    print("long_tx: CarController.update failed:", e)
                continue

            # WHAT WE ACTUALLY PUT ON THE WIRE, logged at the transmit point.
            #
            # Every "are we sending the right thing?" question today was answered by
            # reading crz_en back off the bus -- and that field CANNOT move in this
            # configuration: our own echo returns tagged (bus + 128 for returned)
            # and cs_can.update only decodes bus == 0, so with the radar silenced
            # nothing ever writes it. It sat at 0 and I read that as "we are not
            # setting CRZ_ACTIVE", which the data could not support.
            #
            # So log the bytes here, where there is no ambiguity. Rate-limited to
            # one line a second per address, and only while alpha long is up.
            # Fire the SET handshake before the frames go out, so the press and the
            # engaged CRZ_INFO land in the same window the PCM is evaluating.
            if (_setbtn["on"] and getattr(cc, "longActive", False)
                    and not getattr(cs_can, "acc_active", False)
                    and _setbtn["n"] < _setbtn["max"]
                    and (time.time() - _setbtn["t"]) >= 2.0):
                _setbtn["t"] = time.time()
                _setbtn["n"] += 1
                try:
                    _bmsg = mazdacan.create_button_cmd(
                        packer, CP, int(getattr(cs_can, "crz_btns_counter", 0)),
                        _Btns.SET_MINUS)
                    tx_due.set()
                    with panda_lock:
                        panda.can_send(_bmsg[0], bytes(_bmsg[1]), 0)
                    print("SETBTN  sent SET_MINUS on 0x%03x (attempt %d/%d) -- "
                          "longActive=1 but acc_active=0"
                          % (_bmsg[0], _setbtn["n"], _setbtn["max"]), flush=True)
                except Exception as _e:
                    print("SETBTN  failed:", _e, flush=True)
                finally:
                    tx_due.clear()
            elif getattr(cs_can, "acc_active", False) and _setbtn["n"]:
                # Latched: stop pressing the moment the PCM activates, and say so.
                print("SETBTN  acc_active reached 1 after %d press(es) -- the SET "
                      "handshake WAS the missing piece" % _setbtn["n"], flush=True)
                _setbtn["n"] = 0
                _setbtn["on"] = False

            if sends and (time.time() - _txlog["t"]) >= 1.0:
                _txlog["t"] = time.time()
                _seen_ids = {}
                for _m in sends:
                    if _m[0] in (CRZ_INFO_ADDR, CRZ_CTRL_ADDR):
                        _seen_ids.setdefault((_m[0], _m[2]), bytes(_m[1]))
                if _seen_ids:
                    print("LONGTX  " + "  ".join(
                        "0x%03x/bus%d=%s" % (a, b, d.hex(" "))
                        for (a, b), d in sorted(_seen_ids.items()))
                        + "  | longActive=%d enabled=%d accel=%+.2f"
                        % (int(getattr(cc, "longActive", False)),
                           int(getattr(cc, "enabled", False)),
                           float(getattr(cc.actuators, "accel", 0.0))),
                        flush=True)

            for msg in sends:
                if msg[0] not in LONG_ADDRS:
                    continue          # LKAS/HUD -- tx_thread owns those
                # When the local replay is running it owns the radar's seven other
                # frames, and it sends the OBSERVED bytes rather than the upstream
                # templates. Letting the carcontroller's heartbeat through as well
                # would put two different versions of 0x361-0x366/0x499 on the same
                # bus at the same rate -- worse than either alone, and exactly the
                # kind of contradiction a PCM would reject. Read the env directly:
                # the flag is assigned later in main() than this thread is defined.
                if _radar_local_shadow and msg[0] in RADAR_IDS:
                    continue
                # The carcontroller emits its own tester-present at
                # TESTER_PRESENT_STEP. It must obey radar_hold too, or the release
                # never happens: the keep-alive above would stop while this one
                # kept the session alive at 2 Hz, which is more than enough to hold
                # it, and the radar would never come back.
                if msg[0] == RADAR_ADDR and not radar_hold["on"]:
                    continue
                if (not tx_enabled or not longtx_gate["on"]) and msg[0] != RADAR_ADDR:
                    # longtx_gate is the ENGAGE-FIRST hold (OP_ALPHA_ENGAGE_FIRST).
                    # It starts closed in that mode so the radar keeps sole ownership
                    # of 0x21b/0x21c until the PCM has actually entered ACC, and the
                    # takeover watcher opens it. Always open otherwise, so every
                    # existing path behaves exactly as before.
                    #
                    # Diagnostic mode: hold the session open with tester-present but
                    # put nothing of ours on 0x21b/0x21c, so "radar silent" can be
                    # tested apart from "we are transmitting".
                    continue
                tx_due.set()
                try:
                    with panda_lock:
                        panda.can_send(msg[0], bytes(msg[1]), msg[2])
                    tx_state["long_tx_frames"] = tx_state.get("long_tx_frames", 0) + 1
                    # Per-address echo count. Everything we put on bus 0 comes
                    # back through can_thread and lands in can_census, so the
                    # census cannot tell our frames from the radar's by itself.
                    # The suppression census subtracts this; the 0x21b watchdog
                    # above does the same thing with its own crz_info_tx counter.
                    #
                    # Bus 0 only: the census reads can_census[0], and the frame
                    # list can carry the same address on bus 2 as well (the
                    # go-no-malfunction variant sends both). Counting a bus-2
                    # copy would over-subtract and make a live radar look quiet.
                    if msg[2] == 0:
                        _ourtx[msg[0]] = _ourtx.get(msg[0], 0) + 1
                    # ONE-SHOT DIFF against the radar's own frame, printed the first
                    # time we send each address. The PCM obeys the radar's 0x21b/0x21c
                    # and ignores ours (acc_active never reaches 1), so the bytes it
                    # accepts versus the bytes it discards is the whole question --
                    # and nothing was ever capturing the former to compare with.
                    if msg[0] in radar_pre and not tx_state.get("diff_%x" % msg[0]):
                        tx_state["diff_%x" % msg[0]] = True
                        _pre = radar_pre[msg[0]]["d"]
                        _our = bytes(msg[1])
                        if len(_pre) == len(_our) == 8:
                            _bits = "".join(
                                "^" if _pre[_i] != _our[_i] else " " for _i in range(8))
                            print("LONGDIFF 0x%03x  radar=%s  (n=%d)\n"
                                  "         0x%03x  ours =%s\n"
                                  "         differing bytes: %s"
                                  % (msg[0], _pre.hex(" "), radar_pre[msg[0]]["n"],
                                     msg[0], _our.hex(" "),
                                     " ".join(("%02d" % _i) for _i in range(8)
                                              if _pre[_i] != _our[_i]) or "(none)"),
                                  flush=True)
                            del _bits
                    if msg[0] == CRZ_INFO_ADDR:
                        # Counted separately from long_tx_frames (which also covers
                        # 0x21c and 0x764) so the watchdog above can subtract our
                        # own echo from the census and see only the radar's share.
                        tx_state["crz_info_tx"] = tx_state.get("crz_info_tx", 0) + 1
                    elif msg[0] == RADAR_ADDR:
                        # Tester-present. The probe proves 2 Hz of these holds the
                        # session for 90 s, so if the radar comes back here, the
                        # first question is whether these are actually going out
                        # under the 0x243 tx load -- not whether the radar timed out.
                        tx_state["radar_tp_tx"] = tx_state.get("radar_tp_tx", 0) + 1
                except Exception as e:
                    # Was a bare swallow. A tester-present that silently fails to
                    # send is exactly how the session lapses with no symptom.
                    tx_state["long_tx_err"] = tx_state.get("long_tx_err", 0) + 1
                    tx_state["long_tx_last_err"] = "%s @0x%x" % (type(e).__name__, msg[0])
                    time.sleep(0.002)
                finally:
                    tx_due.clear()

    def radar_shadow_thread():
        """Replay the radar's OTHER seven frames while it is suppressed.

        Alpha long silences the radar to take over 0x21b/0x21c, but the radar also
        sends 0x361-0x366 and 0x499, which we do NOT replace. Those simply vanish,
        and the forward camera -- which normally receives them -- is the module
        that reports "front camera sensor" on the cluster.

        Silencing it more politely is not an option: MEASURED 2026-08-09, this
        radar refuses CommunicationControl (0x28) and both the extended and
        safety-system diagnostic sessions, answering none of them, while accepting
        the programming session instantly. So the radar goes fully mute whatever we
        do, and the only remaining lever is to put its other frames back ourselves.

        Replay is only credible because these frames carry NO CHECKSUM. Five have a
        4-bit counter in the low nibble of byte 7 and two are static, so a replayed
        frame is indistinguishable from a real one except for its CONTENT, which is
        frozen at whatever the radar last said before it went quiet.

        THAT IS THE PART THAT MAY NOT WORK, and it is worth being honest about:
        0x361 carries DISTANCE_LEAD and RELATIVE_VEL_LEAD. If the camera checks
        those against its own vision, a frozen lead that never moves is exactly the
        kind of implausibility a sensor-fusion check exists to catch, and we will
        have traded one fault for another. Capture with nothing ahead if you can --
        a "no target" snapshot is the least likely to contradict anything.
        """
        plan = []
        for _addr in RADAR_IDS:
            _r = radar_shadow.get(_addr)
            if not _r or not _r.get("d") or _r["n"] < 5:
                continue
            # Rate from the capture window, not a constant. n-1 intervals span
            # t-t0, and a window shorter than half a second is too little to
            # divide by.
            _span = _r["t"] - _r.get("t0", _r["t"])
            _hz = (_r["n"] - 1) / _span if _span > 0.5 else 0.0
            if _hz < 1.0:
                continue
            plan.append({"addr": _addr, "d": bytearray(_r["d"]),
                         "period": 1.0 / _hz, "next": time.time(),
                         "ctr": _r["d"][7] & 0x0F,
                         "has_ctr": _addr in RADAR_CTR_IDS,
                         # SNAPSHOT, taken once here and never refreshed. It has
                         # to be: radar_shadow[addr]["t"] is updated by ANY frame
                         # at that address on either bus, which includes our own
                         # replay echoing back off bus 0 -- so re-reading it later
                         # would show our echo's age and never expire anything.
                         # This field is the age of THE BYTES, which is the thing
                         # the gate below is actually reasoning about.
                         "captured": _r.get("t", time.time()),
                         "static": _addr in RADAR_STATIC_IDS,
                         "expired": False})

        if not plan:
            print("radar shadow: NOTHING CAPTURED -- the radar was already quiet "
                  "before suppression, so there is nothing to replay. The camera "
                  "fault (if any) is not explained by these frames.")
            return
        print("radar shadow: replaying %d frame(s) -- %s"
              % (len(plan), ", ".join("0x%03x@%.0fHz%s"
                                      % (p["addr"], 1.0 / p["period"],
                                         "+ctr" if p["has_ctr"] else "")
                                      for p in plan)))

        sent = 0
        while not stop_flag["v"]:
            now = time.time()
            for p in plan:
                if now < p["next"]:
                    continue
                # AGE GATE. The docstring above calls a frozen lead "the part
                # that may not work"; it did not work. MEASURED 2026-08-11: a
                # snapshot replayed at age=113.7 s raised an FSB fault, because
                # a lead that never moves is exactly the implausibility a
                # sensor-fusion check exists to catch.
                #
                # So the dynamic frames get an expiry and the static ones do
                # not. 0x361 DISTANCE and 0x362 TURN carry real obstacle data
                # (see RADAR_IDS); 0x363-0x365 are undecoded and are treated as
                # dynamic because "unknown" is not "constant". 0x366 and 0x499
                # are the two the DBC has no varying signals for, so replaying
                # them indefinitely asserts nothing that can go stale.
                #
                # Letting a dynamic frame expire brings back the front camera
                # fault it was added to suppress. That is the correct trade: the
                # fault is honest about the radar being gone, where a frozen
                # lead is a false statement about the road ahead. expired is
                # surfaced so a run can be read without guessing which happened.
                if not p["static"] and (now - p["captured"]) > SHADOW_MAX_AGE_S:
                    if not p["expired"]:
                        p["expired"] = True
                        radar_shadow_state["expired"] = \
                            radar_shadow_state.get("expired", 0) + 1
                        print("radar shadow: 0x%03x EXPIRED after %.0f s -- its "
                              "content is %.0f s old and no longer describes the "
                              "road. Replay stopped for this address; expect the "
                              "front camera fault to return."
                              % (p["addr"], now - p["captured"], now - p["captured"]),
                              flush=True)
                    continue
                if p["has_ctr"]:
                    p["ctr"] = (p["ctr"] + 1) & 0x0F
                    p["d"][7] = (p["d"][7] & 0xF0) | p["ctr"]
                try:
                    # BOTH BUSES. This used to be bus 0 only, which is exactly why
                    # the local replay was retired in favour of the carcontroller
                    # heartbeat: the forward camera lives on bus 2 and normally
                    # RECEIVES the radar's frames, so a bus-0-only replay left it
                    # with nothing and raised the front camera fault.
                    #
                    # But the heartbeat trades that fault for WRONG CONTENT. It
                    # sends RADAR_TRACK_EMPTY_TEMPLATES, fixed bytes from the
                    # upstream port, and they do not match this car. MEASURED
                    # 2026-08-11 against a census of the real radar:
                    #     0x365  upstream fff7fe7ffbff3fc0
                    #            this car 2faebe7ffbff07c8
                    # -- the first four bytes differ, so that is content, not the
                    # counter nibble the other addresses differ by. We were feeding
                    # the PCM a frame this radar never sends.
                    #
                    # Replaying the OBSERVED frames on BOTH buses is the combination
                    # that was never tried: real content, and the camera still fed.
                    # MAZDA_LONG_TX_MSGS already permits all seven on bus 2.
                    with panda_lock:
                        for _b in (0, 2):
                            panda.can_send(p["addr"], bytes(p["d"]), _b)
                    sent += 1
                    # ONE, not two. The replay goes out on both buses but the
                    # census this feeds counts bus 0 only, so counting the bus-2
                    # copy here would over-subtract and show the radar as more
                    # silent than it is -- the exact direction of error that
                    # would hide a radar coming back.
                    _ourtx[p["addr"]] = _ourtx.get(p["addr"], 0) + 1
                except Exception as _e:
                    # NOT a bare swallow. MEASURED 2026-08-11: this thread printed
                    # "replaying 7 frame(s)" and then transmitted NOTHING for whole
                    # runs -- d_p.static21c.log and d_p.dist2.log show the replay
                    # frozen (age 85 s and 220 s) while d_p.21cmatched.log shows it
                    # healthy, same code and same flags. With the failure swallowed
                    # and `sent` never surfaced, a dead shadow is indistinguishable
                    # from a live one, and several conclusions about the dashboard
                    # were drawn from runs where it silently was not running.
                    radar_shadow_state["err"] = radar_shadow_state.get("err", 0) + 1
                    radar_shadow_state["last_err"] = "%s @0x%03x" % (
                        type(_e).__name__, p["addr"])
                # Advance by whole periods rather than from `now`, so a late wake
                # does not permanently shift this frame's phase. If we have fallen
                # more than a period behind, resynchronise instead of trying to
                # catch up with a burst -- a burst is more suspicious to a
                # consumer than a missed frame.
                p["next"] += p["period"]
                if p["next"] < now:
                    p["next"] = now + p["period"]
            radar_shadow_state["sent"] = sent
            time.sleep(0.002)

    def carstate_thread():
        # Publish carState at a steady ~110 Hz. selfdrived's loop is gated by a
        # blocking recv_one(carState, 20ms), so this rate IS selfdrived's loop rate;
        # 110 Hz keeps it under 10 ms (fixes selfdrivedLagging) and stays inside
        # controlsd's 100 Hz [80,120] freq band. Minimal work per iteration so it
        # holds rate even while the model thread holds the GIL for preprocessing.
        if pm_cs is None:
            return
        period = 1.0 / 110.0
        # one-shot startup hold, so cruiseState.enabled has a rising
        # edge even when MRCC MAIN was already on before we started
        cs_edge = {"t0": time.time(), "hold_s": 3.0,
                   "pulse_until": 0.0, "btn_prev": False}
        nxt = time.time()
        while not stop_flag["v"]:
            m = messaging.new_message('carState')
            cs = m.carState
            cs.vEgo = cs_can.v_ego; cs.vEgoRaw = cs_can.v_ego
            # STEERING ANGLE SIGN. MEASURED 2026-08-08 from a 250 Hz trace,
            # n=19426 over 104 s engaged:
            #   corr(curvature, torque_norm) = +0.599   command direction OK
            #   corr(torque_norm, applied)   = +0.755   carcontroller OK
            #   corr(applied,   eps_eff)     = +0.698   rack follows us
            #   corr(curvature, angle)       = -0.902   <-- ONLY inverted link
            # Every link in the command chain agrees; the reported angle is the
            # odd one out. Positive applied torque yields positive eps_eff but a
            # NEGATIVE steering angle, so this signal runs opposite to openpilot's
            # convention (positive = left, matching positive curvature).
            #
            # LatControlTorque feeds it straight into
            #   measured_curvature = -VM.calc_curvature(radians(steeringAngleDeg - offset), ...)
            #   error = setpoint - measurement
            # so a flipped measurement turns the loop from negative into POSITIVE
            # feedback: error grows instead of shrinking and it oscillates by
            # construction. That is the 1.09 Hz ring, and it is why kp 1.0->0.5,
            # ki 0.3->0.10 and delta_up 40->10 all barely moved it -- they were
            # gains on a loop with the wrong sign.
            #
            # Only the FEEDBACK term is affected, which is why the car still
            # steered the correct way and it presented as jitter rather than
            # pulling the wrong direction.
            #
            # DEFAULT LEFT AT +1 -- the measurement does NOT identify which end is
            # wrong. corr(curvature, angle) = -0.902 proves the two DISAGREE, but
            # not which one sits in openpilot's convention. Negating the angle and
            # negating the curvature both drive that correlation positive, and
            # they have opposite consequences: fixing the angle repairs the
            # feedback sign (a stability fix), while negating a correct angle
            # would BREAK a working feedback path. The steering angle decode also
            # matches the DBC exactly (23|16@0+ (0.05,-1600)) and is the same
            # signal opendbc's own Mazda carstate reads, so it is the better
            # supported of the two.
            #
            # The likelier suspect is the CURVATURE end: the camera was flipped
            # 180 deg in the pipeline today (nvvidconv flip-method=2). If the
            # correct correction were a mirror rather than a rotation, the model
            # would see left and right swapped and emit inverted curvature --
            # which produces exactly this correlation.
            # OP_STEER_ANGLE_SIGN=-1 to A/B the other hypothesis.
            cs.steeringAngleDeg = STEER_ANGLE_SIGN * cs_can.steering_angle
            cs.steeringTorque = cs_can.steering_torque
            cs.steeringPressed = cs_can.steering_pressed
            cs.brakePressed = cs_can.brake_pressed
            cs.gasPressed = cs_can.gas_pressed
            cs.standstill = cs_can.v_ego < 0.3
            cs.gearShifter = car_struct.CarState.GearShifter.drive
            # DesireHelper reads all four of these. Blinkers are debounced in
            # CarStateFromCAN; blindspot is the lane-change veto and is False
            # until 0x477 is seen, so an unparsed BSM would silently disable the
            # veto rather than block the manoeuvre.
            cs.leftBlinker = cs_can.left_blinker
            cs.rightBlinker = cs_can.right_blinker
            cs.leftBlindspot = cs_can.left_blindspot
            cs.rightBlindspot = cs_can.right_blindspot
            cs.canValid = cs_can.seen > 0
            cs.cruiseState.available = cs_can.cruise_available
            # MADS: lateral engages on cruise MAIN, not on ACC being set.
            #
            # Stock Mazda ACC will not set below ~19 mph, and clip_curvature caps
            # lateral accel at 3.0 m/s^2 -- so max curvature is 3.0/v^2, i.e. a
            # 24 m minimum radius at 19 mph. The planner CANNOT ask for a tighter
            # turn at that speed regardless of how much torque is available, which
            # is why turns felt wide. Below ~11 mph the same limit allows an 8 m
            # radius. So the fix for tight turns is being allowed to go slow, and
            # that means engaging lateral without ACC holding the speed up.
            #
            # openpilotLongitudinalControl is False on this car, so openpilot never
            # commands throttle or brake either way -- this only changes WHEN the
            # lateral controller is allowed to run. The driver keeps gas and brake.
            #
            # The panda enforces its own gate independently (mazda_rx_hook ->
            # pcm_cruise_check), so this flag alone does nothing without the
            # matching firmware build. That is deliberate: two independent
            # switches, and the hardware one still has to be flashed on purpose.
            #
            # Under alpha long the source changes. CRZ_CTRL is OUR frame once the
            # radar is suppressed, so cruise_available/cruise_enabled would be us
            # reading our own output back. PEDALS is the PCM's own report and is
            # the same signal the panda gates on, so the two stay in agreement.
            if alpha_long:
                cs.cruiseState.available = cs_can.acc_armed
                # The SAME edge trap the panda had, on the software side:
                # car_specific.py raises pcmEnable only on a RISING edge of
                # cruiseState.enabled. Wiring it to acc_armed alone leaves it high
                # for the whole drive, so after any disengage (pedalPressed) there
                # is no new edge and openpilot never re-enables -- the driver has
                # to cycle MRCC MAIN. Dropping it on brake gives the edge back:
                # press to disengage, release to re-engage, which is what mazda.h
                # now does with controls_allowed so the two stay in step.
                # ALPHA LONG MUST USE acc_armed TOO, not acc_active.
                #
                # acc_active is the stock ACC reporting itself engaged -- and the
                # stock ACC lives in the radar, which alpha long has just muted.
                # Keying engagement off it is circular: we suppress the ACC ECU and
                # then wait for the ACC to say it is active, which it never can.
                #
                # MEASURED 2026-08-09 over 595 samples / 596 s of a live alpha-long
                # run: acc_active NEVER set once, while acc_armed was set for 445
                # of them and the panda's own controls_allowed was true for 400.
                # The panda was willing the whole time; only this line was not, and
                # because nothing was REFUSING, openpilot logged no events at all --
                # op=disabled with an empty EVT line, which is the hardest possible
                # symptom to trace.
                #
                # The panda already resolved this: mazda.h:210 gates alpha long on
                # `acc_armed && !brake`, where acc_armed comes from PEDALS, chosen
                # precisely because PEDALS stays truthful with the radar silent.
                # Use the same signal here so the two halves of the stack agree --
                # a host that engages on a different flag than the panda enforces is
                # a disagreement waiting to surface as exactly this.
                # ...THAT ARGUMENT IS WRONG, and it is what has blocked alpha long.
                #
                # It assumes the radar OWNS ACC, so muting the radar must make
                # acc_active unreachable. The DBC says otherwise: CRZ_INFO carries
                # ACC_SET_ALLOWED (34|1@0+), a PERMISSION bit. The radar tells the
                # PCM whether the driver may engage; the PCM does the engaging and
                # reports it back on PEDALS. That split is exactly why the vendored
                # port sets pcmCruise = True. With the radar muted WE own the
                # permission bit, so acc_active is still reachable -- provided we
                # present the ready state and let the driver press SET.
                #
                # Engaging off acc_armed inverts the sequence. openpilot enables the
                # moment MRCC MAIN comes on, so the very next CRZ_INFO asserts
                # ACC_ACTIVE=1 with a full ACCEL_CMD while the PCM knows cruise was
                # never set -- the radar claiming the system is running when the PCM
                # never started it. It refuses, every time.
                #
                # MEASURED 2026-08-11, decoded at the transmit point: all three
                # states ARE produced correctly --
                #   ACC_ACTIVE=0 ACC_SET_ALLOWED=0 ACCEL_CMD=+4094   standby
                #   ACC_ACTIVE=0 ACC_SET_ALLOWED=1 ACCEL_CMD=    +0  ready
                #   ACC_ACTIVE=1 ACC_SET_ALLOWED=1 ACCEL_CMD= +2000  engaged
                # -- but we never SAT in the ready state while the driver pressed
                # SET; MAIN took us straight to commanding. And across every log
                # this port has, acc_active=1 appears 769 times and ALL 769 have the
                # radar transmitting (crz_avail=1). Not once from our own frames.
                #
                # The 2026-08-09 measurement quoted above is real but proves less
                # than it claims: acc_active never set in that run because we never
                # gave the PCM a state it could act on, not because it cannot.
                #
                # The panda stays permissive (mazda.h gates on acc_armed && !brake),
                # so the host is now the stricter of the two gates. That is the
                # correct direction and needs no reflash.
                #
                # OP_ALPHA_ENGAGE=armed restores the old behaviour for A/B without a
                # restart of the argument.
                if CP.pcmCruise:
                    # Legacy paths, kept for A/B while pcmCruise is still True --
                    # i.e. only if interface.py's alpha-long flip is reverted.
                    if os.environ.get("OP_ALPHA_ENGAGE", "pcm") == "pcm" and not mads:
                        cs.cruiseState.enabled = bool(cs_can.acc_active)
                    else:
                        cs.cruiseState.enabled = ((cs_can.acc_armed and not cs_can.brake_pressed)
                                                  if (mads or alpha_long) else cs_can.acc_active)
                else:
                    # OPENPILOT OWNS ENGAGEMENT. cruiseState.enabled is the CAR's
                    # report, and with pcmCruise False the car has none to give --
                    # ACC_ACTIVE never rises once the radar is suppressed. It must
                    # also stay False for a second reason: selfdrived.py:425 makes
                    # ANY true value here a permanent cruiseMismatch when
                    # pcmCruise is False.
                    #
                    # Engagement comes from buttonEnable below instead.
                    cs.cruiseState.enabled = False
            else:
                # MADS engagement edge, WITHOUT disengaging on every brake.
                #
                # car_specific.py:46 raises pcmEnable only on a RISING edge of
                # cruiseState.enabled. Wired straight to cruise_available that
                # signal is HIGH for the whole drive, so if the stack starts with
                # MRCC MAIN already on there is NO edge and openpilot can never
                # enable -- op=disabled with trq=+0 while the panda's
                # controls_allowed sits happily latched. That is why --alpha-long
                # engaged and plain --mads did not.
                #
                # The first fix for that ANDed in `not brake_pressed`, which does
                # produce an edge -- but a FALLING one on every brake press too,
                # so openpilot disengaged at every stop (333 pcmDisable events in
                # one drive) and re-engaged on release. That defeats the point of
                # MADS, which exists so the driver can work the pedals while
                # lateral keeps steering.
                #
                # Instead hold the signal low for the first HOLD_S after this
                # thread starts, then follow MAIN alone. That manufactures exactly
                # ONE rising edge -- when the hold expires with MAIN already on, or
                # later when MAIN is switched on -- and never takes it away for
                # braking. openpilot's own pedalPressed still disengages on brake
                # (as does the panda's controls_allowed), so this does not grant
                # any authority the stack did not already have; it only stops the
                # engage/disengage flapping this signal was adding on top.
                # RE-ENGAGE after a disengage.
                #
                # The one-shot hold above fixed "never engages", but created
                # "never engages AGAIN": once it expires, enabled simply follows
                # MRCC MAIN, which stays high for the whole drive. So after any
                # disengage (pedalPressed on brake, say) there is no second
                # RISING edge and pcmEnable can never fire -- pressing RES/SET
                # does nothing, because those move acc_active, not MAIN. The only
                # way back was to physically cycle the MAIN switch.
                #
                # So treat a RES/SET press while openpilot is disabled as an
                # explicit re-engage request: drop the signal LOW for 150 ms,
                # which on release gives car_specific.py the rising edge it wants.
                # Only when openpilot is actually disabled, so a mid-drive
                # set-speed adjustment cannot disengage anything.
                _now = time.time()
                _btn = bool(cs_can.buttons.get("res") or cs_can.buttons.get("set_p")
                            or cs_can.buttons.get("set_m"))
                if _btn and not cs_edge["btn_prev"] and not tx_state.get("op_enabled"):
                    cs_edge["pulse_until"] = _now + 0.15
                cs_edge["btn_prev"] = _btn
                _held = ((_now - cs_edge["t0"]) < cs_edge["hold_s"]
                         or _now < cs_edge["pulse_until"])
                cs.cruiseState.enabled = ((cs_can.cruise_available and not _held)
                                          if mads else cs_can.cruise_enabled)
            cs.cruiseState.speed = 25.0

            # BUTTON EVENTS. This port builds carState by hand instead of running
            # opendbc's CarState, so nothing here has ever published buttonEvents
            # or buttonEnable -- they simply did not exist on the bus. That was
            # invisible while pcmCruise was True, because engagement came from the
            # car. With pcmCruise False they ARE the engagement path, and without
            # them openpilot can never enable no matter what the panda allows.
            #
            # opendbc's equivalents, for reference: create_button_events() in
            # carstate.py builds these, and CarInterfaceBase.update()
            # (interfaces.py:261) derives buttonEnable from them.
            _btn_now = {
                "res": bool(cs_can.buttons.get("res")),
                "set_m": bool(cs_can.buttons.get("set_m")),
                "off": bool(cs_can.buttons.get("off")),
            }
            _btn_prev = cs_edge.setdefault("btn_state", dict.fromkeys(_btn_now, False))
            _events = [(_k, _btn_now[_k]) for _k in _btn_now if _btn_now[_k] != _btn_prev[_k]]
            cs_edge["btn_state"] = _btn_now

            if _events:
                _types = {"res": "accelCruise", "set_m": "decelCruise", "off": "cancel"}
                _be = cs.init('buttonEvents', len(_events))
                for _i, (_k, _pressed) in enumerate(_events):
                    _be[_i].type = _types[_k]
                    _be[_i].pressed = _pressed

            # buttonEnable: "user is requesting enable, usually one frame"
            # (car.capnp:215). MazdaCarState.update_button_enable() defines the
            # rule for this car -- RESUME on press, SET on release -- and the
            # panda's CRZ_BTNS branch in mazda.h arms on exactly the same two
            # edges, deliberately. If these two ever disagree, openpilot engages
            # while the panda drops every frame, which looks like a car ignoring
            # us rather than a gate that has not opened.
            cs.buttonEnable = (not CP.pcmCruise) and any(
                (_k == "res" and _pressed) or (_k == "set_m" and not _pressed)
                for _k, _pressed in _events)

            m.valid = True
            pm_cs.send('carState', m)

            # carOutput at this thread's rate, pandaStates decimated to ~10 Hz.
            # Both used to ride the shared publish loop, which in the panda role
            # runs at ~52 Hz rather than the 250 it is paced for -- putting them
            # under selfdrived's 0.8x floor and soft-disabling lateral every few
            # seconds. This thread holds 110 Hz, which is what they need.
            if pm_fast is not None:
                _fp = cs_edge.setdefault("fast_n", 0)
                cs_edge["fast_n"] = _fp + 1
                try:
                    if 'carOutput' in _fast_list:
                        _co = messaging.new_message('carOutput')
                        _co.valid = True
                        pm_fast.send('carOutput', _co)
                    # 110 Hz / 11 = 10 Hz, the nominal for pandaStates.
                    if 'pandaStates' in _fast_list and _fp % 11 == 0:
                        _hv = health_state["v"]
                        _pm = messaging.new_message('pandaStates', 1)
                        _ps = _pm.pandaStates[0]
                        _ps.pandaType = log.PandaState.PandaType.dos
                        if _hv is not None:
                            _ps.uptime = int(_hv[0])
                            _ps.ignitionLine = bool(_hv[8])
                            _ps.ignitionCan = bool(_hv[9])
                            _ps.controlsAllowed = bool(_hv[10])
                            _ps.safetyModel = 'mazda'
                            _ps.safetyParam = int(_hv[13])
                            _ps.rxBufferOverflow = int(_hv[6])
                            _ps.txBufferOverflow = int(_hv[5])
                            _ps.heartbeatLost = bool(_hv[16])
                            _ps.powerSaveEnabled = bool(_hv[15])
                        _pm.valid = True
                        pm_fast.send('pandaStates', _pm)
                except Exception:
                    pass
            nxt += period
            slp = nxt - time.time()
            time.sleep(slp) if slp > 0 else (nxt := time.time())

    # --- on-screen window (DisplayPort) instead of the HTTP preview ------------
    # The JPEG preview exists to get a frame across a network to a phone. When the
    # output is a monitor wired to this board, encoding to JPEG and decoding it again
    # is pure loss and pure cost, so --display swaps the whole preview+HTTP path for
    # an appsrc window and does not start the encoder at all.
    win = {"w": None}

    def _no_camera_frame(t0):
        """Placeholder so the window still opens when no frame has arrived.

        Without this the window is created lazily off the first frame, so a camera
        that never enumerates looks identical to a broken --display flag: nothing
        appears and nothing is printed. Show the state instead."""
        img = np.zeros((720, 1280, 3), np.uint8)
        img[:] = (28, 24, 22)
        el = time.time() - t0
        lines = [
            ("NO CAMERA FRAMES", (60, 60, 235)),
            (f"waiting {el:5.1f}s   op_camera has produced nothing yet", (200, 200, 200)),
            ("", None),
            ("check:  ls /dev/video*        (nvarguscamerasrc needs it)", (170, 170, 170)),
            ("        dmesg | grep imx477  (driver probe result)", (170, 170, 170)),
            ("the rest of the dashcam (CAN, model, publish) is still running",
             (120, 220, 120)),
        ]
        y = 220
        for text, col in lines:
            if text:
                cv2.putText(img, text, (80, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(img, text, (80, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            col, 1, cv2.LINE_AA)
            y += 42
        return img

    def display_thread():
        from op_display import DisplayWindow
        w = None
        placeholder = False        # is the open window the 720p "no camera" one?
        period = 1.0 / max(1.0, float(getattr(args, "display_fps", 30.0)))
        nxt = time.time()
        t_start = time.time()

        def _open(width, height, note):
            nonlocal w
            ww = DisplayWindow(width, height, title="jetson dashcam",
                               fps=int(1.0 / period))
            win["w"] = ww
            print(f"display: {ww.sink_name} on DISPLAY={ww.display} {note}", flush=True)
            return ww

        while not stop_flag["v"]:
            with cap_lock:
                fr = cap["frame"]
            if fr is None:
                # open the window anyway and say why it is empty
                if w is None:
                    w = _open(1280, 720, "(no camera frames yet)")
                    placeholder = True
                w.push(_no_camera_frame(t_start))
                if w.error():
                    w.close(); w = None; placeholder = False
                    win["w"] = None; time.sleep(2.0)
            if fr is not None:
                h_, w_ = fr.shape[:2]
                # The placeholder is a fixed 1280x720. Once real frames arrive, rebuild
                # at their size -- pushing 1080p into it would silently downscale the
                # frame the whole point of the 1080p capture change was to preserve.
                if w is not None and placeholder:
                    w.close(); w = None; placeholder = False
                if w is None:
                    w = _open(w_, h_, f"({w_}x{h_} frames)")
                with LOCK:
                    snap = dict(STATE)
                    snap["_path"] = STATE.get("_path")
                    snap["_model"] = STATE.get("_model")
                try:
                    mode = getattr(args, "display_view", "pip")
                    with LOCK:
                        sc = SCENE_FRAME["img"]
                        dp = DEPTH_FRAME["img"]
                        oc = OCC_FRAME["img"]
                    if mode == "scene" and sc is not None:
                        # the 3D view alone, filling the window
                        out = cv2.resize(sc, (w_, h_), interpolation=cv2.INTER_LINEAR)
                    elif mode == "depth" and dp is not None:
                        out = cv2.resize(dp, (w_, h_), interpolation=cv2.INTER_LINEAR)
                    elif mode == "occ" and oc is not None:
                        # letterboxed, not stretched: this pane is metric, and a
                        # non-uniform scale would make a square cell read as a
                        # rectangle and the grid lie about its own proportions
                        k = min(w_ / oc.shape[1], h_ / oc.shape[0])
                        r = cv2.resize(oc, (int(oc.shape[1] * k), int(oc.shape[0] * k)),
                                       interpolation=cv2.INTER_NEAREST)
                        out = np.zeros((h_, w_, 3), np.uint8)
                        y0 = (h_ - r.shape[0]) // 2; x0 = (w_ - r.shape[1]) // 2
                        out[y0:y0 + r.shape[0], x0:x0 + r.shape[1]] = r
                    else:
                        out = overlay(fr, snap, minimal=True)
                        if mode == "pip" and sc is not None:
                            _blit_pip(out, sc)
                        # depth goes bottom-LEFT, opposite the 3D scene, so the two
                        # insets never overlap and the HUD text along the top stays
                        # clear of both
                        if mode == "pip" and dp is not None:
                            _blit_pip(out, dp, corner="bl", label="depth v2 (metric)")
                        # occupancy top-right: the two bottom corners are taken, and
                        # the HUD text runs along the top LEFT
                        if mode == "pip" and oc is not None:
                            _blit_pip(out, oc, frac=0.26, corner="tr",
                                      label="free space")
                    w.push(out)
                except Exception as e:
                    print("display push failed:", e, flush=True)
                err = w.error()
                if err:
                    # Do not tear the whole dashcam down for a dead window -- the CAN
                    # and model loops are the point of this process, the preview is not.
                    print("display pipeline error:", err, flush=True)
                    try:
                        w.close()
                    except Exception:
                        pass
                    w = None
                    win["w"] = None
                    time.sleep(2.0)
            nxt += period
            slp = nxt - time.time()
            time.sleep(slp) if slp > 0 else (nxt := time.time())

    use_display = bool(getattr(args, "display", False))

    # In --can-only there is no frame to capture, encode or infer on, so every
    # one of these would either crash on `cam is None` or spin on an empty
    # buffer. can_thread / carstate_thread / tx_thread below still start: those
    # ARE the mode.
    if not can_only:
        threading.Thread(target=capture_thread, daemon=True).start()
        if use_display:
            threading.Thread(target=display_thread, daemon=True).start()
        else:
            threading.Thread(target=preview_thread, daemon=True).start()
    # The 3D scene feeds both paths: /scene.mjpg on the web, and the window
    # composite (--display-view) on a local monitor.
    if not args.no_scene:
        threading.Thread(target=scene_thread, daemon=True).start()
    if not args.no_detect:
        threading.Thread(target=det_thread, daemon=True).start()
    # the ground layer needs both: seg_thread produces the class map, scene_thread
    # is the only consumer, so --no-scene makes the network pure waste
    if not args.no_roadseg and not args.no_scene:
        threading.Thread(target=seg_thread, daemon=True).start()
    if not args.no_depth:
        threading.Thread(target=depth_thread, daemon=True).start()
    # Both of these talk to the panda directly, so they belong to whichever
    # process owns the handle. In the model role carState is subscribed instead
    # of decoded (see _CarStateFromMsgq), and publishing it from here as well
    # would put two writers on one topic.
    if owns_panda:
        threading.Thread(target=can_thread, daemon=True).start()
    if not can_only:
        threading.Thread(target=model_thread, daemon=True).start()
    if owns_panda:
        threading.Thread(target=carstate_thread, daemon=True).start()

    # Checked whether or not we arm: in dashcam mode it is a free warning that the
    # tree has drifted, which is exactly when you want to know rather than on the
    # first armed drive after a rebuild.
    _limit_complaints = check_safety_limits(lkas_hz, CarControllerParams)
    if _limit_complaints:
        print("\n" + "*" * 64)
        print("** PANDA LIMIT CHECK")
        for c in _limit_complaints:
            print("**  " + c)
        print("**  (mazda.h is the SOURCE -- this cannot see what is actually flashed)")
        print("*" * 64)
    else:
        print("panda limit check: sender and mazda.h agree, ramp fits max_rt_delta at %g Hz"
              % lkas_hz)

    # The OTHER limit on --lkas-hz, and the one mazda.h knows nothing about: a
    # can_recv holds the serial link for ~15.5 ms and cannot be preempted once
    # started. Ask for a period shorter than that and the link scheduler stops
    # being able to place it in a gap -- it then runs the recv straight after a
    # send, and the achieved rate settles near 1/(period + recv) no matter what
    # was requested. Say so here rather than letting the log explain it later.
    # ~15.5 ms profiled recv + ~1.5 ms for the send itself. Both are estimates
    # here; the loop measures the real recv cost and reports it as lkas_recv_ms.
    _recv_ms_est, _send_ms_est = 15.5, 1.5
    if 1000.0 / lkas_hz < _recv_ms_est + 2.0:
        _cycle = _recv_ms_est + _send_ms_est
        print("note: --lkas-hz %g asks for a %.1f ms period, shorter than one "
              "can_recv (~%.1f ms)."
              % (lkas_hz, 1000.0 / lkas_hz, _recv_ms_est))
        print("      The link then runs send/recv back to back, so expect ~%.0f Hz "
              "at a REGULAR" % (1000.0 / _cycle))
        print("      ~%.0f ms interval rather than %g Hz -- the floor is the cycle "
              "cost, not the period." % (_cycle, lkas_hz))
        print("      Regularity is what the EPS faults on, so this is a fair trade "
              "against 50 Hz")
        print("      with 30-40 ms holes -- but watch lkas_hz / lkas_worst_ms / "
              "lkas_rx_overruns")
        print("      rather than trusting the target.")

    if armed:
        print("\n" + "!" * 64)
        print("!!  ARMED -- REAL TRANSMIT to the car's steering bus (0x243 CAM_LKAS)")
        print("!!  The panda Mazda safety model is the hardware net: it passes a")
        print("!!  steering frame ONLY when the car reports cruise engaged, and")
        # Read from the params rather than hardcoded, so raising STEER_MAX can
        # never leave this banner quietly claiming the old ceiling.
        print("!!  clamps torque<=%d, rate_up<=%d, rate_down<=%d."
              % (CarControllerParams.STEER_MAX, CarControllerParams.STEER_DELTA_UP,
                 CarControllerParams.STEER_DELTA_DOWN))
        print("!!  Car STATIONARY / wheels up, hand on the wheel, kill-switch ready.")
        print("!!  Ctrl-C returns the panda to SAFETY_SILENT.")
        print("!" * 64)
        for s in range(5, 0, -1):
            print("   arming in %d ..." % s); time.sleep(1.0)
        # The THIRD argument is the safety PARAM, and mazda_init() reads bit 0 of it
        # to choose between the stock tx table and the longitudinal one. Left at 0
        # the panda would reject every 0x21b/0x21c frame -- alpha long would look
        # like "the car ignores our acceleration" rather than "the firmware is in
        # the wrong mode". Must match MAZDA_PARAM_LONGITUDINAL in mazda.h.
        safety_param = 1 if alpha_long else 0
        # Stash it so the tx_thread watchdog can repair the param, not just the
        # mode. It used to re-assert with a hardcoded 0 and turn alpha long off.
        tx_state["safety_param"] = safety_param
        with panda_lock:
            panda.control_write(0xdc, SAFETY_MAZDA, safety_param)
        time.sleep(0.2)
        # READ IT BACK. Every other "is the firmware in the right mode?" question
        # this port has asked was answered by inference; this one is one control
        # transfer away. health_t: [12] safety_mode, [13] safety_param.
        try:
            with panda_lock:
                _hh = panda.control_read(0xd2, 0, 0, 64)
            _hv = (struct.unpack(HEALTH_FMT, _hh[:struct.calcsize(HEALTH_FMT)])
                   if _hh and len(_hh) >= struct.calcsize(HEALTH_FMT) else None)
            if _hv and (_hv[12] != SAFETY_MAZDA or _hv[13] != safety_param):
                print("!!! PANDA REFUSED THE SAFETY CONFIG: mode=%d param=%d, "
                      "wanted mode=%d param=%d. Alpha long CANNOT work in this "
                      "state -- the tx hook will reject every 0x21b/0x21c."
                      % (_hv[12], _hv[13], SAFETY_MAZDA, safety_param), flush=True)
            else:
                print("   panda safety: mode=%d param=%d (alpha long %s)"
                      % (SAFETY_MAZDA, safety_param,
                         "ARMED" if safety_param & 1 else "off"), flush=True)
        except Exception as _e:
            print("   panda safety readback failed:", _e, flush=True)

        # --- radar suppression handshake -----------------------------------
        # CarInterface.init() is the hook that puts the radar into a UDS
        # programming session; stock openpilot calls it from card.py, which this
        # process replaces, so nothing was calling it. Without it the radar keeps
        # sending 0x21b/0x21c and long_tx_thread's frames land on top of them --
        # two writers on one address, independent counters, and the PCM sees both.
        #
        # Runs BEFORE tx_thread so the ISO-TP exchange has the serial link to
        # itself; the EPS only faults on an irregular 0x243 stream, and that
        # stream has not started yet. Needs the safety param written above, since
        # 0x764 is only in the panda's tx table under MAZDA_LONG_TX_MSGS.
        radar_suppressed = False

        # UDS TAP HELPERS, hoisted so BOTH suppression paths can reach them.
        # They used to live inside the immediate-suppression branch, which
        # engage_first deliberately skips -- so the takeover watcher closed over a
        # name that was never bound and every handover died with
        #   "cannot access free variable '_uds_can_recv'"
        # MEASURED ON THE CAR 2026-08-12: the handover fired correctly at
        # acc_active=1, the suppression then raised, and we transmitted alongside a
        # live radar -- two writers on 0x21b/0x21c, which is what put the yellow
        # radar warning on the cluster.
        from opendbc.car.can_definitions import CanData as _CanData
        from opendbc.car.mazda.longitudinal import CRZ_INFO_ADDR as _CRZ_INFO
        from opendbc.car.mazda.longitudinal import RADAR_ADDR as _RADAR_ADDR

        def _uds_can_recv(wait_for_one: bool = False):
            # One drain of the tap is one "packet". wait_for_one blocks briefly
            # rather than spinning, so the query does not burn its whole timeout
            # between sending the request and the reply arriving via can_thread.
            deadline = time.time() + 0.05
            while True:
                msgs = []
                while uds_sniff["q"]:
                    _a, _d, _b = uds_sniff["q"].popleft()
                    msgs.append(_CanData(_a, _d, _b))
                if msgs:
                    return [msgs]
                if not wait_for_one or time.time() > deadline:
                    return []
                time.sleep(0.002)

        def _uds_can_send(msgs):
            with panda_lock:
                for _m in msgs:
                    panda.can_send(_m.address, bytes(_m.dat), _m.src)

        # ENGAGE FIRST, SUPPRESS SECOND.
        #
        # Every failure this port has had is at the ACC ENTRY transition: the PCM
        # will not enter ACC on our synthesised CRZ_INFO/CRZ_CTRL, no matter how
        # exactly they reproduce the radar's. MEASURED 2026-08-12 with the frames
        # byte-identical to a real activation, the panda arming from the button,
        # openpilot enabled and commanding up to +0.99 m/s^2: acc_active stayed 0
        # across every sample of every run.
        #
        # So stop asking it to enter. Let the RADAR run the entry handshake it has
        # always run -- driver presses SET, radar activates ACC, PCM reports
        # acc_active=1 -- and only then take the frames over. The PCM is never
        # asked to start ACC, only to keep obeying something it already accepts.
        # SUSTAIN has never once been tested; only entry has, and only entry has
        # ever failed.
        #
        # Ordering matters: our frames go out FIRST and the radar is silenced a
        # beat later, so the PCM sees a brief overlap rather than a gap. A gap is
        # what would make it drop ACC.
        #
        # Failure here is benign and informative: if the PCM drops ACC when the
        # radar goes quiet, cruise disengages and the car coasts -- which is a
        # clean answer to "does it obey us at all".
        engage_first = os.environ.get("OP_ALPHA_ENGAGE_FIRST") == "1" and alpha_long and not coexist
        if engage_first:
            print(">>> ENGAGE-FIRST: radar left ALIVE. Press SET to engage stock ACC "
                  "normally;\n    we take over 0x21b/0x21c the moment the PCM reports "
                  "acc_active.\n    FCW/AEB/SBS stay live until that moment.")
            longtx_gate["on"] = False        # hold our tx until the handover
            radar_suppressed = True          # let the tx threads start (gated)
        if alpha_long and coexist:
            # No UDS, no session, no tester-present -- the radar is left entirely
            # alone. Nothing suppressed means nothing for the cluster to notice,
            # and FCW/AEB/SBS keep working throughout.
            print("radar suppression: SKIPPED (OP_ALPHA_COEXIST) -- the radar keeps "
                  "running and keeps FCW/AEB/SBS. We inject alongside it.")
        elif alpha_long and not engage_first:
            # engage_first defers this whole handshake to the takeover watcher --
            # the radar has to stay alive long enough to run the ACC entry itself.
            # _uds_can_recv / _uds_can_send / _CRZ_INFO / _RADAR_ADDR are defined
            # above the branch now, so the engage-first takeover can use them too.

            # 0x764 + response_offset 0x8. Without 0x76C the reply is filtered
            # out in the panda before it reaches the host and this always fails.
            # 0x21B and 0x202 RIDE ALONG, and they are not optional.
            #
            # This tap IS the can_recv handed to enter_radar_standby, so whatever it
            # filters out is invisible to the UDS layer. Limited to 0x76C, the
            # standby ladder could see the radar's diagnostic replies and nothing
            # else -- so its check for "did 0x21b actually stop, with the car
            # demonstrably awake" could never observe either signal.
            #
            # MEASURED 2026-08-11: it reported "quiet-check INCONCLUSIVE -- no
            # ENGINE_DATA, car asleep" while restore_radar.py, seconds earlier on
            # the same running car, counted 260 0x21b frames in 3 s. The car was
            # awake; the tap simply never carried the evidence.
            #
            # That check exists because the canonical 28 83 01 suppresses its own
            # response (see longitudinal.py), so watching the bus is the ONLY way to
            # tell whether standby landed. Blinding it guarantees the fallback to
            # the programming session -- the very outcome standby exists to avoid.
            uds_sniff["addrs"] = frozenset({_RADAR_ADDR + 0x8, _CRZ_INFO, 0x202})
            uds_sniff["q"].clear()
            uds_sniff["seen"] = 0
            uds_sniff["log"].clear()
            # SUPPRESS AT IGNITION, NOT AFTER.
            #
            # Every run so far silenced the radar with the car already fully awake,
            # because the stack is launched after the engine is started. By then the
            # cluster, camera and PCM have completed their power-on checks and have
            # SEEN a working radar -- so when it goes quiet they each log a
            # communication fault against it, and the dash reports the malfunction.
            #
            # Striking at ignition instead means the radar is muted before those
            # modules ever enumerate it. A module that never saw a radar has nothing
            # to miss. This is the sequence that reportedly produced a clean dash,
            # and it is the one thing the current flow structurally cannot do: with
            # the car asleep it reports INCONCLUSIVE and gives up.
            #
            # So when the bus is dead at launch, WAIT for it rather than quitting,
            # and run the ladder on the first sight of ENGINE_DATA. OP_SUPPRESS_WAIT
            # bounds the wait; 0 restores the old give-up behaviour.
            # Waits for a RISING EDGE of ignition, not merely for the bus to be
            # alive. Two reasons:
            #
            #   - If the car is already awake at launch, "0x202 is present" is true
            #     immediately and we would suppress in the same after-the-fact way
            #     that has produced the fault every time so far. The edge is the
            #     whole point.
            #
            #   - Launching against a DEAD bus is its own trap: CAN1 cannot leave
            #     initialisation without traffic, so the core wedges and 0x202 never
            #     arrives even after the key turns. Starting on a live bus and
            #     riding through a key cycle avoids that entirely.
            #
            # So: OP_SUPPRESS_AT_IGNITION=1, start the stack with the engine
            # RUNNING, then cycle the key. We watch 0x202 stop, then start, and fire
            # the ladder on the upswing.
            # WAIT FOR OUR OWN RECEPTION BEFORE JUDGING THE CAR.
            #
            # Every "is the car awake" test in this block reads can_census[0][0x202],
            # which can_thread fills. On a freshly restarted panda role that census
            # is empty for the first moment -- so a perfectly awake car reads as
            # asleep, suppression is declared INCONCLUSIVE, and alpha long disables
            # itself.
            #
            # MEASURED 2026-08-11: two consecutive runs reported "the car is asleep
            # (0x202 0 frames over 3.0 s)" with gear_rpm=750 and the radar visibly
            # transmitting seconds earlier. Several of today's alpha-long runs were
            # invalidated this way, and the resulting shadow_sent=0 was then read as
            # a shadow bug rather than a run that never started.
            #
            # So establish reception first, and only then decide anything.
            _rx_deadline = time.time() + 8.0
            while time.time() - 0 < _rx_deadline:
                _c0 = can_census[0].get(0x202, 0)
                time.sleep(0.3)
                if can_census[0].get(0x202, 0) - _c0 > 5:
                    break
                if time.time() > _rx_deadline:
                    print("radar suppression: no ENGINE_DATA after 8 s -- the car "
                          "really is asleep, or reception is not up")
                    break

            _wait_edge = os.environ.get("OP_SUPPRESS_AT_IGNITION") == "1"
            _ign_wait = float(os.environ.get("OP_SUPPRESS_WAIT", "300"))
            if _wait_edge and _ign_wait > 0:
                print("radar suppression: WAITING FOR AN IGNITION EDGE (up to %.0f s).\n"
                      "    Turn the car OFF, wait for the bus to go quiet, then ON.\n"
                      "    The radar is silenced at wake, before the cluster/camera/PCM\n"
                      "    enumerate it -- a module that never saw a radar has nothing\n"
                      "    to miss. Suppressing after the fact is what has been\n"
                      "    producing the malfunction." % _ign_wait)
                _w0 = time.time()
                _slept = False
                while time.time() - _w0 < _ign_wait:
                    _c0 = can_census[0].get(0x202, 0)
                    time.sleep(0.25)
                    _rate = can_census[0].get(0x202, 0) - _c0
                    if not _slept:
                        if _rate == 0:
                            _slept = True
                            print("    ignition OFF detected at %.1f s -- now waiting "
                                  "for wake" % (time.time() - _w0))
                    elif _rate > 2:
                        print("    IGNITION ON at %.1f s -- suppressing NOW"
                              % (time.time() - _w0))
                        break
                else:
                    print("    no ignition edge within %.0f s -- proceeding anyway "
                          "(this will be an after-the-fact suppression)" % _ign_wait)

                # LET THE RADAR FINISH ITS SELF-TEST FIRST.
                #
                # The fast path now reaches standby 0.48 s after ignition, and at
                # that point the radar has NOT finished starting up: it answers
                # 0x7f 10 22 conditionsNotCorrect for the first ~0.12 s, and the
                # dash's radar indicators -- which normally flash through a power-on
                # self-test -- instead go straight out. We are cutting the radar off
                # mid-init.
                #
                # That is a different failure from suppressing too late. A module
                # that watched the radar complete a healthy self-test and then go
                # quiet may well tolerate it; one that saw the self-test aborted has
                # every reason to log a fault. Racing to be first is only correct if
                # enumeration is the trigger -- and 5.54 s (after the self-test) and
                # 0.48 s (during it) BOTH produced the malfunction, which is exactly
                # what you would expect if neither extreme is the right moment.
                #
                # So make the moment adjustable and aim it at "just after the lights
                # stop flashing". Seconds, not milliseconds.
                _sup_delay = float(os.environ.get("OP_SUPPRESS_DELAY_S", "0"))
                if _sup_delay > 0:
                    print("    holding %.1f s before suppressing, so the radar can "
                          "complete its power-on self-test (watch the dash "
                          "indicators: suppress AFTER they stop flashing)"
                          % _sup_delay)
                    time.sleep(_sup_delay)
                    print("    self-test window elapsed -- suppressing now")

            # WHICH SUPPRESSION THIS RUN USED. interface.py picks between the
            # programming session and the 0x28 standby ladder off OP_RADAR_SUPPRESS
            # and says nothing about it, so every d_p.*.log to date is unattributable
            # -- and the two are not interchangeable. The programming session is what
            # the community port with on-car results uses (opendbc PR #3355) and it
            # makes every other module log a communication DTC against the radar;
            # standby is this port's gentler route, added to keep those DTCs off the
            # cluster. Whether either lets the car still engage is the open question,
            # and it cannot be answered from logs that do not record which ran.
            # Resolved the same way interface.py resolves it, including the
            # default, so this line cannot drift from what actually runs: the
            # default is standby only while longitudinal.py still HAS a standby
            # ladder, and the pristine vendored module does not.
            _has_standby = hasattr(_mazda_long, "enter_radar_standby")
            _sup_mode = os.environ.get(
                "OP_RADAR_SUPPRESS", "standby" if _has_standby else "programming")
            if not _has_standby:
                _sup_mode = "programming"
            print("radar suppression: mode=%s (OP_RADAR_SUPPRESS; "
                  "programming|standby|standby_keepdtc)" % _sup_mode)
            uds_sniff["on"] = True
            _t_hs = time.time()
            try:
                CarInterface.init(CP, _uds_can_recv, _uds_can_send)
            except Exception as e:
                print("radar suppression: CarInterface.init raised:", e)
            finally:
                uds_sniff["on"] = False
            # Diagnostic, not a gate. enter_radar_programming_session reports
            # failure even when the radar answers and goes quiet, so this says
            # whether the reply reached the host at all.
            print("radar suppression: handshake %.2fs, tap saw %d frame(s) on 0x%03x"
                  % (time.time() - _t_hs, uds_sniff["seen"], _RADAR_ADDR + 0x8))
            for _t, _a, _h in uds_sniff["log"]:
                print("    tap %.3f 0x%03x %s" % (_t - _t_hs, _a, _h))

            # Verify by observation, not by return value: init() discards the
            # bool from enter_radar_programming_session, and the only thing that
            # actually matters is whether the radar went quiet. Nothing of ours
            # is on 0x21b yet, so any traffic here is the radar still driving.
            #
            # Sustained silence, not one window. A single 0.5 s sample is not
            # enough: the radar goes quiet WHILE it is handling the UDS requests
            # and then resumes, and a short window lands inside that pause and
            # reads a transient as a state change. Require every window in a 3 s
            # span to be silent, and bail out the moment one is not.
            # SILENCE ONLY COUNTS IF THE CAR IS AWAKE.
            #
            # This test asks "is 0x21b quiet?" and calls quiet a success. An
            # unpowered radar is also quiet, so switching the ignition off looks
            # EXACTLY like a successful UDS takeover.
            #
            # OBSERVED 2026-08-09: the handshake failed outright -- "extended
            # session failed", zero frames on the 0x76C tap -- and this check
            # still printed "OK, radar silent" and started alpha long, because
            # the car had been switched off. A failed handshake was overridden
            # by a false positive, and the run transmitted 0x21b/0x21c believing
            # it had taken over a radar that simply had no power.
            #
            # So require the car to be demonstrably alive in the SAME windows:
            # 0x202 ENGINE_DATA is the CAN ignition source and runs at 100 Hz, so
            # if it is absent nothing can be concluded about the radar at all.
            _WIN, _SPAN, _TOL = 0.5, 3.0, 3
            radar_suppressed = True
            _seen_total = 0
            _ign_total = 0
            # HOLD THE SESSION WHILE MEASURING IT.
            #
            # CommunicationControl only lasts as long as tester-present keeps the
            # diagnostic session alive -- disable_ecu says so outright: "The ECU will
            # stay silent as long as openpilot keeps sending Tester Present." This
            # loop used to sleep for 3 s sending nothing, which is long enough for
            # the session to lapse and the radar to come back ON ITS OWN, mid-test.
            #
            # MEASURED 2026-08-11: enter_radar_standby reported "0x21b stopped" from
            # its own 1.2 s window, and this loop then counted 23 frames over 2.5 s
            # with the ignition alive -- and alpha long disabled itself. Both numbers
            # were right. The radar accepted 28 83 01, went quiet, timed out and
            # resumed, and the pair reads as a rejection when it is a lapse.
            #
            # long_tx_thread pumps these at 10 Hz once it starts; the gap is the
            # window between suppression and that thread existing, which is exactly
            # where this measurement lives.
            def _tp_pump(_dur):
                _t_end = time.time() + _dur
                while time.time() < _t_end:
                    tx_due.set()
                    try:
                        with panda_lock:
                            panda.can_send(_RADAR_ADDR,
                                           bytes([0x02, 0x3E, 0x80, 0, 0, 0, 0, 0]), 0)
                    except Exception:
                        pass
                    finally:
                        tx_due.clear()
                    time.sleep(0.1)

            for _i in range(int(_SPAN / _WIN)):
                _n0 = can_census[0].get(_CRZ_INFO, 0)
                _i0 = can_census[0].get(0x202, 0)
                _tp_pump(_WIN)
                _n = can_census[0].get(_CRZ_INFO, 0) - _n0
                _ign_total += can_census[0].get(0x202, 0) - _i0
                _seen_total += _n
                if _n > _TOL:               # ~21 frames per window at 43 Hz if alive
                    radar_suppressed = False
                    break
            if _ign_total < 10:
                # Far below the ~300 frames 0x202 should produce over 3 s.
                radar_suppressed = False
                print("radar suppression: INCONCLUSIVE -- the car is asleep "
                      "(0x202 %d frames over %.1f s). An unpowered radar is silent "
                      "too, so this proves nothing. Start the engine and retry."
                      % (_ign_total, (_i + 1) * _WIN))
            else:
                print("radar suppression: %s (0x21b %d frames over %.1f s, "
                      "ignition alive at %d frames)"
                      % ("OK, radar silent" if radar_suppressed else "FAILED, radar still transmitting",
                         _seen_total, (_i + 1) * _WIN, _ign_total))

        threading.Thread(target=tx_thread, daemon=True).start()
        print(">>> ARMED: streaming 0x243 at %g Hz -> ramp %g counts/s "
              "(torque still gated by the panda)\n"
              % (lkas_hz, lkas_hz * CarControllerParams.STEER_DELTA_UP))
        # COEXIST MODE: inject alongside a LIVE radar, without suppressing it.
        #
        # MEASURED 2026-08-11 at an ignition edge: the radar's first 0x21b lands
        # 14 ms after the bus wakes, and it refuses UDS with 0x7f 10 22
        # conditionsNotCorrect until 163 ms. We lose that race by 148 ms and no
        # amount of polling closes it -- the car ALWAYS hears the radar first, so
        # suppression is necessarily after the fact and the cluster always sees a
        # working radar go quiet.
        #
        # So stop trying to win a race we cannot win. Two senders on one CAN id is
        # legal: the frames arbitrate and interleave, and the PCM acts on what
        # arrives. The panda already permits this -- 0x21b/0x21c sit in
        # MAZDA_LONG_TX_MSGS with check_relay=false precisely so a radar that
        # resumes does not latch relay_malfunction.
        #
        # NOTHING IS SUPPRESSED HERE, so FCW/AEB/SBS keep working and the cluster
        # has nothing to complain about. That is the whole point. What is unknown
        # is whether the PCM obeys our frames, the radar's, or neither -- which is
        # exactly what the run will show.
        _coexist = os.environ.get("OP_ALPHA_COEXIST") == "1"
        if _coexist and alpha_long and not radar_suppressed:
            print(">>> COEXIST: radar NOT suppressed. Injecting 0x21b/0x21c "
                  "alongside it.\n    FCW/AEB/SBS remain ACTIVE. The radar is also "
                  "writing these frames;\n    the PCM sees both.")
            radar_suppressed = True          # let the tx threads start
        if alpha_long and radar_suppressed:
            # Started only when ARMED. Unarmed the panda is in SAFETY_SILENT and
            # would drop these anyway, but more to the point: the radar-suppression
            # UDS session is a real change to the car's state and should not happen
            # in dashcam mode.
            # ISOLATION SWITCHES. Alpha long does three things at once -- a UDS
            # handshake, a replay of seven shadow frames, and 0x21b/0x21c at 50 Hz --
            # and the dash fault has only ever been observed with all three running.
            # Timing has been ruled out (0.48 s, 5.54 s and 15.4 s all fault), so the
            # next question is WHICH of the three the car objects to, and that cannot
            # be answered while they are inseparable.
            #
            #   OP_ALPHA_NO_LONGTX=1   suppress + shadow, but send no 0x21b/0x21c
            #   OP_ALPHA_NO_SHADOW=1   suppress + 0x21b/0x21c, but no shadow replay
            #   both                   suppress and transmit NOTHING
            #
            # The last is the decisive one: if the fault still appears with the radar
            # silent and us sending nothing at all, then the car is reacting to the
            # radar's absence and no amount of frame-crafting will help. If it does
            # NOT appear, the fault is in something WE send, which is fixable.
            #
            # Tester-present still runs in both cases -- it lives in long_tx_thread's
            # keep-alive, and without it the session lapses and the radar returns,
            # which would invalidate the test rather than simplify it.
            _no_longtx = os.environ.get("OP_ALPHA_NO_LONGTX") == "1"
            # In coexist mode the real radar is still sending all seven of these,
            # so replaying them would put a SECOND copy of each on the bus -- two
            # sources for frames that carry a counter, which is far more likely to
            # upset a consumer than the frames simply being absent. The shadow
            # exists to cover for a silenced radar; there is no silence to cover.
            # engage_first is included for the same reason as coexist: the radar is
            # ALIVE for the first part of the run, so replaying its seven frames
            # would put a second counter-bearing copy of each on the bus. The cost
            # is that after the handover there is no shadow, so the front camera
            # fault may appear at that point -- acceptable for a test whose whole
            # question is whether the PCM keeps obeying, and preferable to
            # corrupting the entry handshake we are relying on.
            _no_shadow = coexist or engage_first or os.environ.get("OP_ALPHA_NO_SHADOW") == "1"
            if _no_longtx:
                print("!!! OP_ALPHA_NO_LONGTX: radar suppressed, but NOTHING will be "
                      "sent on 0x21b/0x21c. Diagnostic only -- the PCM has no "
                      "longitudinal source, so cruise will not work.")
            threading.Thread(target=long_tx_thread, daemon=True,
                             kwargs={"tx_enabled": not _no_longtx}).start()

            if engage_first:
                def takeover_thread():
                    """Wait for the RADAR to enter ACC, then take the frames over.

                    The handover order is deliberate: open our tx first, let it
                    overlap the radar for OVERLAP_S, and only then silence the
                    radar. The PCM therefore never sees 0x21b stop -- it sees two
                    sources briefly, then one. A gap is the thing most likely to
                    make it drop ACC, and a gap is exactly what suppressing first
                    would create.
                    """
                    OVERLAP_S = float(os.environ.get("OP_ALPHA_OVERLAP_S", "0.30"))
                    while not stop_flag["v"]:
                        # Entry needs a LIVE acc_active, not a remembered one.
                        # Without rx_alive() a stale True left over from before a
                        # link fault would start a handover -- suppressing the
                        # radar on the strength of a reading we can no longer
                        # confirm, at a speed we can no longer see.
                        if not rx_alive() or not getattr(cs_can, "acc_active", False):
                            time.sleep(0.02)
                            continue

                        _v = float(getattr(cs_can, "v_ego", 0.0)) * 3.6
                        print("\n>>> HANDOVER: PCM reports acc_active=1 at %.1f kph. "
                              "Opening our 0x21b/0x21c..." % _v, flush=True)
                        longtx_gate["on"] = True
                        time.sleep(OVERLAP_S)

                        print(">>> HANDOVER: suppressing the radar (%.2f s overlap "
                              "sent)..." % OVERLAP_S, flush=True)
                        _n0 = can_census[0].get(0x21B, 0)
                        _o0 = tx_state.get("crz_info_tx", 0)
                        uds_sniff["on"] = True
                        _sup_failed = None
                        try:
                            CarInterface.init(CP, _uds_can_recv, _uds_can_send)
                        except Exception as _e:
                            _sup_failed = _e
                        finally:
                            uds_sniff["on"] = False

                        if _sup_failed is not None:
                            # ABORT, do not keep transmitting. The radar is still
                            # driving 0x21b/0x21c and we would be a second writer on
                            # both -- which is what put the yellow radar warning on
                            # the cluster on 2026-08-12 when this path raised.
                            # Closing the gate puts the car back to exactly stock.
                            longtx_gate["on"] = False
                            print(">>> HANDOVER ABORTED: suppression raised: %s\n"
                                  "    Our tx is CLOSED again -- the radar keeps "
                                  "0x21b/0x21c and the car is back to stock.\n"
                                  "    Nothing was taken over; this run proves "
                                  "nothing about the PCM." % _sup_failed, flush=True)
                            tx_state["handover_failed"] = True
                            return

                        # Did the RADAR actually stop? Census minus our own echo --
                        # the same subtraction the radar-return watchdog uses.
                        time.sleep(1.5)
                        _foreign = ((can_census[0].get(0x21B, 0) - _n0)
                                    - (tx_state.get("crz_info_tx", 0) - _o0))
                        if _foreign > 20:
                            # Same reasoning as the exception path: two writers on
                            # 0x21b is worse than not taking over at all, so stand
                            # down rather than report a success we did not get.
                            longtx_gate["on"] = False
                            print(">>> HANDOVER ABORTED: radar STILL SENDING (%d "
                                  "foreign 0x21b in 1.5 s).\n    Our tx is CLOSED "
                                  "again -- two writers on one address is worse than "
                                  "none.\n    The car is back to stock." % _foreign,
                                  flush=True)
                            tx_state["handover_failed"] = True
                            return
                        print(">>> HANDOVER COMPLETE: radar silent, we own "
                              "0x21b/0x21c with the PCM already in ACC.", flush=True)
                        tx_state["handover_done"] = True
                        tx_state["handovers"] = tx_state.get("handovers", 0) + 1

                        # --- drive under openpilot until ACC ends ---------------
                        #
                        # rx_alive() is the second exit, and it is not optional.
                        # acc_active is a decoded value with no expiry: when the
                        # bus feed died on 2026-08-13 it stayed True and this
                        # loop spun for 264 s, so the release below never ran and
                        # the radar could not be freed by anything the driver
                        # did. A stale True is not evidence ACC is on -- it is
                        # evidence we have stopped being told.
                        while (not stop_flag["v"] and rx_alive()
                               and getattr(cs_can, "acc_active", False)):
                            time.sleep(0.02)
                        if stop_flag["v"]:
                            return
                        if not rx_alive():
                            print("\n>>> RELEASE (RX DEAD): no bus-0 frame for "
                                  "%.1fs. acc_active=%s is STALE, not current.\n"
                                  "    Releasing the radar and re-arming."
                                  % (rx_live.age(),
                                     getattr(cs_can, "acc_active", None)), flush=True)

                        # --- RELEASE ------------------------------------------
                        # ACC has ended (brake, cancel, MAIN off). Two facts make
                        # releasing the radar mandatory rather than optional:
                        #
                        #  1. Only the RADAR can perform the ACC entry handshake.
                        #     Our frames cannot -- that is the wall this whole mode
                        #     exists to route around. Holding it suppressed means
                        #     ONE engagement per run and no way back.
                        #  2. A suppressed radar with ACC off is pure cost: no
                        #     FCW/AEB/SBS and a dash warning, with nothing using
                        #     the suppression. OBSERVED 2026-08-12: exactly this,
                        #     right after the first brake.
                        #
                        # Release is just dropping tester-present and letting the
                        # session lapse. FINDINGS is explicit that this is the
                        # clean exit and that explicitly requesting the default
                        # session makes this radar fault.
                        # KEEP TRANSMITTING. Only tester-present stops here.
                        #
                        # Closing the gate at the same time leaves NOBODY sending
                        # 0x21b/0x21c for the ~6 s the radar's session takes to
                        # lapse -- a total gap in the PCM's longitudinal input.
                        # OBSERVED ON THE CAR 2026-08-12: after that gap the PCM
                        # latched an MRCC fault and MAIN could not be switched on
                        # again at all, which is far worse than the transient
                        # warning it looks like.
                        #
                        # The entry handover already knows this: it opens our tx
                        # FIRST and overlaps, precisely so the PCM never sees its
                        # input stop. The exit has to be symmetric -- we hold the
                        # frames until the radar is demonstrably back, and only
                        # then stand down. What we send meanwhile is the radar's
                        # own idle frame (long_active is False), so the PCM sees a
                        # continuous, plausible stream throughout: us, then briefly
                        # both, then the radar.
                        radar_hold["on"] = False
                        tx_state["handover_done"] = False   # re-arm the watchdog hold
                        print("\n>>> RELEASE: ACC ended. Tester-present STOPPED; our "
                              "idle frames KEEP FLOWING\n    so the PCM never loses "
                              "its longitudinal input. Waiting for the radar...",
                              flush=True)

                        # WATCH 0x361, NOT 0x21b. Two nodes cannot transmit the same
                        # CAN ID: while our tx is open the radar's 0x21b/0x21c lose
                        # arbitration and never appear, so waiting for 0x21b here
                        # waits for something we are ourselves blocking.
                        #
                        # MEASURED ON THE CAR 2026-08-12, this exact bug: the radar
                        # came back at +5 s and resumed 0x361-0x499 at full 10 Hz,
                        # while 21b/21c stayed pinned at 0 for the whole 20 s
                        # timeout. The detector then gave up and killed the loop.
                        #
                        # 0x361-0x499 are the radar's OTHER frames, and engage-first
                        # disables the shadow replay, so we never send them -- they
                        # are an uncontaminated "the radar is alive again" signal.
                        # Once seen we close our gate and the radar picks up
                        # 0x21b/0x21c within a frame or two, so the gap is ~20 ms
                        # instead of the 6 s that latched the PCM.
                        _t_rel = time.time()
                        while not stop_flag["v"] and time.time() - _t_rel < 20.0:
                            _c0 = can_census[0].get(0x361, 0)
                            time.sleep(1.0)
                            _foreign_back = can_census[0].get(0x361, 0) - _c0
                            if _foreign_back > 3:      # nominal 10 Hz
                                # Radar is back and both of us are on the wire. NOW
                                # stand down -- the same overlap-not-gap ordering the
                                # entry handover uses, run in reverse.
                                longtx_gate["on"] = False
                                print(">>> RELEASE COMPLETE: radar alive again after "
                                      "%.1f s (0x361 at %d Hz).\n"
                                      "    Our tx now CLOSED so it can take back "
                                      "0x21b/0x21c. AEB restored.\n"
                                      "    Press SET again to re-engage."
                                      % (time.time() - _t_rel, _foreign_back), flush=True)
                                break
                        else:
                            if not stop_flag["v"]:
                                # Do NOT keep transmitting indefinitely with no radar
                                # and no ACC: stand down and say so.
                                longtx_gate["on"] = False
                                print(">>> RELEASE: radar did NOT return within 20 s. "
                                      "Our tx CLOSED anyway.\n    Longitudinal cannot "
                                      "re-arm; steering is unaffected. Restart to "
                                      "recover.", flush=True)
                                return
                        if stop_flag["v"]:
                            return
                        radar_hold["on"] = True     # ready to hold the NEXT session
                        # ...and loop back to waiting for the next acc_active.

                threading.Thread(target=takeover_thread, daemon=True).start()
            # Put the radar's OTHER seven frames back. Started only
            # alongside alpha long, because the panda only accepts them
            # under mazda_longitudinal -- outside it they would be
            # rejected, which is the right failure direction.
            # SUPERSEDED BY THE CARCONTROLLER'S RADAR HEARTBEAT.
            #
            # longitudinal.py (yummydirtx @ go-no-malfunction) now emits these seven
            # frames itself via create_radar_heartbeat_messages, at
            # RADAR_HEARTBEAT_STEP, to BOTH bus 0 and bus 2 -- and bus 2 is the half
            # that matters, because the forward camera lives there and our replay
            # never reached it. Running this thread as well would put a SECOND copy
            # of 0x361-0x366/0x499 on bus 0, from a different counter sequence,
            # which is worse than the absence it was written to cover.
            #
            # It also replays a CAPTURED snapshot, so its lead data is frozen at
            # whatever the radar last saw; the heartbeat uses fixed empty-target
            # templates, which is a coherent "no target" instead.
            #
            # OP_ALPHA_LOCAL_SHADOW=1 brings it back for A/B against the heartbeat.
            _local_shadow = os.environ.get("OP_ALPHA_LOCAL_SHADOW") == "1"
            if _no_shadow or not _local_shadow:
                print("radar shadow: using the carcontroller's heartbeat "
                      "(bus 0 + bus 2). The local bus-0-only replay is OFF -- set "
                      "OP_ALPHA_LOCAL_SHADOW=1 to use it instead.")
            else:
                threading.Thread(target=radar_shadow_thread, daemon=True).start()
            print(">>> ALPHA LONG ACTIVE: 0x21b/0x21c at 50 Hz, radar tester-present "
                  "at 2 Hz.\n    FCW / AEB / SBS ARE OFF while this runs.\n")
        elif alpha_long:
            # A PREVIOUS RUN MAY HAVE LEFT THE RADAR MUTE. Restore it before
            # giving up, or this is the worst state the car can be in.
            #
            # OBSERVED 2026-08-09: a run started with --alpha-long while the car
            # was briefly asleep came back INCONCLUSIVE, so neither the alpha-long
            # tx nor the radar-shadow replay started. But the run BEFORE it had
            # genuinely suppressed the radar, and the radar recovers slowly --
            # still mute 45 s after that process was killed. Net result: radar
            # silent, nobody replaying its frames, and the forward camera saw its
            # partner vanish with no substitute. The front camera fault came
            # straight back, and this branch's "stock radar cruise is INTACT"
            # message was simply wrong.
            #
            # So do not assume a failed suppression means an untouched radar. Send
            # the documented inverse and say what happened. Safe if the radar was
            # never suppressed: 10 01 / 28 00 01 / 85 01 into an already-default
            # session is a no-op, and any of them may be NAKed without harm.
            try:
                with panda_lock:
                    for _p in ([0x02, 0x10, 0x01, 0, 0, 0, 0, 0],
                               [0x03, 0x28, 0x00, 0x01, 0, 0, 0, 0],
                               [0x02, 0x85, 0x01, 0, 0, 0, 0, 0]):
                        panda.can_send(_RADAR_ADDR, bytes(_p), 0)
                        time.sleep(0.05)
                print("    (sent radar restore 10 01 / 28 00 01 / 85 01 in case an "
                      "earlier run left it muted)")
            except Exception as _e:
                print("    (radar restore failed: %s -- if the dash shows a camera "
                      "fault, power-cycle)" % _e)
            # Hard refusal, not a warning. Transmitting 0x21b while the radar is
            # still transmitting it puts two writers with independent counters on
            # one address and the PCM acts on whichever it sees -- worse than no
            # longitudinal at all. Steering is unaffected and stays armed; the
            # car keeps stock MRCC, which is the safe failure direction.
            print("!!! ALPHA LONG DISABLED: radar was not suppressed, so nothing "
                  "will be sent on 0x21b/0x21c.\n"
                  "    Stock radar cruise is INTACT. Steering is still armed.\n"
                  "    Check that the panda runs the 19-ID mazda_filter.h build "
                  "(0x76C must reach the host).\n")
    else:
        # NOOUTPUT, not SILENT -- every control_write above sets NOOUTPUT. The old
        # message said SILENT and was simply wrong, which matters because the two
        # differ in the one way that bites this build: SILENT cannot ACK, and with
        # no harness relay that kills the forward camera. Reading "SAFETY_SILENT"
        # here while hunting a camera fault points at exactly the wrong suspect.
        print("panda: SAFETY_NOOUTPUT -- dashcam mode, nothing transmitted "
              "(cam<->car forwarding and ACK stay alive).\n")

    # Per-service publish periods taken from the REAL openpilot service
    # frequencies, so each topic goes out at its nominal rate and selfdrived's
    # average-frequency check passes. On-demand (freq 0) topics get a mild 20 Hz
    # cap so they don't flood. The loop is paced at 100 Hz below.
    # carOutput is selfdrived's fastest polled input from us. selfdrived's loop
    # has no sleep -- it is paced purely by message arrival -- and flags
    # selfdrivedLagging if its average loop is slower than ~90 Hz. Publishing
    # carOutput a touch hot (~110 Hz, still inside the 100 Hz [40,120] band) wakes
    # that loop >90 Hz. The loop below runs at 250 Hz so every service's actual
    # rate lands cleanly on its nominal (a slow loop aliases the rates).
    HOT = {"carOutput": 1.0 / 110.0}
    def _period(name):
        if name in HOT:
            return HOT[name]
        fr = SERVICE_LIST[name].frequency if name in SERVICE_LIST else 20.0
        return (1.0 / fr) if fr > 1e-3 else 0.05
    pub_period = {name: _period(name) for name in avail}
    next_pub = {name: 0.0 for name in avail}
    LOOP_DT = 0.004                # 250 Hz publish loop -> accurate per-service rates
    loop_next = time.time()
    last_state = 0.0

    try:
        while True:
            now = time.time()
            dt = now - last
            last = now

            # inference runs in model_thread; read its latest output
            out = mstate["out"]
            curv = mstate["curv"]
            frames = mstate["frames"]

            if cs_can.v_ego > 0.5:
                moving += dt

            for name in main_pub:                       # carState handled in its own thread
                if now < next_pub[name]:
                    continue
                next_pub[name] = now + pub_period[name]
                m = messaging.new_message(name, 0) if name in LIST_TOPICS else messaging.new_message(name)
                if name == 'deviceState':
                    m.deviceState.started = True
                    # Without this selfdrived reads the default 0 and raises
                    # outOfSpace (< 7%), which is nonsense here -- the disk is 28%
                    # used with 629GB free. Sampled once at startup, not per frame.
                    m.deviceState.freeSpacePercent = free_pct
                elif name == 'pandaStates':
                    # Real values from the panda's own health packet. Publishing an
                    # EMPTY pandaStates is what made selfdrived raise usbError -- it
                    # checks sm.valid['pandaStates'], so the message has to actually
                    # carry state. This one is genuinely fixed, not suppressed.
                    hv = health_state["v"]
                    m = messaging.new_message('pandaStates', 1)
                    ps = m.pandaStates[0]
                    ps.pandaType = log.PandaState.PandaType.dos
                    if hv is not None:
                        ps.uptime = int(hv[0])
                        ps.ignitionLine = bool(hv[8])
                        ps.ignitionCan = bool(hv[9])
                        ps.controlsAllowed = bool(hv[10])
                        ps.safetyModel = 'mazda'
                        # hv[13] is struct health_t.safety_param -- what the panda
                        # ACTUALLY holds, not what we asked for, so this still
                        # verifies the mode write landed. Leaving it unset was
                        # harmless while the param was always 0; under --alpha-long
                        # CP.safetyConfigs[0].safetyParam is 1, and selfdrived
                        # compares the two (selfdrived.py:313). A capnp-default 0
                        # here reads as a safety mismatch and raises a BLOCKING
                        # controlsMismatch after 10 s, so openpilot never engages.
                        ps.safetyParam = int(hv[13])
                        ps.rxBufferOverflow = int(hv[6])
                        ps.txBufferOverflow = int(hv[5])
                        ps.heartbeatLost = bool(hv[16])
                        ps.powerSaveEnabled = bool(hv[15])
                elif name == 'liveParameters':
                    lp = m.liveParameters
                    # ANGLE OFFSET. LatControlTorque's measurement is
                    #   measured_curvature = -calc_curvature(steeringAngleDeg - angleOffsetDeg, ...)
                    #   error = setpoint - measurement
                    # and this field was NEVER PUBLISHED, so it defaulted to 0.
                    # Upstream paramsd learns it online; there is no paramsd here.
                    # A steering wheel is essentially never perfectly centred at
                    # zero, so a fixed offset of a degree or two enters the loop as
                    # a CONSTANT error the integrator can never satisfy -- it winds
                    # up, overshoots, unwinds, and repeats. On a straight road that
                    # is indistinguishable from jitter, and no gain change fixes it
                    # because the setpoint itself is biased.
                    #
                    # Learn it the way paramsd effectively does: the mean steering
                    # angle while genuinely tracking straight IS the offset. Gated
                    # hard so only unambiguous samples contribute --
                    #   - above 15 m/s (54 kph): geometry is well conditioned
                    #   - commanded curvature ~ 0: we are asking for straight
                    #   - driver not steering: their input is not our offset
                    #   - |angle| < 10 deg: reject anything that is plainly a turn
                    # The 0.0005 gain gives a ~2000-sample time constant (~20 s at
                    # 100 Hz of qualifying data), slow enough that a long curve
                    # cannot drag it, and it is clamped to +-5 deg so a bad streak
                    # can never inject a large bias.
                    try:
                        if (cs_can.v_ego > 15.0
                                and abs(float(tx_state.get("curvature", 0.0))) < 3e-4
                                and not cs_can.steering_pressed
                                and abs(cs_can.steering_angle) < 10.0):
                            if ang_off["n"] == 0:
                                ang_off["v"] = float(cs_can.steering_angle)
                            else:
                                ang_off["v"] += 0.0005 * (float(cs_can.steering_angle) - ang_off["v"])
                            ang_off["n"] += 1
                            ang_off["v"] = max(-5.0, min(5.0, ang_off["v"]))
                    except Exception:
                        pass
                    lp.angleOffsetDeg = float(ang_off["v"])
                    lp.angleOffsetAverageDeg = float(ang_off["v"])
                    lp.steerRatio = 15.5; lp.stiffnessFactor = 1.0; lp.valid = True
                elif name == 'liveDelay':
                    # This topic was in the publish list but had NO branch, so it
                    # went out as an empty message with lateralDelay = 0.0.
                    # controlsd.py:128 does
                    #     lat_delay = liveDelay.lateralDelay + LAT_SMOOTH_SECONDS
                    # and LatControlTorque turns that into
                    #     delay_frames = clip(lat_delay/dt + 1, 1, buffer_len)
                    #     setpoint     = lat_accel_request_buffer[-delay_frames]
                    # At 0.0 that is delay_frames = 1, i.e. the setpoint is the
                    # CURRENT request compared against a measurement that
                    # physically lags it by the actuator delay. The error is
                    # inflated on every transient, so the loop overshoots and
                    # hunts -- it does not run slow, it runs unsettled, and the
                    # torque tune was being fitted on top of that.
                    #
                    # 0.1 s is CP.steerActuatorDelay for MAZDA_CX5_2022 (lagd's
                    # initial_lag). Upstream lagd LEARNS this online; there is no
                    # lagd on this board, so publish the static value rather than
                    # leave it at a number that is definitely wrong.
                    # Same OP_STEER_DELAY the model uses for LAT_ACTION_T, so the
                    # controller and the model compensate the SAME lag. These were
                    # 0.200 here and 0.275 in op_stream -- two answers for one
                    # physical delay, both short of the ~0.365 s the 1.37 Hz limit
                    # cycle implies. Falls back to CP if the var is unset.
                    # Same STEER_DELAY the model uses for LAT_ACTION_T, so the
                    # controller and the model compensate the SAME lag.
                    m.liveDelay.lateralDelay = float(STEER_DELAY)
                    # Status enum is unestimated/estimated/invalid -- there is no
                    # "calibrated". 'estimated' is the honest label: this is a
                    # static value from CarParams, not something we measured, but
                    # it is a valid estimate rather than an absent one.
                    m.liveDelay.status = log.LiveDelayData.Status.estimated
                    m.liveDelay.validBlocks = 10
                elif name == 'liveTorqueParameters':
                    tp = m.liveTorqueParameters
                    # These came from the MAZDA_CX9_2021 row of opendbc
                    # torque_data/params.toml, which CX5_2022 substitutes to.
                    # They were previously 2.5 / 0.1, both wrong in the same
                    # direction: feedforward is lat_accel / latAccelFactor, so
                    # an inflated factor UNDER-commands torque (~30% low) and
                    # the PI integral has to catch up -- the car enters a curve
                    # lazily, then corrects. Low friction compounds it by
                    # under-compensating steering breakaway.
                    #
                    # RESCALED for STEER_MAX 1000 -> 1400 (2026-07-30).
                    # latAccelFactor is m/s^2 per unit NORMALISED torque, and the
                    # carcontroller turns normalised into counts by multiplying by
                    # STEER_MAX. So 1.0 used to mean 1000 counts and now means
                    # 1400: the same factor would make the feedforward command
                    # 1.4x the torque for the same requested lateral accel.
                    #   1.7601682915983443 * 1400/1000 = 2.464235608
                    # Note this was ALSO never rescaled for the earlier 800 -> 1000,
                    # so before this change the feedforward had been over-commanding
                    # by ~75% cumulative -- which is part of why the measured turn
                    # angles looked so strong. Expect angles to drop; that is the
                    # honest baseline to tune up from, not a regression.
                    # STEER_MAX 2047 was tried and reverted on 2026-08-01, so this
                    # goes back with it. The pair always moves together: counts =
                    # output_lataccel * STEER_MAX / latAccelFactor, and scaling
                    # both left that ratio at 568.13 either way -- which is
                    # precisely why raising the ceiling changed nothing except
                    # where the limit cycle stopped being clipped.
                    # RESCALED for STEER_MAX 1400 -> 2047 (2026-08-01). The pair
                    # always moves together: below the rail counts = lataccel *
                    # STEER_MAX / latAccelFactor is unchanged at 568/m/s^2, which
                    # is the point -- the feedforward stays calibrated. What the
                    # pair DOES change is the ceiling, since steer_max is 1.0
                    # normalised and update_limits() clamps the PID to
                    # latAccelFactor: 1400 counts becomes 2047.
                    tp.useParams = True
                    tp.latAccelFactorFiltered = 1.7601682915983443 * 2047 / 1000
                    tp.latAccelOffsetFiltered = 0.0
                    tp.frictionCoefficientFiltered = 0.17713792194297195
                elif name == 'liveCalibration':
                    # Report the REAL calibration state. This used to be hardcoded
                    # to `calibrated`, which quietly disabled openpilot's own entry
                    # gate: selfdrived reads exactly this field and raises
                    # calibrationIncomplete / calibrationInvalid, both NO_ENTRY.
                    # With calStatus pinned to calibrated and the calibrator never
                    # having converged (valid_blocks 0, rpy 0,0,0), engaging cruise
                    # would have taken openpilot to lat_active and had it steer to a
                    # target derived from an unlearned mount angle. The panda's
                    # torque limits bound the RATE of that, not its correctness.
                    #
                    # Calibrator.status is already exactly openpilot's vocabulary
                    # ("uncalibrated" / "calibrated" / "invalid"), so just map it.
                    cal_obj = getattr(runner, "calibrator", None)
                    st = getattr(cal_obj, "status", "uncalibrated") if cal_obj else "uncalibrated"
                    rpy = list(getattr(runner, "calib_euler", (0.0, 0.0, 0.0)))
                    m.liveCalibration.rpyCalib = [float(x) for x in rpy]
                    m.liveCalibration.calStatus = {
                        "calibrated": log.LiveCalibrationData.Status.calibrated,
                        "invalid": log.LiveCalibrationData.Status.invalid,
                    }.get(st, log.LiveCalibrationData.Status.uncalibrated)
                elif name == 'modelV2':
                    md = m.modelV2
                    md.frameId = frames
                    pxyz = out.get("path_xyz")
                    if pxyz is not None:
                        md.position.x = [float(pxyz[i][0]) for i in range(33)]
                        md.position.y = [float(pxyz[i][1]) for i in range(33)]
                        md.position.z = [float(pxyz[i][2]) for i in range(33)]
                        md.position.t = [float(t) for t in T_IDXS]
                    try:
                        md.action.desiredCurvature = float(curv)
                        # The other two fields modeld puts on this message. Nothing
                        # in THIS port acts on them -- there is no plannerd and no
                        # longitudinal actuation, the panda only sees 0x243 steering
                        # -- but they are what the model is now actually predicting
                        # rather than a value derived from its plan, and publishing
                        # them is what makes modelV2 here mean what it means on a
                        # comma three. Read them off /status.json to see what the
                        # model would do about speed if anything were listening.
                        _act = mstate.get("action")
                        if _act is not None:
                            md.action.desiredAcceleration = float(_act["desiredAcceleration"])
                            md.action.shouldStop = bool(_act["shouldStop"])
                    except Exception: pass
                    # Lane-change state for the UI and for anything downstream
                    # that reads it. desireState is the model's own distribution
                    # over the 8 Desires; laneChangeState/Direction are
                    # DesireHelper's. Published together so a disagreement
                    # between what was asked and what the model is doing is
                    # visible rather than inferred.
                    try:
                        ds = mstate.get("desire_state")
                        if ds is not None:
                            md.meta.desireState = [float(x) for x in ds]
                        md.meta.laneChangeState = mstate.get("lc_state", 0)
                        md.meta.laneChangeDirection = mstate.get("lc_dir", 0)
                    except Exception: pass
                elif name == 'lateralManeuverPlan':
                    # controlsd PREFERS lateralManeuverPlan.desiredCurvature over
                    # model_v2.action.desiredCurvature whenever this msg is valid.
                    # We publish it valid, so an empty (0) desiredCurvature here was
                    # overriding the model's real curvature -> torque stuck at 0 even
                    # while engaged. Feed the model's curvature through so controlsd
                    # actually steers.
                    m.lateralManeuverPlan.desiredCurvature = float(curv)
                elif name == 'longitudinalPlan':
                    # ALPHA LONG. Same story as lateralManeuverPlan above: this topic
                    # was published empty, so every field was a capnp default. That is
                    # harmless while nothing actuates longitudinally and actively wrong
                    # the moment something does.
                    #
                    # controlsd reads exactly three fields off this message
                    # (controlsd.py:92-119, :163, :169):
                    #     aTarget    -> LoC.update() -> actuators.accel
                    #     shouldStop -> LoC.update(), and cruiseControl.resume
                    #     hasLead    -> hudControl.leadVisible -> the carcontroller's
                    #                   CRZ_CTRL follow/cruise profile choice
                    # so those three are all it takes to drive the real long path.
                    #
                    # There is no plannerd on this board and no longitudinal MPC. What
                    # fills them instead is the MODEL'S OWN ACTION HEAD -- the same
                    # trained output that now produces desiredCurvature. That is not a
                    # substitute for the planner, it IS openpilot's e2e longitudinal
                    # path: in Experimental mode longitudinal_planner.py takes
                    # modelV2.action.desiredAcceleration as a lower bound on the MPC,
                    # and with no MPC present it is simply the whole signal.
                    #
                    # WHAT THAT MEANS IN PRACTICE: there is no lead-distance MPC, no
                    # jerk limiting, no speed-limit or curvature-based slowdown, and no
                    # cruise-speed tracking. The car accelerates and brakes on what the
                    # network predicts a human would do from this camera frame, clipped
                    # only by longitudinal.py's accel scale and the panda's +-2000
                    # window. Treat it as such.
                    _lact = mstate.get("action")
                    if _lact is not None:
                        # govern_accel() is the planner this board does not have:
                        # accel ceiling, jerk limit and speed cap. NOT following
                        # distance -- the model owns that (govern_accel step 1).
                        # Applied HERE rather than in the model so the raw action
                        # head stays visible in desired_accel for comparison.
                        _now_g = time.time()
                        _dt_g = _now_g - _gov["t"] if _gov["t"] else 0.05
                        _gov["t"] = _now_g
                        _a_raw = float(_lact["desiredAcceleration"])
                        _ld = mstate.get("lead_d")
                        _lp = mstate.get("lead_p")

                        # ASYMMETRIC LEAD-PROBABILITY FILTER -- rises instantly,
                        # decays slowly. Straight out of openpilot's radard.py:238,
                        # whose comment is "keep lead when uncertain":
                        #
                        #     if lead_prob > filt.x: filt.x = lead_prob   # instant
                        #     else:                  filt.update(lead_prob)
                        #
                        # WHY IT MATTERS HERE. This port tested the RAW per-frame
                        # probability against a hard 0.5. MEASURED 2026-08-12, this
                        # car's lead_p has a median of 0.023 and spikes to 0.995 --
                        # so the lead appeared and vanished frame to frame, and the
                        # MPC alternated between "lead ahead" and "clear road" (with
                        # process_lead fabricating a fast lead 50 m out whenever it
                        # read absent). Brake, release, brake, release: the
                        # brake-checking reported from the car.
                        #
                        # Rising instantly keeps the response to a real lead sharp;
                        # decaying over ~0.2 s stops a single dropped frame throwing
                        # the plan away. Same constants as upstream.
                        if _lp is not None:
                            _lpf = _gov.get("lead_p_filt", 0.0)
                            if _lp > _lpf:
                                _lpf = float(_lp)                 # rise: instant
                            else:                                  # decay: filtered
                                _alpha = _dt_g / (0.2 + _dt_g)
                                _lpf = _lpf + _alpha * (float(_lp) - _lpf)
                            _gov["lead_p_filt"] = _lpf
                            _lp = _lpf
                        # MPC first (if enabled): it PLANS the accel profile.
                        # govern_accel then runs on its output as a safety clamp.
                        if _mpc is not None:
                            try:
                                _vego = float(cs_can.v_ego)
                                _lead = _MpcLead(_ld, mstate.get("lead_v"),
                                                 mstate.get("lead_a"), _lp, _vego)
                                # FOLLOW DISTANCE. personality picks T_FOLLOW in
                                # long_mpc.py:73 -- aggressive 1.25 s, standard
                                # 1.45 s, relaxed 1.75 s -- and the MPC's desired
                                # gap is T_FOLLOW * v_ego + STOP_DISTANCE.
                                #
                                # This was hardcoded to 0, and log.capnp:141 makes
                                # 0 = AGGRESSIVE: the tightest gap openpilot offers,
                                # chosen by accident rather than intent. That is the
                                # "distance from the lead feels hardcoded" reported
                                # from the car -- it was, and to the closest setting.
                                # Default to standard, overridable.
                                _mpc.set_weights(prev_accel_constraint=True, personality=_PERSONALITY)
                                _mpc.set_cur_state(_mpc_state["v"], _mpc_state["a"])
                                # CHASE THE DRIVER'S SET SPEED, NOT A CONFIG CONSTANT.
                                #
                                # v_cruise was OP_V_TARGET_KPH or V_MAX_KPH -- 116 kph
                                # by default -- so at any normal road speed the MPC is
                                # asked to reach something far above the car and pins
                                # at the accel cap. MEASURED 2026-08-11 in coexist
                                # mode, the first configuration where the PCM actually
                                # obeyed us: at 16 kph the model held ma=+1.78,
                                # a_cmd=+1.20, "lim=accel cap" continuously, and the
                                # car accelerated hard. Under suppression this was
                                # invisible because the PCM ignored the frames.
                                #
                                # cs_can.set_speed_* comes from CRZ_EVENTS 0x21F
                                # CRZ_SPEED. Its DBC unit is ambiguous (0.005/-0.5 over
                                # 16 bits reads as either kph or m/s), so ACCEPT IT
                                # ONLY IF EXACTLY ONE INTERPRETATION IS PLAUSIBLE.
                                # Ambiguity here means commanding to the wrong scale on
                                # a moving car.
                                #
                                # With no trustworthy set speed, target the CURRENT
                                # speed: the MPC then holds rather than accelerates.
                                # That is the safe failure -- openpilot does nothing
                                # instead of everything.
                                _vt = None
                                _raw = float(getattr(cs_can, "set_speed_kph", -1.0))
                                _cands = [v for v in (_raw, _raw * 3.6) if 20.0 <= v <= 160.0]
                                if len(_cands) == 1:
                                    _vt = _cands[0] / 3.6
                                _env_t = os.environ.get("OP_V_TARGET_KPH")
                                if _env_t:
                                    _vt = float(_env_t) / 3.6
                                # SPEED-LIMIT TARGET (OP_SPEED_LIMIT=1).
                                #
                                # Runs AFTER the set-speed logic above and takes the
                                # LOWER of the two whenever both are known, so the
                                # driver can always ask for less than the limit but
                                # never for more by leaving a stale set speed behind.
                                # OP_V_TARGET_KPH still wins outright -- it is the
                                # explicit manual override and is what the bench
                                # tests set.
                                #
                                # STANDARD aims at the limit + OP_STD_OVER_MPH; MADS
                                # at the limit + OP_MADS_OVER_MPH. The mode comes from the
                                # --mads flag, so one switch changes both how
                                # openpilot engages and what it aims for.
                                if _speed_target is not None and not _env_t:
                                    try:
                                        # v_ego lets SpeedTarget reject an INFERRED
                                        # limit far below our actual speed -- a
                                        # side-road match, not a real limit.
                                        _lim_kph = _speed_target.update(
                                            v_ego_kph=float(cs_can.v_ego) * 3.6)
                                        _lim_mps = _lim_kph / 3.6
                                        # Publish for the HUD. --display serves no
                                        # web page, so without this the posted limit
                                        # the car is aiming at is invisible in the
                                        # one mode you actually watch while driving.
                                        _sts = _speed_target.status()
                                        STATE["speed_limit_kph"] = _sts["limit_kph"]
                                        STATE["speed_limit_inferred"] = bool(_sts["inferred"])
                                        STATE["speed_target_kph"] = _sts["target_kph"]
                                        STATE["speed_limit_mode"] = _sts["mode"]
                                        # _vt is None when no set speed was
                                        # trustworthy; then the limit IS the target
                                        # rather than the current-speed hold.
                                        _vt = _lim_mps if _vt is None else min(_vt, _lim_mps)
                                        # COAST BAND, MPC SIDE. DORMANT: the MPC is
                                        # off (OP_MPC is unset everywhere in this
                                        # tree) and the car is driven by the action
                                        # head, so govern_accel step 2b is the band
                                        # that actually runs. This exists so the two
                                        # paths cannot disagree if the flag is ever
                                        # flipped -- without it, turning the MPC on
                                        # would silently restore the brake-checking.
                                        #
                                        # govern_accel's band (step 2b) only clamps
                                        # the OUTPUT, and the MPC is upstream of it:
                                        # handed a target the car is already above,
                                        # the MPC plans a deceleration back to it and
                                        # that braking passes straight through the
                                        # band, because govern_accel cannot tell
                                        # cap-braking apart from a lead or a curve.
                                        # The band would work for the e2e path and do
                                        # nothing for the MPC one.
                                        #
                                        # So do not show the MPC an overshoot it is
                                        # meant to ignore. Inside the band, target the
                                        # CURRENT speed -- the MPC then holds instead
                                        # of braking, and cannot accelerate either
                                        # because the target is where the car already
                                        # is. Past the band the target snaps back and
                                        # the MPC brings the car down to it.
                                        # Only when the LIMIT is what is binding. If a
                                        # driver set speed is lower, that is a request,
                                        # not a cap, and stock cruise holds it -- the
                                        # band must not quietly raise it. Never past
                                        # V_MAX_KPH either, matching govern_accel.
                                        _vego_mps = float(cs_can.v_ego)
                                        _brake_mps = min(_lim_mps + V_COAST_BAND_KPH / 3.6,
                                                         V_MAX_KPH / 3.6)
                                        if _vt >= _lim_mps and _lim_mps < _vego_mps <= _brake_mps:
                                            _vt = _vego_mps
                                    except Exception as _ste:
                                        # Never let a lookup fault the control loop:
                                        # fall through to whatever _vt already was.
                                        # time.time() rather than the loop's `now`
                                        # on purpose -- a NameError raised from an
                                        # exception handler would take out the
                                        # thread this exists to protect.
                                        if int(time.time()) % 10 == 0:
                                            print("SPEEDLIM update failed:", _ste, flush=True)
                                if _vt is None:
                                    _vt = _vego
                                _mpc_state["v_cruise"] = _vt
                                _mpc.update(_MpcRadar(_lead), _vt, personality=_PERSONALITY)
                                _a_traj = np.interp(_CTRL_T, _T_IDXS_MPC, _mpc.a_solution)
                                _v_traj = np.interp(_CTRL_T, _T_IDXS_MPC, _mpc.v_solution)
                                # advance the planner's own state 1 tick, exactly
                                # as longitudinal_planner does, so the horizon is
                                # continuous instead of restarting each cycle
                                _a_prev_mpc = _mpc_state["a"]
                                _mpc_state["a"] = float(np.interp(_dt_g, _CTRL_T, _a_traj))
                                _mpc_state["v"] = max(0.0, _mpc_state["v"]
                                                      + _dt_g * (_mpc_state["a"] + _a_prev_mpc) / 2.0)
                                # pull v back toward reality so it cannot diverge
                                _mpc_state["v"] += 0.15 * (_vego - _mpc_state["v"])
                                _a_mpc, _ss_mpc = get_accel_from_plan(
                                    _v_traj, _a_traj, _CTRL_T,
                                    action_t=_MPC_ACTION_T, vEgoStopping=0.5)

                                # BLEND WITH THE ACTION HEAD, as upstream does.
                                #
                                # longitudinal_planner.py:135-142 does NOT choose
                                # between the two -- in experimental mode it runs
                                # both and takes the more conservative:
                                #     output_a_target = min(a_e2e, a_mpc)
                                #     should_stop     = stop_e2e or stop_mpc
                                #
                                # This port used to overwrite _a_raw with the MPC's
                                # value and drop the action head entirely, which
                                # threw away the only component that has ever seen a
                                # stopped car. The MPC's whole view of the road is
                                # one lead object, and MEASURED 2026-08-12 that lead
                                # never arrives: lead_p peaks at 0.344 against a 0.5
                                # threshold, so process_lead() fabricates a clear
                                # road on every frame. An optimiser tracking a speed
                                # target with no obstacle input is exactly as
                                # unnatural as it sounds, and it is what "panics near
                                # stationary leads" and "does not feel natural"
                                # were describing.
                                #
                                # min() keeps the MPC's smooth speed-keeping and jerk
                                # shaping while letting the model brake HARDER
                                # whenever it recognises something the MPC cannot
                                # see. The model can never make the car accelerate
                                # more than the MPC planned -- only less.
                                _a_e2e = _a_raw
                                _ss_e2e = bool(_lact.get("shouldStop"))
                                _a_raw = min(_a_e2e, _a_mpc)
                                _lact["shouldStop"] = bool(_ss_mpc or _ss_e2e)
                                _gov["a_mpc"] = _a_mpc
                                _gov["a_e2e"] = _a_e2e
                                # Which one is binding, so the log says who is driving.
                                _gov["src"] = "e2e" if _a_e2e < _a_mpc else "mpc"
                                _gov["mpc"] = 1
                            except Exception as _e:
                                _gov["mpc"] = 0
                                _gov["mpc_err"] = "%s" % type(_e).__name__
                        # Live speed cap for the e2e path. With the MPC on, the
                        # target is already folded into _vt and the MPC plans to
                        # it; govern_accel then re-applies it as a backstop, which
                        # is harmless. With the MPC off this is the ONLY thing
                        # that makes the posted limit matter.
                        _cap_kph = None
                        if _speed_target is not None:
                            try:
                                _cap_kph = _speed_target.update(
                                    v_ego_kph=float(cs_can.v_ego) * 3.6)
                                _sts = _speed_target.status()
                                STATE["speed_limit_kph"] = _sts["limit_kph"]
                                STATE["speed_limit_inferred"] = bool(_sts["inferred"])
                                STATE["speed_target_kph"] = _sts["target_kph"]
                                STATE["speed_limit_mode"] = _sts["mode"]
                            except Exception:
                                _cap_kph = None
                        _a_cmd, _why = govern_accel(_a_raw, float(cs_can.v_ego),
                                                    _ld, _lp, _gov["a"], _dt_g,
                                                    v_max_kph=_cap_kph)
                        _gov["a"] = _a_cmd
                        _gov["why"] = _why
                        _gov["raw"] = _a_raw
                        m.longitudinalPlan.aTarget = _a_cmd
                        m.longitudinalPlan.shouldStop = bool(_lact["shouldStop"])
                    else:
                        # No action yet (no v_ego, or model not running). Command zero
                        # accel rather than the capnp default and let shouldStop hold.
                        m.longitudinalPlan.aTarget = 0.0
                        m.longitudinalPlan.shouldStop = True
                    # lead_prob[0] is the model's own probability that there is a lead
                    # at t=0. This only selects which CRZ_CTRL profile is sent; it is
                    # not a distance and nothing brakes for it on its own.
                    _lp = out.get("lead_prob")
                    m.longitudinalPlan.hasLead = bool(_lp is not None and float(_lp[0]) > 0.5)
                m.valid = True
                pm.send(name, m)

            sm.update(0)
            ss, cc = sm['selfdriveState'], sm['carControl']

            # In the model role nothing local decodes the bus, so refresh the
            # cs_can shim from the carState the panda role publishes. Done right
            # after sm.update so every consumer below -- the model's v_ego, the
            # angle-offset learner, the desire helper, the display -- sees the
            # same snapshot within one loop, exactly as when cs_can was written
            # in-process by can_thread.
            if role == "model" and sm.updated.get('carState'):
                cs_can.update_from(sm['carState'])

            # hand controlsd's latest decision to the tx thread (every loop).
            # actuators.curvature is the POST-clip_curvature desired curvature --
            # the one the lateral controller targets -- not the raw model output,
            # so the steering angle derived from it already respects the lateral
            # accel/jerk limits controlsd applies.
            tx_state["torque_norm"] = float(cc.actuators.torque)
            tx_state["curvature"] = float(cc.actuators.curvature)
            tx_state["lat_active"] = bool(cc.latActive)
            tx_state["op_enabled"] = bool(ss.enabled)

            # --- engagement edge log ----------------------------------------
            # 799 resyncs in one drive came from torque being killed at
            # engage/disengage boundaries: controls_allowed drops, our apply_last
            # is still ramping down at 25/msg, and every remaining non-zero frame
            # is a panda violation until it reaches 0. Inferring the cause from
            # rejection counts was guesswork, so record the actual edges.
            #
            # Checked on the FULL loop (250 Hz), not the 20 Hz STATE tick: a drop
            # and re-latch inside 50 ms is exactly the case that produces a
            # resync while looking like nothing happened in the trace.
            #
            # Printed to stdout rather than accumulated in STATE. STATE is dumped
            # to the jsonl every record, so carrying a 40-entry history there
            # would add ~6 KB to each of ~24 records/s -- 150x the cost of the
            # thing being measured. stdout is already tee'd to the run log.
            _ca = bool(health_state["v"][10]) if health_state["v"] else False
            _cur = (_ca, bool(cc.latActive), bool(ss.enabled))
            if edge["prev"] is None:
                edge["prev"] = _cur
            elif _cur != edge["prev"]:
                _p = edge["prev"]
                edge["prev"] = _cur
                edge["n"] += 1
                _what = " ".join(
                    ("+" if _cur[i] else "-") + nm
                    for i, nm in enumerate(("controls_allowed", "lat_active", "op_enabled"))
                    if _p[i] != _cur[i])
                edge["last"] = _what
                # Everything needed to attribute the edge without a second run:
                # the panda gate inputs (cruise bits), the EPS's own refusal
                # (lkas_block), what openpilot thought (op_state + blocking
                # events), and the torque left in flight when it happened.
                print("EDGE %8.2fs %-38s v=%5.1fkph crz=%d/%d lkas_block=%d "
                      "trq=%+5d op=%s ev=%s"
                      % (now - t0, _what, cs_can.v_ego * 3.6,
                         int(cs_can.cruise_available), int(cs_can.cruise_enabled),
                         int(cs_can.lkas_block), int(tx_state["applied"]),
                         str(ss.state),
                         ",".join(str(e.name) for e in sm['onroadEvents']
                                  if e.noEntry or e.softDisable or e.immediateDisable)[:60]
                         or "-"),
                      flush=True)

            # --- HIGH-RATE LATERAL TRACE -------------------------------------
            # The 1 Hz CAR line cannot resolve the oscillation: the limit cycle
            # was measured at 1.37 Hz, so a 1 Hz sample aliases it into noise and
            # every stdev computed from it is meaningless. Capture the loop rate
            # instead (~250 Hz) and dump a CSV so the jitter can be FFT'd rather
            # than guessed at. Ring-buffered and written every 10 s, so it costs
            # one list append per iteration and never grows without bound.
            if lat_trace["on"]:
                lat_trace["buf"].append((
                    now - t0,
                    float(tx_state.get("curvature", 0.0)),    # what the planner asked
                    float(tx_state.get("torque_norm", 0.0)),  # controller output, normalised
                    int(tx_state.get("applied", 0)),          # counts actually sent
                    float(getattr(cs_can, "steering_angle", 0.0)),
                    float(getattr(cs_can, "eps_effective", 0.0)),
                    float(getattr(cs_can, "v_ego", 0.0)),
                    1 if cc.latActive else 0,
                    # Needed to FIT latAccelFactor. The car's lateral accel only
                    # reflects OUR torque when the driver is not steering; with a
                    # hand on the wheel the fit is measuring the driver, which is
                    # why the first attempt returned a negative factor at
                    # corr=-0.31. Record driver torque and the engagement state so
                    # the regression can be restricted to samples where openpilot
                    # is actually the one driving.
                    float(getattr(cs_can, "steering_torque", 0.0)),
                    1 if ss.enabled else 0,
                    str(ss.state)))
                if (now - lat_trace["t"]) >= 10.0:
                    lat_trace["t"] = now
                    try:
                        # Drain first, then write. Anything appended by another
                        # thread while the write is in flight belongs to the NEXT
                        # flush, not this one -- iterating the live deque would
                        # race with append.
                        _rows = []
                        _b = lat_trace["buf"]
                        while _b:
                            _rows.append(_b.popleft())
                        if _rows:
                            # "a", not "w": each flush costs only the last ~10 s.
                            # Build one string and write it once rather than
                            # calling write() per row -- with the GIL held on the
                            # main loop, the syscall count is the part that hurts.
                            # First flush of the run truncates ("w") so a stale
                            # file from a previous launch is replaced rather than
                            # appended to with a header stranded in its middle.
                            # Every flush after that appends.
                            _new = not lat_trace["hdr"]
                            with open(lat_trace["path"], "w" if _new else "a") as _f:
                                if _new:
                                    _f.write("t,curvature,torque_norm,applied,angle,"
                                             "eps_eff,v,lat_active,drv_trq,op_enabled,"
                                             "op_state\n")
                                    lat_trace["hdr"] = True
                                _f.write("".join(
                                    "%.4f,%.6f,%.4f,%d,%.2f,%.1f,%.2f,%d,%.1f,%d,%s\n" % r
                                    for r in _rows))
                    except Exception:
                        pass

            # --- 1 Hz CAR STATE + EVERY BLOCKING EVENT -----------------------
            # The EDGE line above only fires on a TRANSITION and truncates the
            # event list to 60 chars, so a condition that blocks engagement
            # CONTINUOUSLY never appears there -- which is exactly the case you
            # need when openpilot sits at op=disabled and nothing changes.
            #
            # CAR line: everything the panda gates on (cruise bits, brake, gas),
            # everything the EPS reports back (request vs effective, lkas_block),
            # and what we ended up commanding. EVT line: every onroadEvent with
            # its category, untruncated -- N=noEntry S=softDisable I=immediate
            # W=warning E=enable P=preEnable U=userDisable M=permanent.
            # noEntry is the one that keeps openpilot from engaging at all.
            if (now - diag["t"]) >= 1.0:
                diag["t"] = now
                # PANDA-SIDE STATUS, published for the model role's web UI.
                #
                # The page is served by the MODEL role, which owns no panda -- so
                # every panda field on it (armed, lkas_hz, applied_torque,
                # controls_allowed, the ck counters) rendered from uninitialised
                # local state and read as "silent" while the panda role was in
                # fact armed and transmitting at 50 Hz. That is a display bug from
                # the Phase 2 split: carState and pandaStates were rewired across
                # msgq for selfdrived, but the UI's data source never was.
                #
                # A file rather than a new cereal message: these are diagnostic
                # counters with no schema home (tx_frames, applied_peak, worst_ms,
                # ck_ok/ck_bad), and inventing a capnp struct for them would be a
                # much larger change than the bug warrants. Written atomically so
                # the reader never sees a half-file, and stamped so a stale one
                # from a dead panda role can be ignored rather than believed.
                if owns_panda:
                    try:
                        # Snapshot the health tuple once -- can_thread rewrites
                        # health_state["v"] from another thread, so indexing it
                        # twice below could straddle an update.
                        _hv6 = health_state.get("v")
                        if _hv6 is not None and len(_hv6) < 7:
                            _hv6 = None
                        _ps = {
                            "t": now, "armed": bool(armed),
                            "tx_frames": int(tx_state.get("tx_frames", 0)),
                            "applied_torque": int(tx_state.get("applied", 0)),
                            "applied_peak": int(tx_state.get("applied_peak", 0)),
                            "want_peak": int(tx_state.get("want_peak", 0)),
                            "lkas_hz": round(float(tx_state.get("hz", 0.0)), 1),
                            "lkas_late": int(tx_state.get("late_frames", 0)),
                            "lkas_worst_ms": round(float(tx_state.get("worst_ms", 0.0)), 1),
                            "tx_blocked": int(tx_state.get("blocked", 0)),
                            "cam_seen": int(cam_state.get("seen", 0)),
                            "ck_ok": int(cam_state.get("ck_ok", 0)),
                            "ck_bad": int(cam_state.get("ck_bad", 0)),
                            "ck_skip": int(cam_state.get("ck_skip", 0)),
                            "lkas_block": bool(getattr(cs_can, "lkas_block", False)),
                            "eps_req": int(getattr(cs_can, "eps_request", 0)),
                            "eps_eff": int(getattr(cs_can, "eps_effective", 0)),
                            # LINK HEALTH. mazda_filter.h says in as many words to
                            # "WATCH rx_buffer_overflow ON THE FIRST RUN" with the
                            # radar shadow IDs present -- and then it was never
                            # surfaced anywhere. It is read at 10 Hz for the
                            # pandaStates publish (index 6, see _hv above) and was
                            # thrown away here, so when reception froze for 450 s
                            # on 2026-08-11 the one number that would have named
                            # the cause had to be reasoned about instead of read.
                            #
                            # rx_ovf climbing is the ONLY direct evidence that the
                            # host lost the 0.71 s drain race; resync/desync are
                            # the downstream damage (torn framing, discarded tail).
                            # Zero rx_ovf over a full drive with the radar IDs in
                            # MAZDA_HOST_IDS is what confirms the core pinning fixed
                            # this, rather than merely moving the threshold.
                            # ALPHA LONG COUNTERS. These live in the panda role's
                            # tx_state and were never published, so the web page
                            # rendered the MODEL role's own empty tx_state instead
                            # -- structurally 0 whatever the panda does.
                            #
                            # MEASURED 2026-08-11: that cost a wrong diagnosis. With
                            # cruise stuck unavailable under alpha long, long_tx_frames
                            # 0 / crz_info_tx 0 / radar_tp_tx 0 read as "we suppressed
                            # the radar and send nothing in its place" -- the exact
                            # shape of the armed-and-silent LKAS bug found earlier the
                            # same day, which made it far too easy to believe. The
                            # 10 Hz tester-present keep-alive increments radar_tp_tx or
                            # long_tx_err on EVERY loop iteration, so a genuine 0/0
                            # would mean the loop never ran once -- a conclusion the
                            # data could not actually support.
                            #
                            # long_halted matters most: long_tx_thread returns outright
                            # if the radar comes back while we are also writing 0x21b,
                            # and nothing outside that thread could see it had stopped.
                            "long_tx_frames": int(tx_state.get("long_tx_frames", 0)),
                            "crz_info_tx": int(tx_state.get("crz_info_tx", 0)),
                            "radar_tp_tx": int(tx_state.get("radar_tp_tx", 0)),
                            # SHADOW REPLAY LIVENESS. "replaying 7 frame(s)" is
                            # printed when the plan is built, not when frames go
                            # out, so it says nothing about whether the thread is
                            # actually transmitting. It has been dead for entire
                            # runs while that line sat in the log.
                            "shadow_sent": int(radar_shadow_state.get("sent", 0)),
                            "shadow_err": int(radar_shadow_state.get("err", 0)),
                            "shadow_last_err": str(radar_shadow_state.get("last_err", "")),
                            "long_tx_err": int(tx_state.get("long_tx_err", 0)),
                            "long_tx_last_err": str(tx_state.get("long_tx_last_err", "")),
                            "long_halted": bool(tx_state.get("long_halted", False)),
                            "rx_ovf": int(_hv6[6]) if _hv6 else -1,
                            "tx_ovf": int(_hv6[5]) if _hv6 else -1,
                            "resync_bytes": int(getattr(panda, "resync_bytes", -1)),
                            "desync_count": int(getattr(panda, "desync_count", -1)),
                            # Link wedges: frames exist in the silicon but never
                            # cross USB. Distinct from every other counter here --
                            # rx_ovf, resync and drain_gap all looked innocent while
                            # carState sat frozen for 11 minutes.
                            "wedge_recoveries": int(getattr(panda, "wedge_recoveries", 0)),
                            "last_wedge": str(getattr(panda, "last_wedge", "")),
                            # The 0.71 s queue deadline, measured rather than
                            # inferred. max_gap_ms is per-status-interval (reset on
                            # read) so a spike shows WHEN; max_gap_ever and
                            # over_budget accumulate for the session.
                            "drain_gap_ms": round(float(getattr(panda, "max_gap_ms", -1.0)), 1),
                            "drain_gap_max_ms": round(float(getattr(panda, "max_gap_ever", -1.0)), 1),
                            "drain_over_budget": int(getattr(panda, "over_budget", -1)),
                            # HOW MUCH the host retrieved, not just how often it
                            # asked. drain_gap can look healthy while the queue
                            # overflows, because one 16 KiB transfer only holds
                            # ~1260 packets -- less than one worst-case gap of
                            # traffic. drain_reads/can_recv > 1 means the extra
                            # transfers are doing real work; drain_capped > 0
                            # means even 8 of them were not enough and frames were
                            # left in the firmware queue to overflow.
                            "drain_reads": int(getattr(panda, "drain_reads", -1)),
                            "drain_capped": int(getattr(panda, "drain_capped", -1)),
                            # CAR STATE THE SHIM CANNOT CARRY.
                            #
                            # _CarStateFromMsgq maps only what stock carState has,
                            # so in the model role these stay at their defaults and
                            # the web page renders rpm 0 with cruise permanently
                            # off -- while the panda role's own log shows the real
                            # values. Reported as "engine on but the web says 0 rpm
                            # and the cruise state keeps stalling", which is exactly
                            # what it looks like from the page.
                            #
                            # engineRpm exists in the schema but this port never
                            # populates it; acc_armed/acc_active have no schema home
                            # at all. Both ride the status file instead, which is
                            # already crossing this boundary for the panda fields.
                            "rpm": int(getattr(cs_can, "rpm", 0)),
                            "acc_armed": bool(getattr(cs_can, "acc_armed", False)),
                            "acc_active": bool(getattr(cs_can, "acc_active", False)),
                            # The web page needs to know which mode it is rendering:
                            # under alpha long, cruise_available/cruise_enabled decode
                            # from the 0x21c WE generate, so they describe our own
                            # frame rather than the driver's button. Without this flag
                            # the page cannot tell the two cases apart and shows
                            # "cruise off" while the car is engaged.
                            "alpha_long": bool(alpha_long),
                            "cruise_available": bool(getattr(cs_can, "cruise_available", False)),
                            "cruise_enabled": bool(getattr(cs_can, "cruise_enabled", False)),
                            "v_ego_kph": round(float(getattr(cs_can, "v_ego", 0.0)) * 3.6, 1),
                        }
                        _tmp = PANDA_STATUS_FILE + ".tmp"
                        with open(_tmp, "w") as _f:
                            json.dump(_ps, _f)
                        # Window reset AFTER a successful write, so drain_gap_ms
                        # always describes the interval this sample covers.
                        try:
                            panda.max_gap_ms = 0.0
                        except Exception:
                            pass
                        os.replace(_tmp, PANDA_STATUS_FILE)
                    except Exception:
                        pass
                g = lambda a, d=0: getattr(cs_can, a, d)
                # Bus 0 core health, from the cache can_thread refreshes at 1 Hz.
                # Keys are can_health_state's own names ("rx"/"fwd"), not the
                # firmware struct's total_rx_cnt/total_fwd_cnt.
                _canh = (can_health_state["v"] or {}).get(0, {})
                print("CAR %8.2fs v=%5.1fkph gear_rpm=%-5s | crz_avail=%d crz_en=%d "
                      "acc_armed=%d acc_active=%d brake=%d gas=%d "
                      "| drv_trq=%+6.1f pressed=%d angle=%+7.2f "
                      "| eps_req=%+5d eps_eff=%+5d lkas_block=%d hands_off=%d "
                      "| blink=%d/%d bsm=%d/%d "
                      "| cam_seen=%d cam_age=%.1fs lane_age=%.1fs ck=%d/%d/%d(l%d,d%d,a%d) "
                      # vtgt is the speed the MPC is actually chasing. It was a
                      # config constant (V_MAX_KPH, 116) and nothing displayed it,
                      # so "accelerating toward 116 kph" looked identical to normal
                      # operation right up until the PCM started obeying us.
                      # slim= is the speed-limit target, empty unless
                      # OP_SPEED_LIMIT=1. Format: "tgt=N lim=N[i] STD|MADS+N".
                      # The trailing 'i' means the limit was INFERRED from the
                      # highway type rather than read off a maxspeed tag -- only
                      # 10.9% of NY ways carry one, so expect to see it often.
                      # src is which of the two blended sources is BINDING:
                      # "mpc" = the optimiser's plan, "e2e" = the model braking
                      # harder than the MPC planned. Upstream takes min(e2e, mpc),
                      # so seeing e2e means the model spotted something the MPC's
                      # single lead object did not. A run where src never reads e2e
                      # means the action head is not reaching the blend.
                      "| mpc=%d/%-3s vtgt=%.0f%s mv=%.1f/%.1f ma=%+.2f a_raw=%+.2f a_cmd=%+.2f lim=%-10s "
                      "| txgap=%.0f/%.0fms late=%d "
                      "| ang_off=%+.2f(%d) "
                      "| lc_state=%d lc_prob=%.3f desire=%d plan_y=%s "
                      # ACC set speed, raw and both candidate scalings, so ONE look
                      # at the cluster settles the DBC's ambiguous unit.
                      #
                      # raw=-1 means 0x21F has not arrived ONCE since startup --
                      # it does NOT mean the frame is absent now. The three value
                      # fields are a latch that is never invalidated (see the note
                      # in dashcam.py's CarStateFromCAN), so under suppression they
                      # keep reading whatever was latched before it. age is the
                      # field that says whether CRZ_EVENTS is still on the bus:
                      # -1 = never seen, and a value that climbs past ~0.1 s means
                      # the 50 Hz frame has stopped. THAT is the measurement that
                      # decides whether the radar's HMI half survives suppression.
                      "| setspd raw=%d a=%.1f b=%.1f age=%.1fs "
                      # Rising-edge counts for SET+/SET-/RES/OFF off 0x09d, and
                      # how long ago the last one landed. Levels are useless at
                      # 1 Hz -- a press is a few hundred ms and the sample lands
                      # between presses. Without these, "pressing SET does
                      # nothing" cannot be told apart from "SET was never
                      # pressed", which is the question the alpha-long runs of
                      # 2026-08-12 were unable to answer about themselves.
                      "| btn +%d/-%d/R%d/O%d last=%.0fs "
                      "| tx_blocked=%d rx_invalid=%d "
                      # CAN CORE HEALTH, on the panda's own line. rx is the
                      # silicon's total_rx_cnt: if it stops advancing while the
                      # ignition is on, the core is wedged in bxCAN init -- it then
                      # stops forwarding cam <-> bus 0 and the cluster raises a
                      # front camera sensor fault that looks exactly like the
                      # alpha-long radar one. bus_off/txe/rxe all read HEALTHY in
                      # that state (a core in init does not error-count), so rx is
                      # the field to watch; the others catch the ordinary
                      # error-passive / bus-off failures instead.
                      # wedge counts the automatic 0xd8 recoveries.
                      "| can rx=%d fwd=%d off=%d txe=%d rxe=%d wedge=%d "
                      # Alpha long, on the panda's own log line. Without this the
                      # only place these appeared was a web field fed by the model
                      # role, which never has them -- see the note in the status
                      # writer. "halt" is the one that must be impossible to miss:
                      # long_tx_thread returns for good if the radar resumes.
                      "%s"
                      "| ctrl_allowed=%d lat_active=%d op=%s applied_trq=%+5d"
                      % (now - t0, g("v_ego") * 3.6, g("rpm"),
                         int(g("cruise_available")), int(g("cruise_enabled")),
                         int(g("acc_armed")), int(g("acc_active")),
                         int(g("brake_pressed")), int(g("gas_pressed")),
                         g("steering_torque"), int(g("steering_pressed")),
                         g("steering_angle"), int(g("eps_request")),
                         int(g("eps_effective")), int(g("lkas_block")),
                         int(g("hands_off_5s")),
                         int(g("left_blinker")), int(g("right_blinker")),
                         int(g("left_blindspot")), int(g("right_blindspot")),
                         int(cam_state.get("seen", 0)),
                         (now - cam_state["last_t"]) if cam_state["last_t"] else -1.0,
                         (now - cam_state["lane_t"]) if cam_state["lane_t"] else -1.0,
                         # ok/bad/skipped. skipped is the one that decides whether
                         # OP_LKAS_COPY_CAM_LINES can help at all: it counts camera
                         # frames carrying LINE_NOT_VISIBLE or LDW, which we
                         # normally pin to 0. Stays 0 -> this camera never sets
                         # them, the copy is a no-op, and the front camera fault
                         # is something else.
                         int(cam_state.get("ck_ok", 0)), int(cam_state.get("ck_bad", 0)),
                         int(cam_state.get("ck_skip", 0)),
                         # which bit forced the skip: LINE_NOT_VISIBLE, LDW,
                         # ANGLE_ENABLED. OP_LKAS_COPY_CAM_LINES only copies the
                         # first two, so a run dominated by 'a' means the flag
                         # cannot fix the mismatch.
                         int(cam_state.get("ck_lnv", 0)),
                         int(cam_state.get("ck_ldw", 0)),
                         int(cam_state.get("ck_ang", 0)),
                         int(_gov.get("mpc", 0)), str(_gov.get("src", "-")),
                         float(_mpc_state.get("v_cruise", 0.0)) * 3.6,
                         (" [%s]" % _speed_target.line()) if _speed_target is not None else "",
                         float(_mpc_state["v"]) * 3.6, float(cs_can.v_ego) * 3.6,
                         float(_mpc_state["a"]),
                         float(_gov.get("raw", 0.0)), float(_gov.get("a", 0.0)),
                         (str(_gov.get("why", "")) or "-")[:10],
                         float(tx_state.get("worst_ms", 0.0)),
                         float(tx_state.get("worst_ms_run", 0.0)),
                         int(tx_state.get("late_frames", 0)),
                         float(ang_off["v"]), int(ang_off["n"]),
                         int(mstate.get("lc_state", 0)),
                         float(mstate.get("lane_change_prob", 0.0)),
                         int(mstate.get("desire", 0)),
                         ("%+.2f/%+.2f/%+.2f" % mstate["plan_y"]
                          if mstate.get("plan_y") else "-"),
                         int(getattr(cs_can, "set_speed_raw", -1)),
                         float(getattr(cs_can, "set_speed_kph", -1.0)),
                         float(getattr(cs_can, "set_speed_ms", -1.0)) * 3.6,
                         ((now - _sst) if (_sst := float(getattr(cs_can, "set_speed_t", 0.0)))
                          else -1.0),
                         *(lambda _bc: (int(_bc.get("set_p", 0)), int(_bc.get("set_m", 0)),
                                        int(_bc.get("res", 0)), int(_bc.get("off", 0))))(
                             getattr(cs_can, "btn_counts", {})),
                         ((now - _blt) if (_blt := float(getattr(cs_can, "btn_last_t", 0.0)))
                          else -1.0),
                         int(health_state["v"][3]) if health_state["v"] else -1,
                         int(health_state["v"][4]) if health_state["v"] else -1,
                         # Read from the cached per-bus health that can_thread
                         # already refreshes at 1 Hz -- NOT a fresh control
                         # transfer. This line runs on the tx path and the
                         # panda_lock is what the 0x243 stream is most sensitive
                         # to; adding a USB round trip here to print a diagnostic
                         # would risk causing the very irregularity it exists to
                         # help diagnose.
                         int(_canh.get("rx", -1)),
                         int(_canh.get("fwd", -1)),
                         int(_canh.get("bus_off", -1)),
                         int(_canh.get("tx_err", -1)),
                         int(_canh.get("rx_err", -1)),
                         int(getattr(panda, "wedge_recoveries", 0)),
                         ("| long tx=%d crz=%d tp=%d err=%d%s "
                          % (int(tx_state.get("long_tx_frames", 0)),
                             int(tx_state.get("crz_info_tx", 0)),
                             int(tx_state.get("radar_tp_tx", 0)),
                             int(tx_state.get("long_tx_err", 0)),
                             " HALTED" if tx_state.get("long_halted") else "")
                          if alpha_long else ""),
                         int(_ca), int(cc.latActive),
                         str(ss.state), int(tx_state["applied"])),
                      flush=True)
                _evs = []
                for e in sm['onroadEvents']:
                    _t = "".join(c for c, on in (("N", e.noEntry), ("S", e.softDisable),
                                                 ("I", e.immediateDisable), ("W", e.warning),
                                                 ("E", e.enable), ("P", e.preEnable),
                                                 ("U", e.userDisable), ("M", e.permanent)) if on)
                    _evs.append("%s[%s]" % (str(e.name), _t or "-"))
                print("EVT %8.2fs %s" % (now - t0, "  ".join(_evs) or "(no events)"),
                      flush=True)
                if alpha_long:
                    # SUPPRESSION CENSUS. What is still on bus 0, per address,
                    # over the last window, with our own transmits subtracted.
                    #
                    # This exists because the port has been reasoning about
                    # "which frames does suppression silence?" from indirect
                    # evidence -- a latched set speed, a cluster message -- and
                    # getting it wrong. can_census counts everything can_thread
                    # sees, including the echo of our own frames, so a raw count
                    # of 0x21b says nothing; the delta minus _ourtx is the
                    # radar's actual share. Same subtraction the 0x21b watchdog
                    # in long_tx_thread does, applied to every address at once.
                    #
                    # 0x21f is the one to read first: it is the radar's third
                    # longitudinal output and NOTHING in this port has ever
                    # transmitted it, so its count here is the car's alone. If it
                    # holds ~50 Hz under suppression the radar's HMI half is
                    # still running and engagement should be reachable the way
                    # upstream reaches it. If it goes to 0, the ACC state machine
                    # went with it and it has to be synthesised.
                    _cw = now - diag.get("ct", now)
                    _prev = diag.get("census", {})
                    _pours = diag.get("census_ours", {})
                    _cur = {_a: can_census[0].get(_a, 0) for _a in CENSUS_IDS}
                    _co = {_a: _ourtx.get(_a, 0) for _a in CENSUS_IDS}
                    if _cw >= 0.5 and _prev:
                        _parts = []
                        for _a in CENSUS_IDS:
                            _n = (_cur[_a] - _prev.get(_a, 0)) - (_co[_a] - _pours.get(_a, 0))
                            # Clamp only the print: a small negative means our own
                            # echo outran the census by a frame across the window
                            # boundary, not that the car sent a negative number.
                            _parts.append("%03x=%.0f" % (_a, max(_n, 0) / _cw))
                        print("CENSUS %6.2fs bus0 Hz (ours subtracted) %s"
                              % (now - t0, " ".join(_parts)), flush=True)
                    diag["ct"], diag["census"], diag["census_ours"] = now, _cur, _co
                if (now - diag.get("rt", 0)) >= 5.0:
                    diag["rt"] = now
                    _parts = []
                    for _a in RADAR_IDS:
                        _r = radar_shadow.get(_a)
                        if _r is None:
                            _parts.append("%s:SILENT" % RADAR_NAMES[_a])
                        else:
                            # age is time since this address was last seen on the
                            # bus -- which, once the shadow replay is running, is
                            # OUR echo and not the radar. A healthy replay pins it
                            # near zero and says nothing about whether the radar is
                            # back; that is what the CENSUS line is for.
                            _parts.append("%s:%d age=%.1fs %s" % (RADAR_NAMES[_a], _r["n"],
                                          now - _r["t"], _r["d"].hex()))
                    print("RADAR %7.2fs %s" % (now - t0, "  ".join(_parts)), flush=True)

                    # CKBAD -- the frames our own packer got wrong.
                    #
                    # Printed on the same slow tick as RADAR because it is rare and
                    # only interesting in aggregate. Each row is the camera's frame,
                    # ours built from the SAME decoded fields, and the fields
                    # themselves. Wherever those two hex strings differ is a byte
                    # create_steering_control computes wrong -- and since that is the
                    # function that builds what we TRANSMIT, it is a byte the EPS
                    # sees wrong too.
                    _bad = cam_state.get("ck_bad_frames") or []
                    if _bad and not cam_state.get("ck_bad_shown"):
                        cam_state["ck_bad_shown"] = True
                        print("CKBAD  %d captured (ok=%d bad=%d). cam vs ours:"
                              % (len(_bad), cam_state.get("ck_ok", 0),
                                 cam_state.get("ck_bad", 0)), flush=True)
                        for _f in _bad[:12]:
                            _diff = [i for i in range(8)
                                     if _f["cam"][2 * i:2 * i + 2] != _f["ours"][2 * i:2 * i + 2]]
                            print("   cam=%s ours=%s  ang=%-6d ahi=%d trq=%-5d ctr=%-2d  differs@bytes %s"
                                  % (_f["cam"], _f["ours"], _f["ang"], _f["ahi"],
                                     _f["torque"], _f["ctr"], _diff), flush=True)

            # STATE + log at ~20 Hz -- the phone and the jsonl don't need 100 Hz,
            # and writing every loop would bloat the trace 5x.
            if now - last_state < 0.05:
                loop_next += LOOP_DT
                s = loop_next - time.time()
                if s > 0:
                    time.sleep(s)
                else:
                    loop_next = time.time()
                continue
            last_state = now

            cal = getattr(runner, "calibrator", None)
            rpy = list(getattr(runner, "calib_euler", (0.0, 0.0, 0.0)))
            nsamp = int(getattr(cal, "valid_blocks", 0) or 0) if cal is not None else 0
            # Ask op_calibrate where it actually saves rather than repeating the
            # path: a hardcoded copy here reported "not saved" forever the moment
            # the writer moved off /tmp.
            calfile = os.path.isfile(_calib_file_path())
            pxyz = out.get("path_xyz")

            # --- action head telemetry -------------------------------------------
            # `curv` above is what the controller gets. These are what produced it:
            # the head's raw lateral accel (pre v_ego^2 division, so directly against
            # the 3.0 m/s^2 ceiling clip_curvature imposes) and the longitudinal
            # command nothing here actuates. All zero on a model without the head.
            model_name = os.path.basename(getattr(runner, "engine_path", "") or "-")
            _act, _araw = mstate.get("action"), mstate.get("action_raw")
            lat_accel = float(_araw[0]) if _araw is not None else 0.0
            desired_accel = float(_act["desiredAcceleration"]) if _act is not None else 0.0
            should_stop = bool(_act["shouldStop"]) if _act is not None else False

            el = now - t0
            with LOCK:
                STATE.update(
                    t=el, frames=frames, fps=frames / max(el, 1e-3),
                    v_ego_kph=cs_can.v_ego * 3.6, rpm=cs_can.rpm,
                    cruise_available=cs_can.cruise_available,
                    cruise_enabled=cs_can.cruise_enabled,
                    brake=cs_can.brake_pressed,
                    steer_torque=cs_can.steering_torque,
                    steer_pressed=cs_can.steering_pressed,
                    curvature=float(curv),
                    path_reach=float(pxyz[-1][0]) if pxyz is not None else 0.0,
                    # --- the action head -------------------------------------
                    model=model_name, action_head=bool(getattr(runner, "has_action", False)),
                    lat_accel=lat_accel, desired_accel=desired_accel, should_stop=should_stop,
                    calib_rpy=[float(x) for x in rpy],
                    # UNCLAMPED estimate. calib_rpy is run through sanity_clip, so a
                    # camera aimed outside openpilot's range reports the clamp value
                    # and stops responding to physical adjustment entirely. This is
                    # what the mount angle actually wants to be, in degrees.
                    calib_rpy_raw_deg=[round(float(x) * 57.2958, 3) for x in
                                       getattr(cal, "rpy_unclipped", rpy)] if cal is not None else None,
                    calib_clipped=bool(getattr(cal, "clipped", False)) if cal is not None else False,
                    calib_samples=nsamp, calib_file=calfile,
                    calib_status=str(getattr(cal, "status", "-")) if cal is not None
                                 else "no calibrator",
                    calib_converged=bool(nsamp >= 5),   # INPUTS_NEEDED
                    op_enabled=bool(ss.enabled), op_state=str(ss.state),
                    lat_active=bool(cc.latActive),
                    would_command_torque=float(cc.actuators.torque),
                    armed=armed, tx_frames=int(tx_state["tx_frames"]),
                    applied_torque=int(tx_state["applied"]),
                    # --- headroom telemetry: is STEER_MAX the binding limit? ---
                    steer_max=int(CarControllerParams.STEER_MAX),
                    applied_peak=int(tx_state["applied_peak"]),
                    want_peak=int(tx_state["want_peak"]),
                    lkas_hz=round(float(tx_state["hz"]), 1),
                    # counts/s the ramp limiter allows at the ACHIEVED rate
                    ramp_cps=round(float(tx_state["hz"]) * CarControllerParams.STEER_DELTA_UP, 0),
                    # tx regularity: target interval vs the worst actually seen
                    lkas_target_ms=round(1000.0 / lkas_hz, 2),
                    lkas_worst_ms=round(float(tx_state["worst_ms"]), 1),
                    lkas_worst_ms_all=round(float(tx_state["worst_ms_all"]), 1),
                    lkas_late=int(tx_state["late_frames"]),
                    # Link scheduler telemetry. lkas_recv_ms is what a can_recv
                    # actually costs on this link; the achieved interval is
                    # max(period, send + recv), so recv_ms above the period is
                    # what caps --lkas-hz.
                    lkas_recv_ms=round(float(tx_sched["recv_ms"]), 1),
                    # 0.5 ms ticks spent waiting to phase a recv behind a send.
                    lkas_rx_waits=int(tx_sched["waits"]),
                    cam_trq_peak=int(cam_state["trq_peak"]),
                    # --- EPS feedback: does the RACK follow what we send? ------
                    # Everything above is what this stack asked for. These four
                    # are what the EPS did about it, read off 0x240/0x241.
                    # eps_request vs eps_effective is the EPS's own account of
                    # command-in vs torque-applied, so if effective flattens
                    # while request keeps climbing, the rack is the ceiling and
                    # no amount of STEER_MAX or ramp tuning moves it. If they
                    # track, the ceiling is entirely on our side.
                    eps_request=int(cs_can.eps_request),
                    eps_effective=int(cs_can.eps_effective),
                    eps_motor_torque=round(float(cs_can.eps_motor_torque), 1),
                    eps_seen=int(cs_can.eps_seen),
                    eps_req_peak=int(eps_peak["req"]), eps_eff_peak=int(eps_peak["eff"]),
                    # follow ratio at the peak: 1.0 = the EPS applied everything
                    # it was asked for, < 1.0 = it clamped. Only meaningful once
                    # eps_req_peak is well above the stock camera's own peak.
                    eps_follow=round(eps_peak["eff"] / eps_peak["req"], 3) if eps_peak["req"] > 0 else -1.0,
                    lkas_block=bool(cs_can.lkas_block),
                    hands_off_5s=bool(cs_can.hands_off_5s),
                    steer_angle_rate=round(float(cs_can.steer_angle_rate), 1),
                    left_blinker=bool(cs_can.left_blinker),
                    right_blinker=bool(cs_can.right_blinker),
                    # what the model was ASKED for vs what it says it is doing
                    desire=int(mstate.get("desire", 0)),
                    lane_change_state=int(mstate.get("lc_state", 0)),
                    lane_change_dir=int(mstate.get("lc_dir", 0)),
                    lane_change_prob=round(float(mstate.get("lane_change_prob", 0.0)), 3),
                    left_blindspot=bool(cs_can.left_blindspot),
                    right_blindspot=bool(cs_can.right_blindspot),
                    cam_bit1=int(cam_state["BIT_1"]), cam_seen=int(cam_state["seen"]),
                    cam_lane_seen=int(cam_state.get("lane_seen", 0)),
                    # camera liveness: BIT_1 is latched from the last frame seen, so
                    # a stale camera keeps asserting "LKAS active" with nothing behind
                    # it. These say how long ago that frame actually was.
                    cam_age_s=round(now - cam_state["last_t"], 2) if cam_state["last_t"] else -1.0,
                    cam_lane_age_s=round(now - cam_state["lane_t"], 2) if cam_state["lane_t"] else -1.0,
                    # mazdacan 0x243 pack+checksum proven against the real camera
                    ck_ok=int(cam_state["ck_ok"]), ck_bad=int(cam_state["ck_bad"]),
                    ck_angle_seen=int(cam_state["ck_angle_seen"]),
                    # Histogram of the gap in the camera's own CTR between
                    # consecutive frames WE saw. All-1s means we receive the
                    # camera whole and ~16 Hz is its true rate; a peak at 6
                    # means we see every sixth frame of a 100 Hz stream.
                    cam_ctr_gaps={str(k): v
                                  for k, v in sorted(cam_state["ctr_d"].copy().items())},
                    steer_angle_deg=float(cs_can.steering_angle),   # does the wheel respond to our torque?
                    # what the MODEL wants the wheel at, vs steer_angle_deg above
                    model_angle_deg=round(float(tx_state["angle_deg"]), 2),
                    tx_angle_raw=int(tx_state["angle_raw"]),
                    steer_angle_injected=bool(steer_angle_inject),
                    controls_allowed=bool(health_state["v"][10]) if health_state["v"] else False,
                    # tx_frames counts frames HANDED TO the panda over USB, not frames
                    # that reached the bus: can_send succeeds and the firmware drops the
                    # frame afterwards if mazda_tx_hook finds a violation. This counter
                    # (health_t.safety_tx_blocked_pkt) is the only way to see that --
                    # if it climbs while steering, our 0x243 is being rejected and the
                    # EPS is seeing gaps in a stream it faults on.
                    panda_tx_blocked=int(health_state["v"][3]) if health_state["v"] else 0,
                    # frames the panda rejected, and how many times we had to zero
                    # apply_last to break the resulting lockout. resyncs climbing
                    # while steering means the rate limits are being tripped.
                    tx_blocked_seen=int(tx_state["blocked_seen"]),
                    tx_resyncs=int(tx_state["resyncs"]),
                    # count + most recent only; the full history is in the run
                    # log as EDGE lines (see the note at the edge detector)
                    engage_edges=int(edge["n"]), engage_last=str(edge["last"]),
                    panda_rx_invalid=int(health_state["v"][4]) if health_state["v"] else 0,
                    panda_faults=int(health_state["v"][7]) if health_state["v"] else 0,
                    # --- is the panda still the thing we armed? ---------------
                    # panda_safety_mode != 13 (SAFETY_MAZDA) means the firmware
                    # is in SAFETY_SILENT, which sets disable_forwarding and
                    # cuts the ENTIRE camera -> car path, not just our 0x243.
                    # panda_remodes counts how many times the watchdog in
                    # tx_thread had to put it back. panda_uptime is logged as a
                    # CHECK on that watchdog, not as a reboot detector: TIM9
                    # never fires on this board, so uptime is pinned at 0 and
                    # panda_resets can never trip (see the SIGTERM note in
                    # main()). If uptime is ever non-zero the tick came alive
                    # and the firmware's own heartbeat->SILENT revert is armed
                    # again -- which on this relay-less board cuts the camera
                    # off the car -- so that is worth seeing.
                    panda_safety_mode=int(health_state["v"][12]) if health_state["v"] else -1,
                    # Alpha-long tx telemetry. radar_tp_tx is the keep-alive that
                    # holds the radar's programming session: the probe shows 2 Hz
                    # of it holds for 90 s, so a stalled counter here is the first
                    # thing to check if the radar comes back mid-drive.
                    long_tx_frames=int(tx_state.get("long_tx_frames", 0)),
                    crz_info_tx=int(tx_state.get("crz_info_tx", 0)),
                    radar_tp_tx=int(tx_state.get("radar_tp_tx", 0)),
                    radar_resuppress=int(tx_state.get("radar_resuppress", 0)),
                    # PEDALS-derived MRCC state -- the signals that ACTUALLY drive
                    # engagement under alpha long. cruise_available/cruise_enabled
                    # above come from CRZ_CTRL, which is OUR OWN frame once the
                    # radar is suppressed, so they are circular and cannot explain
                    # why MADS does or does not engage. These two can: acc_armed is
                    # what mazda.h feeds pcm_cruise_check() under MAZDA_MADS, and
                    # what carState.cruiseState.enabled is set from with --mads.
                    # Live-tuned lateral delay, so the log records what was
                    # actually in effect rather than what the command line said.
                    steer_delay=round(STEER_DELAY, 4),
                    acc_armed=bool(cs_can.acc_armed),
                    acc_active=bool(cs_can.acc_active),
                    # govern_accel: raw action head vs what was actually commanded,
                    # and which clamp bound it. accel_raw != accel_cmd is the whole
                    # point -- it shows the governor working rather than hiding it.
                    accel_raw=round(float(_gov["raw"]), 3),
                    accel_cmd=round(float(_gov["a"]), 3),
                    accel_limit=str(_gov["why"]),
                    lead_d_m=(round(float(mstate["lead_d"]), 1)
                              if mstate.get("lead_d") is not None else None),
                    # lead_v_ms and lead_p are logged so the ABSOLUTE-vs-RELATIVE
                    # question can be answered from data rather than argued from
                    # source. openpilot's radard.py settles it (lead.v[0] is ground
                    # speed -- see the note in _MpcLead), but that convention had
                    # already been guessed wrong once here, and the cost was a car
                    # that accelerated at leads instead of holding station.
                    #
                    # The check: compare lead_v_ms against v_ego + d(lead_d_m)/dt.
                    # If they agree, it is absolute. If lead_v_ms instead matches
                    # d(lead_d_m)/dt alone, it is relative and _MpcLead is wrong.
                    lead_v_ms=(round(float(mstate["lead_v"]), 2)
                               if mstate.get("lead_v") is not None else None),
                    lead_p=(round(float(mstate["lead_p"]), 3)
                            if mstate.get("lead_p") is not None else None),
                    long_tx_err=int(tx_state.get("long_tx_err", 0)),
                    long_tx_last_err=tx_state.get("long_tx_last_err", ""),
                    long_halted=bool(tx_state.get("long_halted", False)),
                    panda_uptime=int(health_state["v"][0]) if health_state["v"] else -1,
                    panda_remodes=int(tx_state["remodes"]),
                    panda_resets=int(tx_state["panda_resets"]),
                    panda_heartbeat_lost=int(health_state["v"][16]) if health_state["v"] else 0,
                    panda_ignition=int(health_state["v"][8]) if health_state["v"] else 0,
                    # RX FIFO overflow inside the panda: frames the CAN core got
                    # and the serial link was too slow to drain. The camera is
                    # observed at ~16 Hz on a stream that should be 100 Hz, and
                    # this is the counter that says whether that is loss or the
                    # camera's real rate.
                    panda_rx_overflow=int(health_state["v"][6]) if health_state["v"] else 0,
                    panda_tx_overflow=int(health_state["v"][5]) if health_state["v"] else 0,
                    # Per-CAN firmware counters. can_fwd2 is the camera -> car
                    # forward count: if it is flat while the camera is alive on
                    # bus 2, the car is not seeing the FSC at all.
                    # .copy() because can_thread adds to these dicts from
                    # another thread, and iterating one mid-insert raises
                    # "dictionary changed size during iteration". copy() is a
                    # single C call, so the snapshot itself is safe.
                    **{("can_%s%d" % (k, b)): v
                       for b, hh in can_health_state["v"].copy().items()
                       for k, v in hh.items()},
                    events=[str(e.name) for e in sm['onroadEvents']][:6],
                    events_blocking=[str(e.name) for e in sm['onroadEvents']
                                     if e.noEntry or e.softDisable
                                     or e.immediateDisable][:4],
                    # where the CAMERA sits between the ego lane lines (+ = right
                    # of centre). See lane_offset(); this is the measurement that
                    # separates "the model is steering off centre" from "the model
                    # is centred and the camera is not where it thinks it is".
                    **(dict(zip(("lane_off_m", "lane_off_near_m", "lane_width_m",
                                 "lane_p_left", "lane_p_right"),
                                (round(x, 3) for x in _loff)))
                       if (_loff := lane_offset(out.get("lane_lines"),
                                                out.get("lane_prob"))) is not None
                       else dict(lane_off_m=None, lane_off_near_m=None,
                                 lane_width_m=None, lane_p_left=None,
                                 lane_p_right=None)),
                    moving_seconds=moving,
                    note=("MOVING >15km/h - calibration learning" if cs_can.v_ego > 4.17 else
                          "moving but under 15 km/h - too slow to calibrate" if cs_can.v_ego > 0.5
                          else "stationary - calibration will NOT learn until you drive"),
                )

                # MERGE THE PANDA ROLE'S STATUS INTO THE PAGE.
                #
                # This UI is served by the MODEL role, which owns no panda, so
                # every panda field above rendered from uninitialised local state:
                # armed False, lkas_hz 0.0, applied_torque 0, the ck counters 0 --
                # while the panda role was armed and transmitting at 50 Hz. The
                # page reported "silent" and was believed, which is worse than
                # showing nothing at all.
                #
                # Overlay only in the model role, and only from a FRESH file. A
                # stale one means the panda role has died or was never started;
                # showing its last values then would be the same lie in a new
                # form, so age it out and let the fields read as absent instead.
                if role == "model":
                    try:
                        with open(PANDA_STATUS_FILE) as _f:
                            _ps = json.load(_f)
                        if (time.time() - float(_ps.get("t", 0))) < 5.0:
                            _ps.pop("t", None)
                            STATE.update(_ps)
                            STATE["panda_role"] = "live"
                        else:
                            STATE["panda_role"] = "stale"
                    except Exception:
                        STATE["panda_role"] = "absent"
                # JPEG encoding moved to preview_thread -- doing it here put a
                # full encode of every frame on the model's critical path.
                STATE["_path"] = pxyz if pxyz is not None else None
                # Everything draw_model() needs, captured under the same lock as
                # the rest of the state so the preview thread cannot render a path
                # from one frame against lane lines from the next.
                STATE["_model"] = {
                    "path": pxyz,
                    "lanes": out.get("lane_lines"),
                    "edges": out.get("road_edges"),
                    "lane_prob": out.get("lane_prob"),
                    "lead": out.get("lead"),
                    "lead_prob": out.get("lead_prob"),
                }

            rec = {k: v for k, v in STATE.items() if not k.startswith("_")}
            # Address census every 10 s rather than every record: it is a dict of
            # ~30 entries and the log already runs to tens of MB per drive. Ten
            # seconds is plenty to derive a per-address rate by differencing two
            # samples, which is the actual question (what rate does the real
            # camera send 0x243 at, vs the rate we imitate it with).
            if frames % 200 == 0:
                rec["can_census"] = {str(b): {("0x%x" % a): n
                                              for a, n in sorted(c.copy().items())}
                                     for b, c in can_census.items() if c}
            logf.write(json.dumps(rec) + "\n")
            if frames % 60 == 0:
                logf.flush()

            # pace the publish loop at 100 Hz
            loop_next += LOOP_DT
            s = loop_next - time.time()
            if s > 0:
                time.sleep(s)
            else:
                loop_next = time.time()
    finally:
        stop_flag["v"] = True          # stops the tx thread within one 10 ms cycle
        time.sleep(0.05)
        # Stop transmitting FIRST, before anything else, even if a later step throws.
        # The tx thread is already stopped above, so no frame of ours can reach the
        # bus from here on regardless of which safety mode we leave behind.
        #
        # WHICH mode to leave is the trade. SAFETY_SILENT also sets
        # disable_forwarding, and with no harness relay on this build that severs
        # the forward camera from the car -- so quitting the stack would itself
        # raise a front camera fault, every time, which is what kept happening.
        # With the car awake, leave Mazda safety in place so cam<->car forwarding
        # and the camera's ACK survive teardown.
        #
        # KNOWN COST, stated plainly: a car safety mode outliving its host process
        # is normally guarded by panda's heartbeat watchdog, which reverts to
        # SILENT after 2-5 s without a 0xf3. On THIS board that watchdog is DEAD --
        # the TIM9 tick ISR never fires, health.uptime stays 0 forever (verified
        # 2026-07-30) -- so nothing will ever revert it. The panda stays permissive
        # with no supervisor until the next process arms it or the car sleeps.
        # Nothing transmits, but the safety net is a relay this build does not have.
        try:
            if owns_panda:
                with panda_lock:
                    panda.control_write(0xdc, SAFETY_NOOUTPUT, 0)
                print("panda: transmit stopped; safety left at SAFETY_NOOUTPUT "
                      "(cam<->car passthrough kept alive, nothing transmittable)")
        except Exception as e:
            print(f"panda: could NOT re-assert safety mode ({e}) -- power-cycle before driving")
        time.sleep(0.25)
        for p in procs:
            try:
                p.send_signal(signal.SIGINT); p.wait(timeout=4)
            except Exception:
                p.kill()
        # cam is None whenever there is no camera to open -- --can-only, and now
        # --role panda. Unguarded this raised AttributeError on every clean
        # shutdown of those modes. It fires AFTER the panda has been returned to
        # SAFETY_NOOUTPUT, so nothing unsafe came of it, but a traceback on the
        # normal exit path is exactly what hides the next real one.
        if cam is not None:
            cam.close()
        logf.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--lkas-hz", type=float, default=70.0,
                    help="0x243 tx rate. Stock is 100; the panda and link were "
                         "measured clean at 100 and the cap is GIL contention in "
                         "this process, not the transport. 70 because the EPS "
                         "faults on irregular 0x243 rather than on a lower rate, "
                         "and 70 leaves ~14 ms of period against a can_recv that "
                         "costs ~15.5 ms -- see the link scheduler in can_thread. "
                         "Watch lkas_worst_ms and lkas_rx_forced before going higher.")
    ap.add_argument("--can-only", action="store_true",
                    help="CAN work only: no camera, no supercombo, no JPEG preview, "
                         "no web server, no openpilot daemons. Keeps the 0x243 "
                         "transmit thread, the CAN receive thread and the full "
                         "JSONL log. Use it when the question is about the bus "
                         "rather than about driving -- everything it removes was "
                         "competing with the tx thread for the GIL, and an "
                         "irregular 0x243 is what raises the front camera fault. "
                         "NOTE: no model means no lateral control at all.")
    ap.add_argument("--role", choices=("both", "panda", "model"), default="both",
                    help="Split the stack across two PROCESSES, the way stock "
                         "openpilot does. 'both' (default) is the historical "
                         "single-process behaviour and remains the fallback. "
                         "'panda' owns the USB handle and runs the CAN receive, "
                         "0x243 transmit and carState threads -- stock's pandad "
                         "plus card. 'model' runs the camera, supercombo and the "
                         "message publishing, and never touches the panda. "
                         "WHY: a 10 ms transmit deadline is held to +-0.2 ms "
                         "against ONE competing CPU-bound Python thread and "
                         "collapses to p99 442 ms against THREE (measured "
                         "2026-08-09). Stock never hits this because pandad is "
                         "C++ and card is its own process; we opted out of that "
                         "when we replaced them with in-process threads. Two "
                         "processes means two GILs, which is the actual fix -- "
                         "the CAN link itself accounts for under 4%% of the "
                         "period at 100 Hz.")
    ap.add_argument("--no-daemons", action="store_true")
    ap.add_argument("--no-detect", action="store_true",
                    help="disable YOLO26n object detection (yolo_trt.py). Costs "
                         "~2.8 ms per supercombo step at 5 Hz. Display-only in "
                         "stage 2 -- nothing downstream consumes the boxes.")
    ap.add_argument("--no-scene", action="store_true",
                    help="disable the /scene.mjpg 3D render (tesla_view.py). It is "
                         "display-only and costs ~33 ms of CPU at 8 Hz, but that CPU "
                         "is shared with the model threads -- turn it off if "
                         "selfdrivedLagging appears.")
    ap.add_argument("--no-depth", action="store_true",
                    help="disable the Depth Anything V2 pane (depth_trt.py). It is a "
                         "FOURTH network on a GPU already running supercombo, YOLO "
                         "and the segmenter, and nothing consumes its output -- turn "
                         "it off first if anything starts lagging.")
    ap.add_argument("--depth-hz", type=float, default=2.0,
                    help="depth rate (default 2). Lowest of any network here on "
                         "purpose: it is a display pane, not a measurement.")
    ap.add_argument("--no-smooth", action="store_true",
                    help="disable temporal filtering (temporal.py): class-probability "
                         "smoothing on the segmentation, per-vertex EMA on the lane "
                         "lines and road edges, and alpha-beta tracking of detected "
                         "objects. Use it to see the raw per-frame jitter, or if the "
                         "filtering ever hides something you need to see.")
    ap.add_argument("--seg-overlay", action="store_true",
                    help="tint the raw b2 class map onto the CAMERA pane, in image "
                         "space, before any geometry. The only way to tell a bad "
                         "segmentation apart from bad placement -- use it when "
                         "evaluating road_seg, not while driving.")
    ap.add_argument("--no-voxels", action="store_true",
                    help="disable the voxel occupancy field (vertical.py); "
                         "vertical structure then falls back to road_seg's "
                         "nominal-height wall strips")
    ap.add_argument("--no-freespace", action="store_true",
                    help="disable the BEV occupancy grid and path clipping "
                         "(occupancy.py); the path then draws to full length "
                         "regardless of what is in the way")
    ap.add_argument("--no-roadseg", action="store_true",
                    help="disable the b2-vistas road-surface layer (road_seg.py): "
                         "asphalt extent, markings, crosswalks, kerbs and verge, "
                         "textured onto the 3D view's ground plane. Placed against "
                         "the model's own lane lines, same ruler as the vehicles.")
    ap.add_argument("--seg-hz", type=float, default=3.0,
                    help="road-segmentation rate (default 3). The network costs "
                         "44 ms a frame, so this is the main cost knob: the road is "
                         "ground-fixed, but at 3 Hz and 20 m/s the layer still steps "
                         "~7 m between updates. Drop it if selfdrivedLagging appears.")
    ap.add_argument("--display", action="store_true",
                    help="show the camera in a WINDOW on this board's DisplayPort "
                         "output instead of serving it over HTTP. No web server is "
                         "started and no JPEG is encoded. Needs an X display; if "
                         "DISPLAY is unset (e.g. over SSH) it defaults to :0.")
    ap.add_argument("--display-view",
                    choices=("pip", "scene", "camera", "depth", "occ"), default="pip",
                    help="what --display shows: 'pip' camera with the 3D scene inset "
                         "bottom-right, the depth map bottom-left and the free-space "
                         "grid top-right (default), 'scene' the 3D view full-window, "
                         "'depth' the depth map full-window, 'occ' the top-down "
                         "occupancy grid full-window, 'camera' the old camera-only "
                         "behaviour.")
    ap.add_argument("--display-fps", type=float, default=30.0,
                    help="window refresh rate with --display (default 30). The "
                         "HTTP preview ran at ~6 because it JPEG-encoded every "
                         "frame; a local window does not, so it can keep up.")
    ap.add_argument("--arm", action="store_true",
                    help="REAL TRANSMIT: switch panda to Mazda safety and stream "
                         "0x243 at 70 Hz (also enabled by DASHCAM_ARM=1)")
    ap.add_argument("--mads", action="store_true",
                    help="engage LATERAL on cruise MAIN instead of on ACC being set "
                         "(also DASHCAM_MADS=1). Lets you drive the pedals yourself and "
                         "go slower than the ~19 mph ACC floor, which is what allows a "
                         "tight turn at all: clip_curvature caps lateral accel at "
                         "3.0 m/s^2, so 19 mph means a 24 m minimum radius. REQUIRES a "
                         "panda built with MAZDA_MADS -- without it the firmware still "
                         "gates torque on ACC engaged and nothing will actuate.")
    ap.add_argument("--alpha-long", action="store_true",
                    help="ALPHA LONGITUDINAL: openpilot commands throttle and brake "
                         "(also DASHCAM_ALPHA_LONG=1). Silences the factory radar over "
                         "UDS 0x764 and sends its 0x21b/0x21c frames in its place. "
                         "THIS DISABLES FCW, AEB AND SBS for as long as it is on, and "
                         "the cluster will show malfunctions -- that is the mechanism, "
                         "not a bug. REQUIRES the opendbc alpha-long port installed "
                         "(jetson_port/opendbc_patches/alpha_long/) and a panda built "
                         "from the matching mazda.h. Mutually exclusive with --mads.")
    ap.add_argument("--steer-angle", action="store_true",
                    help="also put the MODEL's steering-wheel angle in "
                         "CAM_LKAS.STEERING_ANGLE (also DASHCAM_STEER_ANGLE=1). OFF by "
                         "default: opendbc says newer Mazdas ignore this field, and its "
                         "terms in the reverse-engineered 0x243 checksum are only fitted "
                         "for the stock value of 0. Watch ck_ok/ck_bad/ck_angle_seen on "
                         "the dashboard first -- if ck_angle_seen stays 0 this camera "
                         "never sends a non-zero angle and the formula is unproven. "
                         "A wrong checksum invalidates the steering frame itself.")
    args = ap.parse_args()

    # --can-only implies every other "off" switch. Set them here rather than
    # checking can_only at each site: the per-feature gates below already exist
    # and are already correct, so the only thing that could drift is a NEW
    # feature that forgets to ask. Forcing the flags means it gets the answer
    # from the switch it already reads.
    # --role panda is --can-only plus a live control input.
    #
    # --can-only already removes exactly the right things -- its own help text
    # says "everything it removes was competing with the tx thread for the GIL"
    # -- so the panda role reuses that switch rather than inventing a parallel
    # set of gates that could drift from it.
    #
    # The one thing it got wrong is the "NO LATERAL CONTROL" caveat. That was
    # true when this was the only process, because --can-only also suppresses
    # the openpilot daemons and nothing was left to publish carControl. Split
    # across two processes, controlsd lives with the MODEL role and carControl
    # arrives over msgq, which the main loop already subscribes to and already
    # turns into tx_state["torque_norm"]. So the panda role transmits real
    # torque; it just does not compute it.
    #
    # Daemons stay off here on purpose: selfdrived and controlsd must be spawned
    # exactly once, and the model role owns them.
    if args.role == "panda":
        args.can_only = True
    if args.can_only:
        args.no_daemons = args.no_scene = args.no_detect = True
        args.no_depth = args.no_roadseg = args.no_smooth = True
        args.no_voxels = args.no_freespace = True
        args.display = False

    if args.display:
        # No HTTP server at all in this mode -- the output device is the monitor on
        # this board, so binding 0.0.0.0 would just be an open port nobody reads.
        sys.path.insert(0, "/home/tran/openpilot_jetson")
        from op_display import available_sink
        sink = available_sink()
        disp = os.environ.get("DISPLAY") or ":0 (default; DISPLAY was unset)"
        print("=" * 58)
        print(f"  MODE: on-screen window     sink={sink}  DISPLAY={disp}")
        print(f"  refresh {args.display_fps:g} fps, no web server, no JPEG encode")
        print("=" * 58)
        if sink is None:
            print("  no usable video sink found -- install the GStreamer base plugins")
            return
    elif args.can_only:
        # No frame is produced in this mode, so the page would serve a dead
        # image and the MJPEG loops would spin on an empty buffer forever.
        print("=" * 58)
        if args.role == "panda":
            print("  ROLE: PANDA -- owns the USB handle (stock's pandad + card)")
            print("  CAN receive + 0x243 transmit + carState publish.")
            print("  Lateral torque arrives as carControl over msgq from the")
            print("  MODEL role; start that separately or nothing will steer.")
        else:
            print("  MODE: CAN ONLY -- no camera, no model, no preview, no web server")
            print("  0x243 transmit + CAN receive + JSONL log. NO LATERAL CONTROL.")
        print("=" * 58)
    else:
        srv = ReusableHTTPServer(("0.0.0.0", args.port), Handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        ips = [i for i in os.popen("hostname -I").read().split() if ":" not in i]
        print("=" * 58)
        for ip in ips:
            print(f"  open on your phone:   http://{ip}:{args.port}/")
        print("=" * 58)
    global HOOD
    try:
        from hood import HoodMask
        HOOD = HoodMask.load()
        ok, why = HOOD.plausible()
        print("hood mask: %s (%s)%s" % (HOOD.note, why,
              "  <- DEFAULT, re-derive from a driving clip: python3 hood.py --clip '...' --save"
              if HOOD.is_default else ""))
        if not ok:
            print("  mask looks wrong (%s) -- ignoring it" % why)
            HOOD = None
    except Exception as e:
        print("hood mask unavailable, bonnet NOT masked:", e)
    SEG_OVERLAY["on"] = bool(getattr(args, "seg_overlay", False))
    if SEG_OVERLAY["on"]:
        print("seg overlay ON -- raw b2 classes tinted on the camera pane")
    if args.arm or os.environ.get("DASHCAM_ARM") == "1":
        print("MODE: ARMED -- panda -> Mazda safety, real 0x243 transmit at %g Hz."
              % (args.lkas_hz or 70.0))
        print("      (panda firmware still gates on cruise-engaged + torque/rate limits)")
        print("      0x440 CAM_LANEINFO is forwarded cam->car BY THE FIRMWARE -- this")
        print("      needs the rebuilt panda_f446 image, see jetson_port/README.md.")
        if args.steer_angle or os.environ.get("DASHCAM_STEER_ANGLE") == "1":
            print("      STEERING_ANGLE injection ON (unverified checksum path)\n")
        else:
            print("      STEERING_ANGLE injection off (model angle is telemetry only)\n")
    else:
        print("MODE: SAFE -- panda in SAFETY_SILENT, nothing transmitted.")
        print("      pass --arm (or DASHCAM_ARM=1) to do a real transmit.\n")
    # SIGTERM must run the same teardown as Ctrl-C. Python's default SIGTERM
    # action terminates the interpreter WITHOUT unwinding, so `kill <pid>` skips
    # pipeline()'s finally: block and the panda is left in whatever safety mode
    # it was armed into.
    #
    # On a stock panda that is survivable -- the firmware's 1 Hz tick reverts to
    # SAFETY_SILENT after 2-5 s without a 0xf3 heartbeat. On THIS board that net
    # does not exist: the TIM9 tick ISR never fires (health.uptime stays 0
    # forever), so heartbeat_counter never increments and the watchdog at
    # board/main.c:221 is dead code. Verified 2026-07-30 -- a `kill` left the
    # panda in SAFETY_MAZDA indefinitely with no host process alive.
    #
    # So this handler is not a convenience; it is the only thing that returns the
    # panda to SILENT on a signal. Re-raising as KeyboardInterrupt reuses the
    # teardown that is already known to work rather than duplicating it.
    import signal as _signal

    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt

    try:
        _signal.signal(_signal.SIGTERM, _on_sigterm)
        _signal.signal(_signal.SIGHUP, _on_sigterm)
    except Exception as e:                       # not the main thread / no SIGHUP
        print("could not install signal handlers:", e)

    try:
        pipeline(args)
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
