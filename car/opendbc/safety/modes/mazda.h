#pragma once

#include "opendbc/safety/safety_declarations.h"

// ============================================================================
// MAZDA SAFETY MODEL -- Jetson port, with ALPHA LONGITUDINAL
// ============================================================================
// Drop-in replacement for opendbc_src/opendbc/safety/modes/mazda.h @ 8ddffb37.
//
// This file is a three-way merge of:
//   1. stock opendbc @ 8ddffb37
//   2. this port's existing changes -- PANDA_NUCLEO tx table, MAZDA_MADS
//      engagement gate, and the measured MAZDA_STEERING_LIMITS. All preserved
//      verbatim, comments included.
//   3. yummydirtx/opendbc:mazda-longitudinal-upstream (community.sunnypilot.ai
//      thread 1482), which adds tx of the radar's own longitudinal frames.
//
// WHAT ALPHA LONG DOES, in one paragraph: the factory radar at UDS 0x764 is the
// ECU that commands longitudinal. Put it in a programming session and hold that
// session open with tester-present, and it stops transmitting -- at which point
// 0x21b CRZ_INFO (which carries ACCEL_CMD) and 0x21c CRZ_CTRL are ours to send,
// and the PCM executes them. Positive ACCEL_CMD accelerates, negative brakes.
//
// WHAT IT COSTS: the radar is the same ECU that runs FCW, AEB and SBS. While it
// is suppressed the car has NONE of them, and the dash will show malfunctions
// saying so. That is not a bug in this file, it is the mechanism. Low-speed
// camera-based SCBS survives; radar AEB at speed does not.
//
// ---------------------------------------------------------------------------
// MADS INTERACTION -- READ THIS
// ---------------------------------------------------------------------------
// MAZDA_MADS widens controls_allowed to "cruise MAIN is on" by gating on
// CRZ_AVAILABLE inside the CRZ_CTRL handler. Alpha long SUPPRESSES CRZ_CTRL --
// the radar no longer sends it and we synthesise it instead -- so that handler
// is skipped entirely when mazda_longitudinal is set, and engagement falls back
// to pcm_cruise_check() driven by PEDALS.ACC_ACTIVE.
//
// The two features are therefore MUTUALLY EXCLUSIVE, by construction rather
// than by a check. Building with both leaves MADS inert. This is deliberate and
// it is the safe direction: alpha long adds throttle and brake authority to
// whatever controls_allowed means, and stacking that on the permissive MAIN-on
// gate would mean the panda passes acceleration commands whenever MAIN is on --
// and MAIN tends to be left on. If you ever wire MADS back in under long, gate
// the 0x21b/0x21c tx on a strict ACC-engaged flag, not on controls_allowed.
// ============================================================================

// CAN msgs we care about
#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_INFO      0x21bU   // alpha long: ACCEL_CMD, normally the radar's
#define MAZDA_CRZ_CTRL      0x21cU
// The radar's OTHER seven frames, replayed by the host while it is suppressed so
// the forward camera does not see its partner vanish. No checksums; 0x361-0x365
// carry a 4-bit counter in the low nibble of byte 7, 0x366 and 0x499 are static.
#define MAZDA_RADAR_DISTANCE 0x361U
#define MAZDA_RADAR_TURN     0x362U
#define MAZDA_RADAR_363      0x363U
#define MAZDA_RADAR_364      0x364U
#define MAZDA_RADAR_365      0x365U
#define MAZDA_RADAR_366      0x366U
#define MAZDA_RADAR_499      0x499U
#define MAZDA_CRZ_BTNS      0x09dU
#define MAZDA_RADAR_UDS     0x764U   // alpha long: radar diagnostic address
#define MAZDA_STEER_TORQUE  0x240U
#define MAZDA_ENGINE_DATA   0x202U
#define MAZDA_PEDALS        0x165U

// CAN bus numbers
#define MAZDA_MAIN 0
#define MAZDA_CAM  2

enum {
  MAZDA_PARAM_LONGITUDINAL = 1,
};

// CRZ_INFO.ACCEL_CMD value the factory radar sends when it is NOT commanding.
// DBC 17|13@0+ (1,-4096), so raw 8190 decodes to +4094. Verified against this
// car 2026-08-12: the radar's idle frame is 01 ff e3 ff c0 80 08 d5.
// MUST equal what build_crz_info() emits for the not-engaged case in
// opendbc/car/mazda/longitudinal.py, or the panda rejects every idle frame.
#define MAZDA_ACCEL_IDLE_SENTINEL 4094

static bool mazda_longitudinal = false;

// Previous CRZ_BTNS state, for the alpha-long button arming in mazda_rx_hook.
// Reset in mazda_init so a safety-mode change cannot leave a stale edge behind.
static bool mazda_res_prev = false;
static bool mazda_set_m_prev = false;

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void mazda_rx_hook(const CANPacket_t *msg) {
  if ((int)msg->bus == MAZDA_MAIN) {
    if (msg->addr == MAZDA_ENGINE_DATA) {
      // sample speed: scale by 0.01 to get kph
      int speed = (msg->data[2] << 8) | msg->data[3];
      vehicle_moving = speed > 10; // moving when speed > 0.1 kph
    }

    if (msg->addr == MAZDA_STEER_TORQUE) {
      int torque_driver_new = msg->data[0] - 127U;
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if (msg->addr == MAZDA_CRZ_CTRL) {
      // Under alpha long this frame is OURS, not the radar's -- see the MADS
      // note at the top. Reading engagement out of a frame we synthesise would
      // be a loop, so the PEDALS branch below owns it instead.
      if (!mazda_longitudinal) {
#ifdef MAZDA_MADS
        // MADS build: gate on CRZ_AVAILABLE (cruise MAIN) instead of CRZ_ACTIVE
        // (ACC actually set). DBC: CRZ_AVAILABLE is 17|1@0+ -> byte 2 bit 1;
        // CRZ_ACTIVE is 3|1@0+ -> byte 0 bit 3.
        //
        // WHY: stock Mazda ACC refuses to set below ~19 mph, and clip_curvature
        // caps lateral accel at 3.0 m/s^2 -- so max curvature is 3.0/v^2 and
        // 19 mph means a 24 m minimum turn radius. No amount of torque buys a
        // tighter turn at that speed; only going slower does, and that requires
        // engaging lateral without ACC holding the speed up.
        //
        // WHAT THIS COSTS: controls_allowed becomes true whenever the cruise MAIN
        // button is on, not when the driver deliberately engages. That is a
        // materially more permissive gate than stock openpilot -- the panda will
        // pass steering torque in situations where it previously would not, and
        // MAIN tends to be left on. The driver-torque override (torque_driver /
        // driver_torque_allowance) and max_torque are the only limits left in
        // front of the rack. Build this deliberately or not at all.
        bool cruise_engaged = msg->data[2] & 0x2U;
        // LEVEL-following, same reason as the alpha-long PEDALS branch below:
        // pcm_cruise_check() arms only on a RISING edge, and under MADS the
        // argument is "MAIN is on", which stays high for the whole drive. An
        // edge gate therefore arms exactly ONCE and the first brake press kills
        // it until the driver cycles MAIN. Measured on the alpha path 2026-08-01:
        // 0 recoveries in a full log. Brake drops it, release re-arms it.
#ifdef MAZDA_MADS_BRAKE
        // BRAKE NO LONGER DROPS LATERAL. Read this before building it.
        //
        // WHY: openpilot's turnLeft/turnRight desires are only issued while
        // lateral is active, and the driver brakes to take a turn -- so with the
        // brake term in place the desire is suppressed at exactly the moment it
        // is wanted, and the car cannot turn under MADS at all. The same term is
        // what made lateral drop on every slowdown: MEASURED over one 1583 s
        // drive, all TWELVE transitions to disabled had brake=1, and nothing else
        // disengaged the stack for the whole drive.
        //
        // WHAT IT COSTS, stated plainly: the panda passes steering torque while
        // the brake is down, whenever cruise MAIN is on. Stacked on the MADS gate
        // above -- which is already "MAIN is on" rather than "the driver engaged"
        // -- this is the most permissive lateral gate in this file. After it, the
        // only things between the model and the rack are the driver-torque
        // override (torque_driver / driver_torque_allowance) and max_torque.
        //
        // Longitudinal authority is NOT widened: this branch only runs when
        // mazda_longitudinal is unset, and the alpha-long PEDALS branch below
        // keeps its own `&& !brake` untouched. Braking still cancels the car's
        // ACC through the car's own logic; what changes is only whether the panda
        // will pass STEERING while it happens.
        //
        // Deliberately a separate build flag rather than folded into MAZDA_MADS,
        // so reverting is a rebuild-and-flash with no source edit, and so nobody
        // gets this behaviour by asking for MADS.
        controls_allowed = cruise_engaged;
#else
        controls_allowed = cruise_engaged && !brake_pressed;
#endif
        cruise_engaged_prev = cruise_engaged;
#else
        bool cruise_engaged = msg->data[0] & 0x8U;
        pcm_cruise_check(cruise_engaged);
#endif
        acc_main_on = GET_BIT(msg, 17U);
      }
    }

    // Alpha long: the physical CANCEL button is the driver's direct kill for
    // longitudinal. Stock reaches this through the radar's CRZ_CTRL, which is
    // suppressed, so read the button itself.
    // Alpha long: the driver's cruise buttons are the ONLY engagement authority.
    //
    // Stock reaches engagement through the radar's CRZ_CTRL and the PCM's
    // ACC_ACTIVE, both of which alpha long removes -- the radar is suppressed and
    // ACC_ACTIVE then never rises, so pcm_cruise_check() below could only ever
    // CLEAR controls_allowed, never set it. MEASURED 2026-08-12: 14 clean SET
    // presses on this frame with acc_active 0 throughout. The press is here and
    // readable; nothing was acting on it.
    //
    // So arm on the button, the way hyundai_common_cruise_buttons_check does.
    // One deliberate difference: that helper arms on the falling edge of BOTH
    // set and resume, while this mirrors the host's own
    // MazdaCarState.update_button_enable() exactly -- RESUME on press, SET on
    // release. Panda and host must arm on the SAME event, or there is a window
    // where openpilot believes it is engaged and every CRZ_CTRL frame is dropped
    // here, which reads as a car ignoring us rather than a gate that has not
    // opened yet.
    //
    // CRZ_BTNS bits are all in byte 0 and GET_BIT uses the DBC's own numbering:
    //   CAN_OFF 0, RES 2, SET_P 4, SET_M 5.
    if ((msg->addr == MAZDA_CRZ_BTNS) && mazda_longitudinal) {
      const bool cancel = GET_BIT(msg, 0U);
      const bool res = GET_BIT(msg, 2U);
      const bool set_m = GET_BIT(msg, 5U);

      if ((res && !mazda_res_prev) || (!set_m && mazda_set_m_prev)) {
        controls_allowed = true;
      }
      // Checked AFTER the arm so a cancel in the same frame always wins.
      if (cancel) {
        controls_allowed = false;
      }

      mazda_res_prev = res;
      mazda_set_m_prev = set_m;
    }

    if (msg->addr == MAZDA_ENGINE_DATA) {
      gas_pressed = (msg->data[4] || (msg->data[5] & 0xF0U));
    }

    if (msg->addr == MAZDA_PEDALS) {
      bool brake = (msg->data[0] & 0x10U);
      if (mazda_longitudinal) {
        // Radar suppression removes the stock CRZ_CTRL frame, so derive Mazda's
        // "main on" state from PEDALS instead. ACC_OFF means MRCC is armed but
        // not actively controlling, and ACC_ACTIVE means stock ACC is engaged.
        bool cruise_engaged = GET_BIT(msg, 3U);
        bool acc_armed = GET_BIT(msg, 2U) || cruise_engaged;
        acc_main_on = acc_armed;

        // THE MADS LEVEL-GATE USED TO LIVE HERE AND HAS BEEN REMOVED.
        //
        // It was `controls_allowed = acc_armed && !brake;` under #ifdef
        // MAZDA_MADS -- a LEVEL assignment, executed on every PEDALS frame at
        // 50 Hz. Under alpha long that is the whole authority flag for THROTTLE
        // AND BRAKE, granted by the MRCC MAIN switch alone with no set-cruise
        // step, and MAIN tends to be left on.
        //
        // It also silently defeated the button arming in the CRZ_BTNS branch
        // above: a SET release would set controls_allowed, and the next PEDALS
        // frame 20 ms later would overwrite it with the level value regardless.
        // MEASURED ON THE CAR 2026-08-12, first run of the flashed build:
        // ctrl_allowed was already 1 with btn +0/-0/R0/O0, before any button had
        // been touched. The buttons were inert.
        //
        // This is the combination the port already refuses everywhere else:
        // opendbc_patches/alpha_long/README.md calls MADS and alpha long
        // mutually exclusive, and dashcam_web.py rejects `--alpha-long --mads`
        // for exactly this reason -- "stacking throttle authority on the
        // permissive MAIN-on gate would be a bad combination". The firmware was
        // the one place that did it anyway, because MAZDA_MADS is a compile-time
        // define and the host flag cannot reach it.
        //
        // MADS itself is NOT removed. Its lateral gate is the CRZ_CTRL branch
        // above, which is `!mazda_longitudinal` and untouched, so a MADS build
        // without alpha long behaves exactly as before.
        //
        // NOTE FOR TESTING: libsafety must be built with the SAME defines as the
        // firmware (-DPANDA_NUCLEO -DMAZDA_FILTER -DMAZDA_MADS -DMAZDA_MADS_BRAKE,
        // see panda_f446/SConscript) or the bench tests a variant that is not
        // what is flashed. That is precisely how this bug reached the car.
        //
        // NO pcm_cruise_check() HERE. It used to run on this branch, fed with
        // PEDALS.ACC_ACTIVE, and it has to go now that the buttons arm instead.
        //
        // pcm_cruise_check() clears controls_allowed on every sample where its
        // argument is false, and sets it only on a RISING edge. Under alpha long
        // ACC_ACTIVE never rises -- that is the whole finding of 2026-08-12 -- so
        // leaving this in means the PEDALS frame at 50 Hz would clear the arm the
        // button just granted, within 20 ms, forever. It would look exactly like
        // "the panda still blocks us" rather than "our own gate is fighting
        // itself", which is a failure mode this port has already been bitten by.
        //
        // Losing it costs nothing: the driver's ways out are all still live --
        // CANCEL clears in the CRZ_BTNS branch above, brake clears generically in
        // safety.h (`brake_pressed && (!brake_pressed_prev || vehicle_moving)`),
        // and MAIN-off clears immediately below.
        //
        // MAIN off is a disengage. Once the buttons can arm, an armed state would
        // otherwise survive the driver switching MRCC off entirely, since nothing
        // else in this branch looks at acc_armed.
        if (!acc_armed) {
          controls_allowed = false;
        }
        cruise_engaged_prev = cruise_engaged;
      }
      brake_pressed = brake;
    }
  }
}

static bool mazda_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits MAZDA_STEERING_LIMITS = {
    // RAISED from the upstream 800 (Jetson port). The LKAS_REQUEST field is
    // 12-bit with a -2048 offset, so the wire allows +-2047; 800 was a
    // conservative fleet default, not a rack limit. This MUST stay equal to
    // CarControllerParams.STEER_MAX in opendbc/car/mazda/values.py: if the
    // sender's ceiling is higher than this one, the frame is rejected here,
    // the firmware zeroes desired_torque_last, and every following frame then
    // fails the rate check too -- 0x243 stops reaching the bus entirely and
    // the EPS raises the front LKAS fault. Loud failure, not a soft clamp.
    // Tried 2047 (the wire maximum) on 2026-08-01 and reverted: STEER_MAX and
    // latAccelFactor scale together, so commanded counts were unchanged and the
    // only effect was to stop clipping the limit cycle. See values.py.
    // 1400 -> 2047 (second attempt). The first revert argued STEER_MAX and
    // latAccelFactor cancel -- true below the rail, false AT it: steer_max is
    // 1.0 normalised, so the ceiling in counts IS STEER_MAX. The controller
    // saturates 26% of the time, so the 46% extra headroom is reachable.
    // MUST stay equal to CarControllerParams.STEER_MAX in values.py.
    .max_torque = 2047,
    // RAISED from 10. Per MESSAGE, not per second: the achievable ramp is
    // tx_rate * max_rate_up, so 100 Hz * 15 = 1500 counts/s (stock openpilot is
    // 100 * 10 = 1000). MUST stay equal to CarControllerParams.STEER_DELTA_UP in
    // opendbc/car/mazda/values.py -- if the sender ramps faster than this the
    // frame is rejected, desired_torque_last is zeroed here, and every following
    // frame fails the rate check too. Same total-failure mode as max_torque.
    // 10 -> 15 -> 30. At 50 Hz that is 1500 counts/s, so 0 -> max_torque (1400)
    // in 0.93 s against 1.87 s at 15. Per-window need is 12.5 * 30 = 375, well
    // under max_rt_delta (1400).
    //
    // MEASURED CONTEXT: the torque ramp was running at only 75-77% of its own
    // limit while MAX_LATERAL_JERK in drive_helpers.py was at 86%, so the ramp
    // was NOT what made turns feel slow. This is raised so it stays out of the
    // way now that the jerk limit has been doubled -- not because it was the
    // constraint. If turn response is still short of expectations after this,
    // look at the plan and the tune, not here.
    // 50/50 TRIED 2026-08-01 AND REVERTED -- the EPS refuses that ramp. Measured
    // on the same car at the same speeds: at DELTA_UP 30 the rack applied 62% of
    // request at 0-20 kph with lkas_block 3.4%; at 50 it applied ZERO with
    // lkas_block 100%, while eps_request still tracked our frames 1:1. The rack
    // receives the command and declines it. 2100 counts/s is accepted, 3500 is
    // not. MUST stay equal to CarControllerParams.STEER_DELTA_UP / _DOWN.
    // RETRY of 50/50 (second attempt). The first measured eff/req 0.00 with
    // lkas_block 100%, but every one of those runs had alpha long active, and
    // alpha long alone degrades EPS acceptance (0.09 / 47% blocked at 40-60 kph
    // vs 0.83 / 0.0% without it). The ramp was never isolated. Retesting with
    // alpha long OFF. MUST stay equal to STEER_DELTA_UP / _DOWN in values.py.
    // 30 -> 50 -> 40. 50 measured good with alpha long off (eff/req 0.93 at
    // 20-40 kph, 0.0% blocked); the earlier collapse at 50 was alpha long, not
    // the ramp. 40 is the middle setting: 2800 counts/s at 70 Hz.
    // max_rate_down left HIGHER than up so the system can always release at
    // least as fast as it grabs -- torque cannot ratchet across a limit cycle.
    // MUST stay equal to CarControllerParams.STEER_DELTA_UP / _DOWN.
    // 40 (2026-08-02). 10 -> 15 -> 30 -> 50 -> 40 -> 10 -> 40. Stock 10 pairs
    // with upstream's STEER_MAX 800; against 2047 it means 2.92 s to full
    // authority, so 40 (0.73 s) is the sane pairing. Measured with alpha long
    // off: eff/req 0.85-0.87 with ~0% blocked, so the rack accepts this ramp.
    // max_rate_down left at 50 (stock 25) so it always releases at least as
    // fast as it grabs. MUST stay equal to STEER_DELTA_UP / _DOWN.
    // 40 -> 80, paired with --lkas-hz 50 (2026-08-09).
    //
    // STEER_DELTA_UP is PER MESSAGE, so the achievable ramp is tx_rate * delta:
    //   100 Hz x 40 = 4000 counts/s     50 Hz x 80 = 4000 counts/s
    // i.e. this keeps the ORIGINAL ramp while halving the transmit rate.
    //
    // WHY HALVE THE RATE. MEASURED 2026-08-09 at 100 Hz with alpha long and the
    // MPC sharing this process: 1177 late frames and a worst inter-frame gap of
    // 34 ms against a 10 ms target. The EPS faults on IRREGULAR 0x243, not on a
    // lower rate -- that irregularity is what produced the intermittent front
    // camera fault. 50 Hz gives 20 ms of period, enough to absorb a can_recv
    // (~15.5 ms) and still transmit on schedule.
    //
    // Headroom: at 50 Hz a 250 ms RT window is 12.5 frames, so 12.5 * 80 = 1000
    // counts, comfortably under max_rt_delta (2047). MUST stay >= the sender's
    // CarControllerParams.STEER_DELTA_UP or every frame is rejected,
    // desired_torque_last is zeroed, and 0x243 stops reaching the bus entirely.
    .max_rate_up = 80,
    .max_rate_down = 50,
    // DIAGNOSTIC VALUE -- raised 300 -> 450 -> 900. Revert to ~450 once the
    // measurement below is done.
    //
    // This is not just a headroom number, it silently becomes a HARD TORQUE
    // CEILING whenever the tx stream is not clean. The check is
    //     violation if |desired| > MAX(rt_torque_last, 0) + max_rt_delta
    // and rt_torque_last only becomes non-zero by surviving a full 250 ms
    // window with ZERO violations -- any violation resets it to 0 (lateral.h:145,
    // in the `if (violation || !controls_allowed)` block). So under a jittery
    // stream, where late frames trip the per-message max_rate_up check
    // constantly, rt_torque_last is pinned at 0 and the panda rejects
    // everything above max_rt_delta, forever.
    //
    // That is exactly what happened on 2026-07-30: 2326 violations from a tx
    // stream with 251 ms gaps, and LKAS_REQUEST read back off 0x241 capped at a
    // flat, symmetric, speed-independent 450 -- which looked like an EPS
    // property and was actually this constant. The car never saw a single frame
    // above it, so the real rack ceiling is still unmeasured and is >= 450.
    //
    // RESULT of that experiment (2026-07-30, 50 Hz, 122 rejections): the ceiling
    // moved 450 -> 900 exactly, tracking this constant. Confirmed the panda was
    // always the clamp; the EPS accepted 1:1 up to 890 with no roll-off.
    //
    // Now set EQUAL TO max_torque. From rt_torque_last = 0 the check permits
    // 0 + 1400, which is max_torque, so this can no longer bind at all -- it is
    // deliberately neutralised so the next run measures the RACK and nothing
    // else. That does remove it as an independent backstop: max_rate_up (per
    // message) and max_torque (absolute) are the only limits left. Put it back
    // to ~2x the per-window need once the EPS ceiling is known.
    // Back to 1400 with max_torque. Headroom check for the raised ramp below:
    // at 70 Hz a 250 ms RT window is 17.5 frames, so 17.5 * 50 = 875, still
    // comfortably under this. At 100 Hz it is 25 * 50 = 1250, also under.
    // Tracks max_torque so it can never bind. At 70 Hz a 250 ms RT window is
    // 17.5 frames, so 17.5 * 50 = 875 -- far under this either way.
    .max_rt_delta = 2047,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  bool tx = true;
  // Check if msg is sent on the main BUS
  if (msg->bus == (unsigned char)MAZDA_MAIN) {
    // steer cmd checks
    if (msg->addr == MAZDA_LKAS) {
      int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

      if (steer_torque_cmd_checks(desired_torque, -1, MAZDA_STEERING_LIMITS)) {
        tx = false;
      }
    }

    if (mazda_longitudinal && (msg->addr == MAZDA_CRZ_INFO)) {
      // Keep Panda's Mazda-long safety window aligned with the software clip in
      // opendbc/car/mazda/longitudinal.py. If this is tighter than the sender,
      // Panda will silently drop 0x21b frames once ACCEL_CMD crosses the
      // safety threshold, which looks like an unexplained set-speed unlatch.
      // inactive_accel is THIS CAR'S IDLE SENTINEL, not zero.
      //
      // longitudinal_accel_checks() permits exactly inactive_accel while
      // controls_allowed is false. Upstream uses 0 for that, which assumes the
      // not-commanding frame carries a zero command. Mazda's radar does not: its
      // idle CRZ_INFO is
      //     01 ff e3 ff c0 80 08 d5      ACCEL_CMD raw 8190 -> +4094
      // i.e. a sentinel meaning "no request", nowhere near zero. Zero is a live
      // command for zero acceleration and is a DIFFERENT statement (see the
      // standby-template note in opendbc/car/mazda/longitudinal.py).
      //
      // MEASURED ON THE CAR 2026-08-12: with the standby template restored so our
      // idle frame matches the radar byte-for-byte, inactive_accel = 0 rejected
      // EVERY 0x21b -- tx_blocked climbed at exactly 50/s, the CRZ_INFO rate, for
      // the whole run. Radar suppressed and our replacement blocked at the gate
      // means the PCM receives no CRZ_INFO at all, which is strictly worse than
      // either alone.
      //
      // 4094 is outside [min_accel, max_accel] on purpose: it is not reachable as
      // a command, so permitting it while disengaged grants no actuation
      // authority. Engaged frames are still clamped to +-2000.
      const LongitudinalLimits MAZDA_LONG_LIMITS = {
        .max_accel = 2000,
        .min_accel = -2000,
        .inactive_accel = MAZDA_ACCEL_IDLE_SENTINEL,
      };

      // CRZ_INFO.ACCEL_CMD is DBC `17|13@0+ (1,-4096)`: 13 bits, big-endian,
      // starting at bit 17. That walks byte 2 bits 1..0, all of byte 3, then
      // byte 4 bits 7..5 -- which is exactly the shift pattern below. The -4096
      // matches the DBC offset, so raw 4096 is zero accel; the neutral template
      // in longitudinal.py (01ffe20006800000) decodes to precisely that.
      uint32_t accel_raw = ((((uint32_t)msg->data[2] & 0x3U) << 11U) |
                            (((uint32_t)msg->data[3]) << 3U) |
                            (((uint32_t)msg->data[4]) >> 5U));
      int desired_accel = (int)accel_raw - 4096;
      if (longitudinal_accel_checks(desired_accel, MAZDA_LONG_LIMITS)) {
        tx = false;
      }
    }

    if (mazda_longitudinal && (msg->addr == MAZDA_CRZ_CTRL)) {
      bool cruise_active = GET_BIT(msg, 3U);
      if (!controls_allowed && cruise_active) {
        tx = false;
      }
    }

    if (mazda_longitudinal && (msg->addr == MAZDA_RADAR_UDS)) {
      // The ONLY two frames allowed to the radar's diagnostic address: raw
      // tester-present (0x3E 0x80, suppress-response) and a session control
      // request for default (0x01) or programming (0x02). Everything else --
      // routine control, security access, memory writes, anything that could
      // reflash or reconfigure the ECU -- is rejected here. Suppressing an ECU
      // is reversible on the next ignition cycle; writing to one is not.
      bool tester_present = (msg->data[0] == 0x02U) && (msg->data[1] == 0x3EU) && (msg->data[2] == 0x80U);
      bool session_control = (msg->data[0] == 0x02U) && (msg->data[1] == 0x10U) &&
                             ((msg->data[2] == 0x01U) || (msg->data[2] == 0x02U));
      if (!tester_present && !session_control) {
        tx = false;
      }
    }

    // cruise buttons check
    if (msg->addr == MAZDA_CRZ_BTNS) {
      // allow resume spamming while controls allowed, but
      // only allow cancel while controls not allowed
      bool cancel_cmd = (msg->data[0] == 0x1U);
      if (!controls_allowed && !cancel_cmd) {
        tx = false;
      }
    }
  }

  return tx;
}

static safety_config mazda_init(uint16_t param) {
#ifdef PANDA_NUCLEO
  // DIY F446 panda (Jetson port). Stock openpilot replaces BOTH camera frames:
  // the CarController sends its own 0x243 every frame and its own 0x440 at 2 Hz
  // (create_alert_command), so blocking the camera's copies of both is correct
  // there. This port replaces only 0x243 -- nothing generates a 0x440 -- so with
  // the stock table the car's LKAS system receives NO lane state at all and
  // ignores the injected steering.
  //
  // disable_static_blocking lifts ONLY the cam->car forward block for 0x440 (see
  // safety_fwd_hook). check_relay stays TRUE, so:
  //   - 0x243 is still blocked cam->car: openpilot's steering frame is the only
  //     one the car ever sees. That is the whole point of the intercept.
  //   - a 0x440 (or 0x243) arriving on bus 0 from any OTHER sender still latches
  //     relay_malfunction, which is the check that catches a camera that was
  //     never electrically cut off the main bus.
  // The panda does not receive its own transmissions (bxCAN never self-receives,
  // and the TX echo is pushed straight to can_rx_q by process_can without going
  // through safety_rx_hook), so forwarding 0x440 onto bus 0 cannot self-trigger.
  static const CanMsg MAZDA_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true},
                                         {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
                                         {MAZDA_LKAS_HUD, 0, 8, .check_relay = true, .disable_static_blocking = true}};
  // Alpha long adds three senders. check_relay is FALSE on all three: relay
  // checking asserts "the frame we send must not also arrive from someone else
  // on bus 0", and the radar is a bus-0 ECU we are silencing by software, not
  // by a relay. A radar that briefly resumes transmitting 0x21b -- the exact
  // failure reported on the sunnypilot thread -- must not latch
  // relay_malfunction and kill steering along with it.
  // RADAR SHADOW. Alpha long silences the radar to take over 0x21b/0x21c, but the
  // radar also sends seven OTHER frames that we do not replace, and that other
  // modules -- the forward camera in particular -- expect to keep seeing. Their
  // disappearance is the leading explanation for the "front camera sensor" fault,
  // and it happens whichever way the radar is muted: MEASURED 2026-08-09, this
  // radar refuses CommunicationControl and both alternate diagnostic sessions, so
  // the programming session is the only route available and a gentler one does
  // not exist.
  //
  // These entries let the host replay those frames while the radar is mute. They
  // carry NO checksum -- five have a plain 4-bit counter in the low nibble of
  // byte 7, two are static -- so replay needs only counter incrementing, which is
  // why this is worth attempting at all.
  //
  // check_relay = false on every one: these are frames the radar would normally
  // put on bus 0 itself, so the relay-blocking test does not apply. They are only
  // transmittable under mazda_longitudinal, i.e. only when the radar has actually
  // been suppressed -- outside alpha long this table is not used and the panda
  // will reject them, which is the correct failure direction if the host ever
  // tries to send radar frames alongside a live radar.
  static const CanMsg MAZDA_LONG_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true},
                                              {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
                                              {MAZDA_LKAS_HUD, 0, 8, .check_relay = true, .disable_static_blocking = true},
                                              {MAZDA_CRZ_INFO, 0, 8, .check_relay = false},
                                              {MAZDA_CRZ_CTRL, 0, 8, .check_relay = false},
                                              {MAZDA_RADAR_UDS, 0, 8, .check_relay = false},
                                              {MAZDA_RADAR_DISTANCE, 0, 8, .check_relay = false},
                                              {MAZDA_RADAR_TURN, 0, 8, .check_relay = false},
                                              {MAZDA_RADAR_363, 0, 8, .check_relay = false},
                                              {MAZDA_RADAR_364, 0, 8, .check_relay = false},
                                              {MAZDA_RADAR_365, 0, 8, .check_relay = false},
                                              {MAZDA_RADAR_366, 0, 8, .check_relay = false},
                                              {MAZDA_RADAR_499, 0, 8, .check_relay = false},
                                              // BUS 2 AS WELL -- the forward camera's segment.
                                              //
                                              // The camera lives on bus 2 and normally RECEIVES the
                                              // radar's frames. Suppress the radar and replay only
                                              // onto bus 0 and the camera is left with nothing, which
                                              // is precisely the front camera sensor fault this port
                                              // has been chasing. It is also why a bus-0 census came
                                              // back clean: MEASURED 2026-08-11 with the host filter
                                              // fully open, bus 0 carries 99 ids and suppression
                                              // removes exactly nine -- every one of them already
                                              // replayed or replaced. The gap was never on bus 0.
                                              //
                                              // yummydirtx/opendbc @ go-no-malfunction sends both the
                                              // longitudinal pair and the radar heartbeat to
                                              // (RADAR_BUS, CAM_BUS); this table is what permits the
                                              // second half of that.
                                              //
                                              // check_relay stays false: on bus 2 these are frames the
                                              // radar would have reached the camera with anyway, and
                                              // the relay test is about our own 0x243 intercept.
                                              {MAZDA_CRZ_INFO, 2, 8, .check_relay = false},
                                              {MAZDA_CRZ_CTRL, 2, 8, .check_relay = false},
                                              {MAZDA_RADAR_DISTANCE, 2, 8, .check_relay = false},
                                              {MAZDA_RADAR_TURN, 2, 8, .check_relay = false},
                                              {MAZDA_RADAR_363, 2, 8, .check_relay = false},
                                              {MAZDA_RADAR_364, 2, 8, .check_relay = false},
                                              {MAZDA_RADAR_365, 2, 8, .check_relay = false},
                                              {MAZDA_RADAR_366, 2, 8, .check_relay = false},
                                              {MAZDA_RADAR_499, 2, 8, .check_relay = false}};
#else
  static const CanMsg MAZDA_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true}, {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false}, {MAZDA_LKAS_HUD, 0, 8, .check_relay = true}};
  static const CanMsg MAZDA_LONG_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true},
                                              {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
                                              {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
                                              {MAZDA_CRZ_INFO, 0, 8, .check_relay = false},
                                              {MAZDA_CRZ_CTRL, 0, 8, .check_relay = false},
                                              {MAZDA_RADAR_UDS, 0, 8, .check_relay = false}};
#endif

  static RxCheck mazda_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };
  // CRZ_CTRL is deliberately absent: under alpha long the radar stops sending it,
  // so a required-rx check on it would fail permanently the moment suppression
  // takes effect and drop the whole safety config.
  static RxCheck mazda_long_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  mazda_longitudinal = GET_FLAG(param, MAZDA_PARAM_LONGITUDINAL);
  acc_main_on = false;
  // Clear the button history too, or a stale `true` from an earlier session
  // survives the mode change and the first CRZ_BTNS frame with SET released
  // reads as a falling edge -- arming with no press from the driver at all.
  // The panda re-inits on every arm, so this is a real path, not a corner case.
  mazda_res_prev = false;
  mazda_set_m_prev = false;

  return mazda_longitudinal ? BUILD_SAFETY_CFG(mazda_long_rx_checks, MAZDA_LONG_TX_MSGS) :
                              BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
};
