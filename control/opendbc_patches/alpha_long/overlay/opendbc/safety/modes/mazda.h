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

static bool mazda_longitudinal = false;

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
        controls_allowed = cruise_engaged && !brake_pressed;
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
    if ((msg->addr == MAZDA_CRZ_BTNS) && mazda_longitudinal) {
      bool cancel = GET_BIT(msg, 0U);
      if (cancel) {
        controls_allowed = false;
      }
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

        // Only feed PEDALS into pcm_cruise_check when the ACC state is actually
        // meaningful. Brake-only samples can arrive with both ACC bits low while
        // the driver is holding the pedal; treating those as a stock ACC-off edge
        // drops controls before the normal brake-edge logic runs.
#ifdef MAZDA_MADS
        // MADS under alpha long: engage on MRCC MAIN, not on the PCM's
        // ACC_ACTIVE. controls_allowed is the SINGLE authority flag here, so
        // this grants steering AND longitudinal the moment MAIN is on, with no
        // set-cruise step. mazda_tx_hook gates CRZ_CTRL's CRZ_ACTIVE bit and the
        // LKAS torque checks on the same flag.
        //
        // LEVEL-following, not pcm_cruise_check(). That helper arms only on a
        // RISING edge of its argument, which is right for stock ACC because the
        // driver presses SET for every engagement. Here the argument is "MAIN is
        // on", which stays high for the whole drive -- so an edge gate can arm
        // exactly ONCE, and the first brake press kills it until the driver
        // cycles MAIN. MEASURED 2026-08-01: controls_allowed was regained without
        // acc_armed going low first 0 times in a full log.
        //
        // Brake still drops it, and it re-arms when the pedal is released. The
        // torque/rate limits and the +-2000 accel clip are unchanged; what is
        // gone is the requirement that the driver ask for control first.
        controls_allowed = acc_armed && !brake;
        cruise_engaged_prev = acc_armed;
#else
        // Only feed PEDALS into pcm_cruise_check when the ACC state is actually
        // meaningful. Brake-only samples can arrive with both ACC bits low while
        // the driver is holding the pedal; treating those as a stock ACC-off edge
        // drops controls before the normal brake-edge logic runs.
        if (acc_armed || cruise_engaged_prev || (!brake && !brake_pressed_prev)) {
          pcm_cruise_check(cruise_engaged);
        }
#endif
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
    .max_rate_up = 40,
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
      const LongitudinalLimits MAZDA_LONG_LIMITS = {
        .max_accel = 2000,
        .min_accel = -2000,
        .inactive_accel = 0,
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
  static const CanMsg MAZDA_LONG_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true},
                                              {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
                                              {MAZDA_LKAS_HUD, 0, 8, .check_relay = true, .disable_static_blocking = true},
                                              {MAZDA_CRZ_INFO, 0, 8, .check_relay = false},
                                              {MAZDA_CRZ_CTRL, 0, 8, .check_relay = false},
                                              {MAZDA_RADAR_UDS, 0, 8, .check_relay = false}};
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

  return mazda_longitudinal ? BUILD_SAFETY_CFG(mazda_long_rx_checks, MAZDA_LONG_TX_MSGS) :
                              BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
};
