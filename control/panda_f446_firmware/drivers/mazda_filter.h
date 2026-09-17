#pragma once

// ---- Host-visible ID gate (Jetson port) ----
//
// The hardware acceptance filters accept EVERYTHING on both buses, because the
// filter sits upstream of can_rx(), and can_rx() is where safety_fwd_hook() does
// the cam<->car forwarding. Anything dropped in silicon is dropped from the
// FORWARD path too -- see the long note in stm32f446/llbxcan.h.
//
// The reason a filter existed at all is the host link, not the buses: the CX-5
// main bus carries ~743 distinct IDs at ~2-3k frames/s, which saturates the
// 1.5 Mbaud serial VCP to the Jetson and overflowed can_rx_q by 1.29M frames.
// So the trim happens here instead -- after forwarding, after the safety hooks,
// immediately before the can_rx_q push.
//
// Bus 0 (car main bus) is gated to the list below. Bus 2 is the isolated camera
// segment -- 11 IDs, a few hundred frames/s -- and reaches the host whole.
//
// The list is exactly what opendbc's Mazda port reads: carstate.py's parser
// signals plus the safety layer's five rx_checks (0x21C, 0x09D, 0x240, 0x202,
// 0x165). Adding an ID here costs host bandwidth only; it has no effect on what
// the car or the camera receive.
//
// The last two are for alpha longitudinal (see opendbc_patches/*_mazda_alpha_long).
// Both are inert until that build is enabled -- they only widen what the host is
// allowed to SEE, never what it may send; the tx side is the safety model's job.
#define MAZDA_HOST_IDS_LEN 19U

static const uint16_t MAZDA_HOST_IDS[MAZDA_HOST_IDS_LEN] = {
  0x078U, // BRAKE
  0x082U, // STEER
  0x09AU, // BLINK_INFO
  0x09DU, // CRZ_BTNS        (safety rx_check)
  0x165U, // PEDALS          (safety rx_check)
  0x202U, // ENGINE_DATA     (safety rx_check, and the CAN ignition source)
  0x215U, // WHEEL_SPEEDS
  0x21BU, // CRZ_INFO        (alpha long: the radar's accel command, and ours)
  0x21CU, // CRZ_CTRL        (safety rx_check, drives controls_allowed)
  0x21FU, // CRZ_EVENTS
  0x228U, // GEAR
  0x240U, // STEER_TORQUE    (safety rx_check, driver torque for the rate limit)
  0x241U, // STEER_RATE
  0x243U, // CAM_LKAS        (our own tx echo on bus 0; the camera's copy on bus 2)
  0x340U, // SEATBELT
  0x43EU, // DOORS
  0x440U, // CAM_LANEINFO
  0x477U, // BSM
  0x76CU, // radar UDS RESPONSE (0x764 + 0x8)
  // REMOVED 2026-08-09: the seven RADAR_* IDs (0x361-0x366, 0x499) added for the
  // alpha-long dash-fault capture. They were justified as "low rate, so the
  // host-feed budget is unaffected", and that was measured wrong -- the budget
  // was already at its limit, so any addition pushed it over.
  //
  // MEASURED over one 990 s drive with them present: rx_buffer_overflow 446,577
  // and resync_bytes 7,217, ending with the host feed starved to 1 frame/s while
  // CAN1's hardware total_rx_cnt kept climbing past 2.8M. carState froze (v pinned
  // at 41.0 kph, steering angle decoding as -1369.6 deg from a torn frame) for
  // 368 s, which read as "cruise MAIN off" / wrongCarMode and made openpilot
  // impossible to re-engage after a brake.
  //
  // Nothing consumes them: the radar-shadow replay they were captured for was
  // never driven, and they are irrelevant to a MADS run. Put them back ONLY for
  // an alpha-long capture session, and expect to trim elsewhere to pay for them.
};

// Bus 2 (camera segment) host feed. The host decodes exactly TWO camera frames
// -- CAM_LKAS 0x243 (its LKAS state bits are copied into our own 0x243, and the
// checksum validator compares against it) and CAM_LANEINFO 0x440 (lane age).
// Everything else the camera sends was being pushed into can_rx_q and then
// thrown away by the host, which is pure queue pressure.
//
// MEASURED 2026-08-08: rx_buffer_overflow reached 1,786,680 with the queue
// (CAN_RX_BUFFER_SIZE 2048) filling in ~0.34 s at ~6000 frames/s. Once it is
// permanently full the USB stream degrades and reception stalls silently, taking
// carState with it -- v/rpm/angle frozen for minutes and openpilot unable to
// engage, with nothing logged.
//
// This gates ONLY the host feed, exactly like mazda_host_visible does for bus 0.
// safety_fwd_hook runs EARLIER in can_rx(), so cam->car forwarding still carries
// the camera's full message set -- which matters, because with no harness relay
// the panda is the only path between them.
static bool mazda_cam_host_visible(uint32_t addr) {
  return (addr == 0x243U) || (addr == 0x440U);
}

static bool mazda_host_visible(uint32_t addr) {
  bool visible = false;
  for (uint8_t i = 0U; i < MAZDA_HOST_IDS_LEN; i++) {
    if ((uint32_t)MAZDA_HOST_IDS[i] == addr) {
      visible = true;
      break;
    }
  }
  return visible;
}
