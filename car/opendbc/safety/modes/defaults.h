#pragma once

#include "opendbc/safety/safety_declarations.h"

// GCOV_EXCL_START
// Unreachable by design (doesn't define any rx msgs)
void default_rx_hook(const CANPacket_t *msg) {
  UNUSED(msg);
}
// GCOV_EXCL_STOP

// *** no output safety mode ***

static safety_config nooutput_init(uint16_t param) {
  UNUSED(param);
#ifdef PANDA_NUCLEO
  /* SOFTWARE RELAY (Jetson port). This build has NO harness relay --
   * nucleo_harness_config defines none -- so the panda is the ONLY path between
   * the forward camera on bus 2 and the car on bus 0. A stock panda survives an
   * idle/crashed openpilot because its relay re-bridges cam<->car; here, nothing
   * does, and the car raises a front camera sensor fault within seconds.
   *
   * disable_forwarding=false makes SAFETY_NOOUTPUT behave like a CLOSED relay:
   *   - tx_msgs is still NULL/0 and nooutput_tx_hook always returns false, so the
   *     HOST can transmit nothing. This is not a weakening of the output guard.
   *   - forwarding is restored, so the camera's frames reach the car and the
   *     car's reach the camera, exactly as they would with the relay closed.
   *   - the check_relay blocking loop in safety_fwd_hook iterates an EMPTY tx
   *     table, so nothing is filtered: full passthrough, including the camera's
   *     own 0x243. That is correct when we are not intercepting -- it is the
   *     stock camera path.
   *
   * Use SAFETY_NOOUTPUT, NOT SAFETY_SILENT, for the idle state: main.c puts
   * SILENT into can_silent=ALL_CAN_SILENT, i.e. hardware bus-monitoring, where
   * the core physically cannot ACK or transmit and no forwarding is possible
   * whatever this flag says. NOOUTPUT uses ALL_CAN_LIVE.
   *
   * The ACK matters as much as the forwarding: the panda is the camera's only
   * bus partner on that segment, and left unacked the camera retransmits
   * CAM_EMPTY at line rate (measured 3656 Hz), climbs to error-passive, and
   * eventually latches a fault that needs a full power-down to clear.
   */
  return (safety_config){NULL, 0, NULL, 0, false}; // NOLINT(readability/braces)
#else
  return (safety_config){NULL, 0, NULL, 0, true}; // NOLINT(readability/braces)
#endif
}

// GCOV_EXCL_START
// Unreachable by design (doesn't define any tx msgs)
static bool nooutput_tx_hook(const CANPacket_t *msg) {
  UNUSED(msg);
  return false;
}
// GCOV_EXCL_STOP

const safety_hooks nooutput_hooks = {
  .init = nooutput_init,
  .rx = default_rx_hook,
  .tx = nooutput_tx_hook,
};

// *** all output safety mode ***
static safety_config alloutput_init(uint16_t param) {
  // Enables passthrough mode where relay is open and bus 0 gets forwarded to bus 2 and vice versa
  const uint16_t ALLOUTPUT_PARAM_PASSTHROUGH = 1;
  controls_allowed = true;
  bool alloutput_passthrough = GET_FLAG(param, ALLOUTPUT_PARAM_PASSTHROUGH);
  return (safety_config){NULL, 0, NULL, 0, !alloutput_passthrough}; // NOLINT(readability/braces)
}

static bool alloutput_tx_hook(const CANPacket_t *msg) {
  UNUSED(msg);
  return true;
}

const safety_hooks alloutput_hooks = {
  .init = alloutput_init,
  .rx = default_rx_hook,
  .tx = alloutput_tx_hook,
};
