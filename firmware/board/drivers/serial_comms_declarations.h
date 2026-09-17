#pragma once

// Framing counters for the UART transport, readable over control endpoint 0xd8.
// Split out of serial_comms.h because main_comms.h is included first and needs
// the type to serve the endpoint, while serial_comms.h needs main_comms.h's
// comms_* dispatch -- so the definition cannot live in either one alone.
//
// Without these a desync is invisible from the host: a transaction just fails,
// with no way to tell a rejected header from a late payload from a lapped ring.
typedef struct __attribute__((packed)) {
  uint32_t txn_ok;         // completed transactions
  uint32_t hdr_resync;     // candidate header failed its checksum -> realigned by 1 byte
  uint32_t hdr_timeout;    // SYNC seen, rest of the header never arrived
  uint32_t mosi_timeout;   // header accepted, payload never arrived
  uint32_t mosi_checksum;  // payload arrived corrupt
  uint32_t mosi_oversize;  // host asked to send more than ser_rx holds
  uint32_t overrun;        // DMA ring lapped us: bytes genuinely lost
  uint32_t rx_late;        // byte read before the DMA's write reached SRAM
  uint8_t  last_bad_hdr[8];// the 7 bytes of the most recently rejected header
  uint8_t  reread_hdr[8];  // the same ring positions, re-read a moment later
} serial_stats_t;

extern serial_stats_t serial_stats;
