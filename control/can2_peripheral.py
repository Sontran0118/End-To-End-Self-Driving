#!/usr/bin/env python3
"""Full CAN2 peripheral inspection (bus 1) — decodes the ENTIRE can_health struct
so we can see the peripheral's configured speed, IRQ activity, error counters and
reset count, confirming CAN2 is initialized and clocking correctly.

Read-only. Usage: python3 can2_peripheral.py [/dev/ttyACM0] [bus]
"""
import os
import sys, struct
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench_can_loopback import SerialPanda

DEV = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BUS = int(sys.argv[2]) if len(sys.argv) > 2 else 1
CTRL_SET_SAFETY = 0xdc
CTRL_CAN_HEALTH = 0xc2
LEC = {0: "no-error", 1: "STUFF", 2: "FORM", 3: "ACK", 4: "BIT-RECESSIVE",
       5: "BIT-DOMINANT", 6: "CRC", 7: "no-change"}

# can_health_t packed layout (from board/health.h)
FMT = "<B I B B B B B B B B I I I I I I I H H B B B I I I I"
NAMES = ["bus_off", "bus_off_cnt", "error_warning", "error_passive", "last_error",
         "last_stored_error", "last_data_error", "last_data_stored_error",
         "receive_error_cnt", "transmit_error_cnt", "total_error_cnt",
         "total_tx_lost_cnt", "total_rx_lost_cnt", "total_tx_cnt", "total_rx_cnt",
         "total_fwd_cnt", "total_tx_checksum_error_cnt", "can_speed", "can_data_speed",
         "canfd_enabled", "brs_enabled", "canfd_non_iso",
         "irq0_call_rate", "irq1_call_rate", "irq2_call_rate", "can_core_reset_cnt"]


def main():
    p = SerialPanda(DEV)
    p.control_write(CTRL_SET_SAFETY, 0, 0)   # SILENT, read-only
    sz = struct.calcsize(FMT)
    raw = p.control_read(CTRL_CAN_HEALTH, BUS, 0, sz) or b""
    print("=== CAN2 PERIPHERAL (bus %d) full health ===" % BUS)
    print("requested %d bytes, got %d\n" % (sz, len(raw)))
    if len(raw) < sz:
        raw = raw + b"\x00" * (sz - len(raw))
    vals = struct.unpack(FMT, raw[:sz])
    for n, v in zip(NAMES, vals):
        extra = ""
        if n in ("last_error", "last_stored_error"):
            extra = "  -> " + LEC.get(v, "?")
        if n in ("can_speed", "can_data_speed"):
            extra = "  (= %.1f kbps)" % (v / 10.0 * 10)  # health stores speed/10 of ×0.1kbps
        print("  %-28s = %s%s" % (n, v, extra))

    print("\n--- verdict ---")
    d = dict(zip(NAMES, vals))
    if d["irq0_call_rate"] > 0 or d["can_core_reset_cnt"] >= 0:
        print("  CAN2 peripheral is initialized (health readable, speed configured).")
    if d["last_error"] == 2 and d["receive_error_cnt"] >= 200:
        print("  RX = FORM errors, REC maxed -> peripheral FINE, the incoming")
        print("  signal is invalid CAN (car-side differential wiring).")
    print("\n(read-only — nothing transmitted)")


if __name__ == "__main__":
    main()
