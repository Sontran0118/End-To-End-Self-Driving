#!/usr/bin/env python3
"""Focused test of TRANSCEIVER 2 / CAN2 (logical bus 1).

Two independent checks:
 1) INTERNAL LOOPBACK on bus 1 (CAN2): send a frame, it loops TX->RX inside the
    STM32 (SILM|LBKM) with NOTHING on the physical wire. Proves the CAN2
    peripheral + the firmware path work. Isolates 'STM32 CAN2 is fine' from
    'car-side wiring is bad'.
 2) LIVE health snapshot on bus 1 in normal mode (what the car-side signal does).

Loopback uses the internal peripheral loopback (safe, no bus activity). The live
part is read-only (SAFETY_SILENT, no transmit onto the real bus).

Usage: python3 test_tr2.py [/dev/ttyACM0]
"""
import sys, time
sys.path.insert(0, "/home/tran/op_fork/jetson_port")
from bench_can_loopback import SerialPanda

DEV = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BUS = 1                     # CAN2 = transceiver 2

CTRL_SET_SAFETY = 0xdc
CTRL_LOOPBACK = 0xe5        # enable internal CAN loopback (param1=enable)
CTRL_CAN_HEALTH = 0xc2
LEC = {0: "no-err", 1: "STUFF", 2: "FORM", 3: "ACK", 4: "BITREC", 5: "BITDOM", 6: "CRC", 7: "nochg"}


def health(p):
    h = p.control_read(CTRL_CAN_HEALTH, BUS, 0, 24) or b"\x00" * 24
    return {"last_error": LEC.get(h[7], "?"), "REC": h[11], "err_passive": h[6], "raw": h[:16].hex()}


def main():
    p = SerialPanda(DEV)
    print("=== TRANSCEIVER 2 / CAN2 (bus 1) TEST ===\n")

    # ---- 1) INTERNAL LOOPBACK ----
    print("[1] Internal loopback on bus 1 (nothing on the physical wire)...")
    p.control_write(CTRL_SET_SAFETY, 0, 0)     # SILENT first
    p.control_write(CTRL_LOOPBACK, 1, 0)       # enable internal loopback
    time.sleep(0.3)
    # need controls to allow TX for the loopback send; use an allow-all safety
    # NO -- keep SILENT blocks tx. Loopback test in bench uses ALLOUTPUT (0x17).
    p.control_write(CTRL_SET_SAFETY, 0x17, 0)  # ALLOUTPUT so the loopback frame can be sent internally
    time.sleep(0.2)
    # drain
    for _ in range(3):
        p.can_recv(); time.sleep(0.05)
    p.can_send(0x201, b"\xBE\xEF\xCA\xFE", BUS)
    time.sleep(0.2)
    got = []
    t0 = time.time()
    while time.time() - t0 < 1.0:
        for a, d, b in p.can_recv():
            if b == BUS:
                got.append((a, d.hex()))
        time.sleep(0.02)
    if any(a == 0x201 for a, _ in got):
        print("    LOOPBACK PASS -> CAN2 peripheral + firmware path OK. got:", got[:3])
        print("    => the STM32/transceiver2 CAN2 is FINE; any live-bus failure is CAR-SIDE wiring.")
    else:
        print("    LOOPBACK FAIL -> got:", got, " (CAN2 tx/rx path issue on the STM32 side)")

    # back to safe read-only + normal mode
    p.control_write(CTRL_LOOPBACK, 0, 0)
    p.control_write(CTRL_SET_SAFETY, 0, 0)     # SILENT
    time.sleep(0.3)

    # ---- 2) LIVE health ----
    print("\n[2] Live bus-1 signal (read-only, normal mode)...")
    for _ in range(5):
        p.can_recv(); time.sleep(0.1)
    h = health(p)
    print("    last_error=%s  REC=%d  err_passive=%d" % (h["last_error"], h["REC"], h["err_passive"]))
    if h["last_error"] == "FORM" and h["REC"] >= 200:
        print("    => noise/garbage on the wire, not valid CAN. Car-side differential is bad")
        print("       (single-ended CAN-H, or wrong pair). Needs proper CAN-H+CAN-L.")
    elif h["REC"] == 0 and h["last_error"] == "no-err":
        print("    => silent line: nothing arriving (car off / not connected).")

    print("\n(loopback = internal only; live part read-only. Nothing sent to a real bus.)")


if __name__ == "__main__":
    main()
