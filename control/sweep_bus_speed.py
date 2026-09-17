#!/usr/bin/env python3
"""Sweep bus 1 (transceiver 2 / camera) across CAN bitrates to find the one the
camera bus actually runs at. FORM errors + REC=255 with no frames means the
STM32's bitrate doesn't match the bus. This tries each standard speed and reports
frames received + error state, so we can tell 'wrong bitrate' from 'bad wiring'.

Read-only (SAFETY_SILENT). No transmit.

Usage: python3 sweep_bus_speed.py [/dev/ttyACM0] [bus]
"""
import sys, struct, time

sys.path.insert(0, "/home/tran/op_fork/jetson_port")
from bench_can_loopback import SerialPanda

DEV = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BUS = int(sys.argv[2]) if len(sys.argv) > 2 else 1

CTRL_SET_SAFETY = 0xdc
CTRL_SET_SPEED = 0xde       # param1=bus, param2=speed(×0.1kbps)
CTRL_CAN_HEALTH = 0xc2

# valid speeds from firmware `speeds[]` (units of 0.1 kbps)
SPEEDS = [1000, 2000, 5000, 10000]   # 100k, 200k, 500k, 1M — the common automotive ones
LEC = {0: "no-err", 1: "STUFF", 2: "FORM", 3: "ACK", 4: "BITREC", 5: "BITDOM", 6: "CRC", 7: "nochg"}


def main():
    p = SerialPanda(DEV)
    p.control_write(CTRL_SET_SAFETY, 0, 0)   # SILENT — no transmit
    print("=== BITRATE SWEEP on bus %d (read-only) ===" % BUS)
    print("looking for the speed where FORM errors clear and frames flow\n")

    for spd in SPEEDS:
        kbps = spd / 10.0
        # set bus speed (this re-inits the CAN peripheral at the new bitrate)
        p.control_write(CTRL_SET_SPEED, BUS, spd)
        time.sleep(0.5)
        # drain fresh
        frames = 0
        addrs = set()
        t0 = time.time()
        while time.time() - t0 < 2.5:
            for addr, dat, bus in p.can_recv():
                if bus == BUS:
                    frames += 1
                    addrs.add(addr)
            time.sleep(0.01)
        # read health
        h = p.control_read(CTRL_CAN_HEALTH, BUS, 0, 24) or b"\x00" * 24
        lec = h[7] if len(h) > 7 else 0
        rec = h[11] if len(h) > 11 else 0
        epv = h[6] if len(h) > 6 else 0
        tag = "  <<< DECODES!" if frames > 0 else ""
        print("  %6.0f kbps: %4d frames, %2d IDs | last_error=%-6s REC=%3d err_passive=%d%s"
              % (kbps, frames, len(addrs), LEC.get(lec, "?"), rec, epv, tag))

    print("\n(read-only — nothing transmitted). Restoring 500k default.")
    p.control_write(CTRL_SET_SPEED, BUS, 5000)


if __name__ == "__main__":
    main()
