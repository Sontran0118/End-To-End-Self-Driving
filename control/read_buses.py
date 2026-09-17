#!/usr/bin/env python3
"""READ-ONLY CAN bus reader for the DIY F446 panda.

Sets SAFETY_SILENT (no transmit), drains CAN for a few seconds, and reports what
each bus (0=CAN1/transceiver1, 1=CAN2/transceiver2, 2=CAN3=CAN2 on F446) is
receiving: frame count, unique addresses, and per-bus health (ESR/last_error).

STRICTLY read-only. No can_send anywhere.

Usage: python3 read_buses.py [/dev/ttyACM0] [seconds]
"""
import sys, struct, time, collections

# reuse the verified serial-panda client
sys.path.insert(0, "/home/tran/op_fork/jetson_port")
from bench_can_loopback import SerialPanda

DEV = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

# control requests (from panda main_comms.h)
SAFETY_SILENT = 0x00      # safety model 0 = SILENT: all TX blocked
CTRL_SET_SAFETY = 0xdc    # set_safety_hooks(mode, param)
CTRL_HEALTH = 0xd2        # health packet
CTRL_CAN_HEALTH = 0xc2    # per-bus CAN health (param1 = bus)


def can_health(p, bus):
    """Read the per-bus CAN health struct (0xc2). Returns raw bytes (parse fields
    of interest: bus_off, error_warning/passive, last_error/LEC, total_rx_cnt)."""
    try:
        return p.control_read(CTRL_CAN_HEALTH, bus, 0, 64)
    except Exception:
        return None


def main():
    print("=== READ-ONLY BUS READER on %s (%.0fs) ===" % (DEV, SECS))
    p = SerialPanda(DEV)

    # SAFETY_SILENT: this blocks ALL transmit at the firmware level.
    p.control_write(CTRL_SET_SAFETY, SAFETY_SILENT, 0)
    print("safety = SILENT (transmit blocked). Reading buses...\n")

    counts = collections.Counter()          # bus -> total frames
    addrs = collections.defaultdict(set)    # bus -> set of addresses
    samples = {}                            # (bus,addr) -> last data bytes

    t0 = time.time()
    while time.time() - t0 < SECS:
        for addr, dat, bus in p.can_recv():
            counts[bus] += 1
            addrs[bus].add(addr)
            samples[(bus, addr)] = dat
        time.sleep(0.01)

    print("--- per-bus summary ---")
    for bus in (0, 1, 2):
        n = counts.get(bus, 0)
        na = len(addrs.get(bus, ()))
        print("  bus %d: %5d frames, %3d unique IDs" % (bus, n, na))
    print()

    # detail for bus 1 (transceiver 2 / camera side)
    for bus in (0, 1, 2):
        if counts.get(bus, 0) == 0:
            continue
        ids = sorted(addrs[bus])
        print("--- bus %d IDs (%d) ---" % (bus, len(ids)))
        print("  " + " ".join("0x%X" % a for a in ids[:40]))
        # show 0x243 CAM_LKAS if present on this bus
        if 0x243 in addrs[bus]:
            print("  >>> 0x243 CAM_LKAS present on bus %d: %s"
                  % (bus, samples[(bus, 0x243)].hex()))
        print()

    print("--- per-bus health (ESR / errors) ---")
    for bus in (0, 1, 2):
        h = can_health(p, bus)
        if h:
            print("  bus %d health: %s" % (bus, h[:24].hex()))
        else:
            print("  bus %d health: (no response)" % bus)

    print("\n(read-only — nothing was transmitted)")


if __name__ == "__main__":
    main()
