#!/usr/bin/env python3
"""Flash the F407 panda with the locally built, signed image.

Deliberately a file rather than an inline snippet: flashing is the one action in
this tree that is awkward to undo, so the exact bytes and the exact target should
be reviewable before it runs, and greppable in history afterwards.

It flashes ONLY board/obj/panda_f407.bin.signed from the local panda_f446 tree.
It never fetches anything and never runs pandad.py -- that would replace this
board's firmware with stock and take the whole port with it (the software relay
in nooutput_init, MAZDA_FILTER, MAZDA_MADS, the ZLP fix in usb.h, the per-bus
host filter in bxcan.h, the F407 board config).

Leaves the panda in SAFETY_NOOUTPUT so cam<->car forwarding and the camera's ACK
survive, which is the safe idle state on a build with no harness relay.
"""
import sys
import time

sys.path.insert(0, "/home/tran/op_fork/jetson_port")
from usb_panda import UsbPanda

import os as _os
FW = _os.environ.get("OP_PANDA_FW", _os.path.join(
    _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))),
    "firmware", "board", "obj", "panda_f407.bin.signed"))
SAFETY_NOOUTPUT = 19


def main():
    p = UsbPanda()
    print("before: version=%s" % p.p.get_version())
    p.p.flash(FW)
    print("flash returned; reconnecting...")
    try:
        p.close()
    except Exception:
        pass
    time.sleep(3.5)

    p2 = UsbPanda()
    print("after : bootstub=%s version=%s" % (p2.p.bootstub, p2.p.get_version()))
    p2.p.set_safety_mode(SAFETY_NOOUTPUT)
    time.sleep(0.4)
    p2._buf = b""

    h = p2.p.health()
    print("health: safety_mode=%s controls_allowed=%s rx_buffer_overflow=%s faults=%s"
          % (h["safety_mode"], h["controls_allowed"], h["rx_buffer_overflow"], h["faults"]))

    seen = {}
    t0 = time.time()
    while time.time() - t0 < 3.0:
        for addr, _dat, bus in p2.can_recv():
            seen.setdefault(bus, {})
            seen[bus][addr] = seen[bus].get(addr, 0) + 1
    if not seen:
        print("bus quiet -- expected with the ignition off")
    for bus in sorted(seen):
        print("bus %d: %.0f fps / %d ids" % (bus, sum(seen[bus].values()) / 3.0, len(seen[bus])))
    p2.close()


if __name__ == "__main__":
    main()
