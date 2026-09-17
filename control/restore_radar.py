#!/usr/bin/env python3
"""Undo a radar suppression that did not time out on its own.

WHY THIS EXISTS. Alpha long silences the radar with a UDS programming session on
0x764, held open by tester-present. The design assumes that dropping
tester-present lets the session time out and the radar return to stock -- the
comment in mazda/interface.py calls that "the intended failure direction".

MEASURED 2026-08-09: it does NOT time out on this car. After an alpha-long run
was stopped, a later run with alpha long OFF still showed all seven RADAR_*
frames silent, and 0x21b never resumed. The radar stays mute, which means the car
has no FCW, no AEB and no SBS until it is power-cycled -- a safety-relevant state
to be left in by simply quitting a process.

WHAT IT SENDS, to 0x764 only:
    10 01        DiagnosticSessionControl -> default session
    28 00 01     CommunicationControl -> enableRxAndEnableTx, normal messages
    85 01        ControlDTCSetting -> ON

All three are the documented inverse of what suppression does. Nothing else is
transmitted.

SAFETY MODE. The panda's tx whitelist only permits the radar UDS address under
SAFETY_MAZDA with the longitudinal param, so that mode is set for the few hundred
milliseconds this takes and reverted to SAFETY_NOOUTPUT immediately afterwards.
No tx thread is running, so nothing else can be sent while it is up.

If 0x21b does not resume afterwards, the radar needs an ignition cycle and this
script cannot help.
"""
import sys
import time

sys.path.insert(0, "/home/tran/op_fork/jetson_port")
from usb_panda import UsbPanda

RADAR_ADDR = 0x764
CRZ_INFO = 0x21B
SAFETY_NOOUTPUT = 19
SAFETY_MAZDA = 13
MAZDA_PARAM_LONGITUDINAL = 1

RESTORE = (
    ("default session  (10 01)",    [0x02, 0x10, 0x01, 0, 0, 0, 0, 0]),
    ("comm ctrl tx on  (28 00 01)", [0x03, 0x28, 0x00, 0x01, 0, 0, 0, 0]),
    ("DTC setting on   (85 01)",    [0x02, 0x85, 0x01, 0, 0, 0, 0, 0]),
)


def count_crz_info(p, seconds=3.0):
    p._buf = b""
    t0, n = time.time(), 0
    while time.time() - t0 < seconds:
        for addr, _dat, bus in p.can_recv():
            if bus == 0 and addr == CRZ_INFO:
                n += 1
    return n


def main():
    p = UsbPanda()
    p.p.set_safety_mode(SAFETY_NOOUTPUT)
    time.sleep(0.4)
    before = count_crz_info(p)
    print("radar 0x21b before: %d frames / 3 s" % before)
    if before > 50:
        print("radar is already transmitting -- nothing to restore")
        p.close()
        return

    p.p.set_safety_mode(SAFETY_MAZDA, MAZDA_PARAM_LONGITUDINAL)
    time.sleep(0.4)
    for label, payload in RESTORE:
        try:
            p.can_send(RADAR_ADDR, bytes(payload), 0)
            print("  sent %s" % label)
        except Exception as e:
            print("  send FAILED %s: %s" % (label, e))
        time.sleep(0.3)

    time.sleep(1.5)
    after = count_crz_info(p)
    print("radar 0x21b after : %d frames / 3 s" % after)
    print("RADAR RECOVERED" if after > 50 else
          "STILL MUTE -- ignition cycle required (the radar must reboot)")

    p.p.set_safety_mode(SAFETY_NOOUTPUT)
    time.sleep(0.3)
    print("panda returned to SAFETY_NOOUTPUT (forwarding + ACK alive)")
    p.close()


if __name__ == "__main__":
    main()
