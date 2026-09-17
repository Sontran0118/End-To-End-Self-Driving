#!/usr/bin/env python3
"""Read (and optionally clear) the radar's stored DTCs at UDS 0x764.

WHY THIS EXISTS. The radar's dashboard indicator used to flash through a normal
power-on self-test and then go out; it no longer flashes at all, and the Smart
City Brake Support malfunction is now permanent. A module carrying stored fault
codes behaves exactly like that -- it skips the healthy startup announcement and
reports the fault on every ignition cycle until the memory is cleared.

There is good reason to think codes accumulated. Every suppression attempt began
with 0x85 ControlDTCSetting OFF -- the request a workshop tool sends to stop an
ECU logging faults during a procedure -- and this radar REFUSED IT EVERY TIME
("mazda radar DTC setting OFF refused (continuing)"). So it was logging throughout
dozens of programming-session entries and CommunicationControl commands.

That would also explain why suppression appeared to work cleanly once and does not
now: the difference is not in what we send, it is in what the radar has stored.

READ IS THE DEFAULT. Clearing fault memory on a safety ECU is a real write and is
not done unless --clear is passed explicitly.

    19 02 FF        ReadDTCInformation, reportDTCByStatusMask, mask 0xFF (all)
    14 FF FF FF     ClearDiagnosticInformation, group 0xFFFFFF (all groups)

Both are standard ISO 14229. Nothing else is transmitted.

SAFETY MODE. 0x764 is only in the panda's tx table under SAFETY_MAZDA with the
longitudinal parameter, so that mode is set for the few hundred milliseconds this
takes and reverted to SAFETY_NOOUTPUT immediately afterwards. No tx thread runs, so
nothing else can be sent while it is up.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from usb_panda import UsbPanda

RADAR_ADDR = 0x764
RADAR_RESP = 0x76C
SAFETY_NOOUTPUT = 19
SAFETY_MAZDA = 13
MAZDA_PARAM_LONGITUDINAL = 1

# ISO 14229 negative response codes worth naming when they come back.
NRC = {
    0x10: "generalReject",
    0x11: "serviceNotSupported",
    0x12: "subFunctionNotSupported",
    0x13: "incorrectMessageLengthOrInvalidFormat",
    0x22: "conditionsNotCorrect",
    0x31: "requestOutOfRange",
    0x33: "securityAccessDenied",
    0x78: "requestCorrectlyReceived-ResponsePending",
}

# DTC status bits (ISO 14229-1 table D.1), the ones that actually matter here.
STATUS_BITS = (
    (0x01, "testFailed"),
    (0x02, "testFailedThisOperationCycle"),
    (0x04, "pendingDTC"),
    (0x08, "confirmedDTC"),
    (0x40, "testFailedSinceLastClear"),
    (0x80, "warningIndicatorRequested"),
)


def send_uds(p, payload, wait=1.0):
    """Single-frame ISO-TP request, collect whatever comes back on 0x76C."""
    frame = bytes([len(payload)]) + bytes(payload)
    frame += b"\x00" * (8 - len(frame))
    p._buf = b""
    p.can_send(RADAR_ADDR, frame, 0)
    out, t0 = [], time.time()
    while time.time() - t0 < wait:
        for addr, dat, bus in p.can_recv():
            if bus == 0 and addr == RADAR_RESP:
                out.append(bytes(dat))
        time.sleep(0.002)
    return out


def describe(resp, service):
    if not resp:
        return "no reply"
    b = resp[0]
    if len(b) >= 3 and b[1] == 0x7F:
        nrc = b[3] if len(b) > 3 else 0
        return "NEGATIVE 0x%02x %s" % (nrc, NRC.get(nrc, "?"))
    if len(b) >= 2 and b[1] == service + 0x40:
        return "positive"
    return "unexpected"


def main():
    do_clear = "--clear" in sys.argv
    p = UsbPanda()
    p.p.set_safety_mode(SAFETY_NOOUTPUT)
    time.sleep(0.3)
    p.p.set_safety_mode(SAFETY_MAZDA, MAZDA_PARAM_LONGITUDINAL)
    time.sleep(0.3)
    try:
        # THE DEFAULT SESSION IS A DEAD END ON THIS RADAR. It answers 10 02 and
        # essentially nothing else -- 19 02 FF got no reply at all, exactly as
        # 0x28, 0x85, 10 03 and 10 04 have all done. Every command that has ever
        # worked today worked inside the programming session, so ask from there.
        print("entering programming session (10 02) first ...")
        resp = send_uds(p, [0x10, 0x02])
        print("  -> %s" % describe(resp, 0x10))
        for r in resp:
            print("     %s" % r.hex(" "))

        print()
        print("radar DTC read (19 02 FF) ...")
        resp = send_uds(p, [0x19, 0x02, 0xFF])
        print("  -> %s" % describe(resp, 0x19))
        for r in resp:
            print("     %s" % r.hex(" "))
        if resp and resp[0][1] == 0x59:
            # 59 02 <availabilityMask> then 4-byte records: 3 DTC bytes + status
            payload = b"".join(r[1:] for r in resp)
            body = payload[3:]
            n = len(body) // 4
            print("  %d DTC record(s):" % n)
            for i in range(n):
                dtc = body[i * 4:i * 4 + 3]
                st = body[i * 4 + 3]
                flags = [name for bit, name in STATUS_BITS if st & bit]
                print("     DTC %s  status 0x%02x  %s"
                      % (dtc.hex().upper(), st, ", ".join(flags) or "-"))

        if do_clear:
            print()
            print("radar DTC CLEAR (14 FF FF FF) ...")
            resp = send_uds(p, [0x14, 0xFF, 0xFF, 0xFF], wait=2.0)
            print("  -> %s" % describe(resp, 0x14))
            for r in resp:
                print("     %s" % r.hex(" "))
            print()
            print("re-reading to confirm ...")
            resp = send_uds(p, [0x19, 0x02, 0xFF])
            print("  -> %s" % describe(resp, 0x19))
            for r in resp:
                print("     %s" % r.hex(" "))
        else:
            print()
            print("(read only -- pass --clear to erase fault memory)")
    finally:
        p.p.set_safety_mode(SAFETY_NOOUTPUT)
        print("panda returned to SAFETY_NOOUTPUT (forwarding + ACK alive)")


if __name__ == "__main__":
    main()
