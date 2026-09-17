#!/usr/bin/env python3
"""Find out why the Mazda radar never answers the programming-session request.

The handshake in longitudinal.py reports only success/failure, and
IsoTpParallelQuery discards anything that is not the exact positive response it
is waiting for. So a NEGATIVE response -- the radar saying "no, and here is why"
-- looks identical to silence. This dumps everything instead.

WHAT IT SENDS. Only the three frames the panda's own safety model permits to
0x764 (mazda.h: tester present, and session control for default/programming).
Security access, routine control and memory writes are rejected in the firmware
by design, so this cannot reflash or reconfigure the radar. Suppression is
reversible on its own: stop tester-present and the radar times out back to
stock, and an ignition cycle always restores it.

HOW TO RUN. Stop dashcam_web first -- this needs exclusive /dev/ttyACM0.
Car stationary, wheels chocked, ignition on, engine running, MRCC MAIN on.

    sudo python3 probe_radar_uds.py

Then paste the output back.
"""
import os
_CX5_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir, os.pardir))
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))   # control/
# sudo drops PYTHONPATH *and* the invoking user's site-packages, so both have to
# be restored by hand: opendbc for create_longitudinal_messages, and tran's
# site-packages for capnp, which opendbc.car.structs imports at module level.
# Without the second one the failure is a bare "No module named 'capnp'".
sys.path.insert(0, os.path.join(_CX5_ROOT, "car"))
import glob                                                        # noqa: E402
for _sp in sorted(glob.glob("/home/tran/.local/lib/python3.*/site-packages")):
    if _sp not in sys.path:
        sys.path.append(_sp)

from bench_can_loopback import SerialPanda            # noqa: E402

SAFETY_MAZDA = 13
SAFETY_SILENT = 0
MAZDA_PARAM_LONGITUDINAL = 1

RADAR_ADDR = 0x764
RESP_ADDR = 0x76C          # 0x764 + response_offset 0x8
CRZ_INFO = 0x21B

# Negative response codes worth naming: each implies a different next step.
NRC = {
    0x10: "generalReject",
    0x11: "serviceNotSupported",
    0x12: "subFunctionNotSupported",
    0x13: "incorrectMessageLengthOrInvalidFormat",
    0x22: "conditionsNotCorrect  -> radar wants a different vehicle state",
    0x31: "requestOutOfRange",
    0x33: "securityAccessDenied -> needs 0x27, which mazda.h forbids",
    0x35: "invalidKey",
    0x36: "exceedNumberOfAttempts",
    0x37: "requiredTimeDelayNotExpired",
    0x78: "responsePending (not an error -- reply is coming)",
    0x7E: "subFunctionNotSupportedInActiveSession",
    0x7F: "serviceNotSupportedInActiveSession",
}

REQUESTS = [
    ("tester present            ", bytes([0x02, 0x3E, 0x80, 0, 0, 0, 0, 0])),
    ("session control: default  ", bytes([0x02, 0x10, 0x01, 0, 0, 0, 0, 0])),
    ("session control: PROGRAM  ", bytes([0x02, 0x10, 0x02, 0, 0, 0, 0, 0])),
]


def drain(p, seconds):
    """Collect every frame for `seconds`, keeping the tx status bits."""
    out = []
    end = time.time() + seconds
    while time.time() < end:
        try:
            out.extend(p.can_recv_ex())
        except Exception as e:
            print("   recv error:", e)
            break
        time.sleep(0.01)
    return out


def crz_info_rate(frames, seconds):
    n = sum(1 for (a, _d, b, _r, _t) in frames if b == 0 and a == CRZ_INFO)
    return n / seconds


def describe(dat):
    """Decode a UDS reply, positive or negative."""
    if len(dat) < 3:
        return "short frame"
    svc = dat[1]
    if svc == 0x7F:
        nrc = dat[3] if len(dat) > 3 else 0
        return (f"NEGATIVE to service 0x{dat[2]:02X}, "
                f"NRC 0x{nrc:02X} {NRC.get(nrc, 'unknown')}")
    if svc == 0x50:
        return f"POSITIVE session control, session type 0x{dat[2]:02X}"
    if svc == 0x7E:
        return "POSITIVE tester present"
    return f"service 0x{svc:02X}"


def hold_test(p, seconds):
    """Enter the programming session, then hold it with tester-present and watch.

    This is the question the one-shot probe cannot answer: the session opens and
    the radar goes quiet, then some seconds later it is transmitting again. Here
    we send the SAME keep-alive the CarController sends (0x3E 0x80 at 2 Hz, from
    make_tester_present_msg) and print the 0x21b rate every second, so the exact
    moment the radar comes back is visible -- and whether tester-present holds it
    at all.
    """
    print(f"\n=== hold test: programming session + tester-present at 2 Hz for {seconds}s ===")
    drain(p, 0.2)
    p.can_send(RADAR_ADDR, bytes([0x02, 0x10, 0x02, 0, 0, 0, 0, 0]), 0)
    frames = drain(p, 0.5)
    for a, d, b, r, t in frames:
        if b == 0 and a == RESP_ADDR and not r and not t:
            print(f"  session reply: {d.hex()} -> {describe(d)}")

    last_tp = 0.0
    t0 = time.time()
    returned_at = None
    while time.time() - t0 < seconds:
        now = time.time()
        if now - last_tp >= 0.5:                    # 2 Hz, same as TESTER_PRESENT_STEP
            p.can_send(RADAR_ADDR, bytes([0x02, 0x3E, 0x80, 0, 0, 0, 0, 0]), 0)
            last_tp = now
        f = drain(p, 1.0)
        rate = crz_info_rate(f, 1.0)
        el = now - t0
        print(f"  t+{el:5.1f}s  0x21b {rate:5.1f} Hz  {'<-- RADAR BACK' if rate > 5 else ''}")
        if rate > 5 and returned_at is None:
            returned_at = el

    if returned_at is None:
        print(f"\n  HELD for the full {seconds}s -- tester-present keeps the session.")
    else:
        print(f"\n  Radar returned after {returned_at:.1f}s despite tester-present at 2 Hz.")
    return returned_at


def hold_tx_test(p, seconds):
    """Same hold, but ALSO emit 0x21b/0x21c at 50 Hz like long_tx_thread does.

    The plain hold test keeps the session for 90 s. dashcam_web loses it in
    ~15 s while sending tester-present at a measured 1.8 Hz with zero errors and
    the panda never leaving Mazda safety mode. The one thing dashcam_web adds is
    this 50 Hz stream to the addresses the radar itself owns -- so this isolates
    whether transmitting them is what knocks the radar out of the session.

    accel is 0 and long_active False, i.e. the exact neutral frames long_tx
    sends while openpilot is disengaged. Nothing here commands motion.
    """
    from opendbc.car.mazda.longitudinal import create_longitudinal_messages

    # long_active flips CRZ_CTRL's CRZ_ACTIVE bit, i.e. the frame that tells the
    # PCM "ACC is engaged". The drivelog shows the radar returning in the SAME
    # sample that controls_allowed went true and openpilot reached preEnabled --
    # which is exactly when long_tx starts setting this bit. With it False the
    # session held 90 s, so this is the remaining untested variable.
    active = "--active" in sys.argv
    print(f"\n=== hold + 0x21b/0x21c at 50 Hz for {seconds}s "
          f"(CRZ_ACTIVE={'1 -- ACC ENGAGED' if active else '0'}) ===")
    if active:
        print("  !! This frame asserts ACC engaged. accel stays 0, but the PCM may")
        print("  !! release its brake hold. Vehicle in PARK, wheels chocked.")
    drain(p, 0.2)
    p.can_send(RADAR_ADDR, bytes([0x02, 0x10, 0x02, 0, 0, 0, 0, 0]), 0)
    for a, d, b, r, t in drain(p, 0.5):
        if b == 0 and a == RESP_ADDR and not r and not t:
            print(f"  session reply: {d.hex()} -> {describe(d)}")

    counter = 0
    last_tp = last_long = last_report = 0.0
    n_crz = n_tp = 0
    foreign = 0
    t0 = time.time()
    returned_at = None
    while time.time() - t0 < seconds:
        now = time.time()
        if now - last_tp >= 0.5:
            p.can_send(RADAR_ADDR, bytes([0x02, 0x3E, 0x80, 0, 0, 0, 0, 0]), 0)
            n_tp += 1
            last_tp = now
        if now - last_long >= 0.02:                       # 50 Hz
            for m in create_longitudinal_messages(0, 0.0, counter, active, False):
                p.can_send(m.address, bytes(m.dat), m.src)
            counter = (counter + 1) % 16
            n_crz += 1
            last_long = now
        for (a, _d, b, r, t) in p.can_recv_ex():
            if b == 0 and a == CRZ_INFO and not r and not t:
                foreign += 1
        if now - last_report >= 1.0:
            # Subtract our own echo: we send one 0x21b per create call.
            radar_share = foreign - n_crz
            el = now - t0
            print(f"  t+{el:5.1f}s  ours {n_crz:4d}  seen {foreign:4d}  "
                  f"radar~{max(0, radar_share):4d}  "
                  f"{'<-- RADAR BACK' if radar_share > 20 else ''}")
            if radar_share > 20 and returned_at is None:
                returned_at = el
            n_crz = foreign = 0
            last_report = now

    if returned_at is None:
        print(f"\n  HELD {seconds}s WITH the 50 Hz stream -- transmitting 0x21b/0x21c")
        print("  is NOT what knocks the radar out. Look elsewhere.")
    else:
        print(f"\n  Radar returned after {returned_at:.1f}s WITH the stream, but the")
        print("  plain hold survived 90s -- transmitting 0x21b/0x21c is the cause.")
    return returned_at


def main():
    p = SerialPanda("/dev/ttyACM0")
    try:
        # Longitudinal param, or the safety model rejects 0x764 outright.
        p.control_write(0xdc, SAFETY_MAZDA, MAZDA_PARAM_LONGITUDINAL)
        time.sleep(0.3)
        drain(p, 0.3)                                   # flush stale rx

        base = drain(p, 2.0)
        r0 = crz_info_rate(base, 2.0)
        print(f"\nbaseline: radar 0x21b at {r0:.1f} Hz "
              f"({'alive' if r0 > 5 else 'ALREADY QUIET -- unexpected'})")
        seen_resp_addrs = {a for (a, _d, b, _r, _t) in base if b == 0 and 0x700 <= a <= 0x7FF}
        print(f"          diagnostic-range addrs already on bus 0: "
              f"{sorted(hex(a) for a in seen_resp_addrs) or 'none'}")

        for label, req in REQUESTS:
            drain(p, 0.2)
            p.can_send(RADAR_ADDR, req, 0)
            frames = drain(p, 1.0)

            # Did our own frame leave the panda, or did safety refuse it?
            echo = [(r, t) for (a, _d, _b, r, t) in frames if a == RADAR_ADDR]
            rejected = any(r for r, _t in echo)
            went_out = any(t for _r, t in echo)
            replies = [(a, d) for (a, d, b, r, t) in frames
                       if b == 0 and 0x700 <= a <= 0x7FF and not r and not t]

            print(f"\n{label} {req[:3].hex()}")
            print(f"   tx: {'REJECTED by safety hook' if rejected else ('on the wire' if went_out else 'no echo seen')}")
            if replies:
                for a, d in replies:
                    print(f"   RX 0x{a:03x}: {d.hex()}  -> {describe(d)}")
            else:
                print(f"   RX: nothing in 0x700-0x7FF (expected a reply on 0x{RESP_ADDR:03x})")

            after = drain(p, 2.0)
            r1 = crz_info_rate(after, 2.0)
            print(f"   radar 0x21b after: {r1:.1f} Hz "
                  f"({'SILENCED' if r1 < 5 else 'still transmitting'})")

        print("\n--- interpretation ---")
        print("reply seen        -> read the NRC above; that is the real blocker")
        print("no reply, tx OK   -> radar ignores 0x764, or answers on an address")
        print("                     the panda filters out (MAZDA_HOST_IDS only")
        print("                     passes 0x76C on bus 0)")
        print("tx REJECTED       -> safety param is not 1; mazda.h dropped the frame")

        if "--hold" in sys.argv:
            i = sys.argv.index("--hold")
            secs = int(sys.argv[i + 1]) if len(sys.argv) > i + 1 else 60
            hold_test(p, secs)
        if "--hold-tx" in sys.argv:
            i = sys.argv.index("--hold-tx")
            secs = int(sys.argv[i + 1]) if len(sys.argv) > i + 1 else 60
            hold_tx_test(p, secs)
    finally:
        p.control_write(0xdc, SAFETY_SILENT, 0)
        print("\npanda returned to SAFETY_SILENT")


if __name__ == "__main__":
    main()
