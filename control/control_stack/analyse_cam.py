#!/usr/bin/env python3
"""Answer one question from a dashcam_web jsonl: why does the cluster raise the
front camera fault?

    python3 analyse_cam.py ~/drivelogs/eps_<ts>.jsonl

On this relay-less board the car's ONLY source of forward-camera data is the
panda: 0x243 comes from our transmit thread, and every other FSC frame is
relayed bus 2 -> bus 0 by the RX ISR. So there are exactly three ways the car
can decide the camera is gone, and this script separates them:

  1. THE PANDA STOPPED FORWARDING.  SAFETY_SILENT sets disable_forwarding, which
     blocks the whole cam->car path, not just our frame. Nothing on the host side
     looks wrong when this happens -- can_send still succeeds over the serial
     link and tx_frames keeps climbing at the nominal rate. can_fwd2 (the
     firmware's own forward counter for the camera bus) and panda_safety_mode
     are the only witnesses.

  2. OUR 0x243 STREAM IS IRREGULAR.  The EPS faults on gaps, not on mean rate.
     lkas_worst_ms is the worst gap in the last second; lkas_recv_ms says how
     much of the link a single can_recv is eating, which is what produces them.

  3. THE CAMERA ITSELF IS DEGRADED OR WE ARE LOSING ITS FRAMES.  The census
     gives the real per-address rate on bus 2. Compare 0x243 there against the
     rate we transmit: that is the reference we are supposed to be imitating.
     panda_rx_overflow / can_rx_lost2 say whether a low number is loss rather
     than the camera's true cadence.
"""
import json
import sys


def load(path):
    rows, bad = [], 0
    with open(path) as f:
        for line in f:
            try:
                rows.append(json.loads(line))
            except Exception:
                bad += 1
    return rows, bad


def main(path):
    rows, bad = load(path)
    if not rows:
        print("no records in %s" % path)
        return 1
    print("%s: %d records%s" % (path, len(rows), "  (%d unparsable)" % bad if bad else ""))
    t0 = rows[0]["t"]
    print("span %.1f s\n" % (rows[-1]["t"] - t0))

    # Only the stretch where the car was actually powered is worth reporting on.
    # Before key-on there is no bus: cam_seen and eps_seen sit at 0 while
    # tx_frames climbs, and averaging over that window makes every rate wrong.
    alive = [r for r in rows if r.get("eps_seen", 0) > 0 or r.get("cam_seen", 0) > 0]
    if not alive:
        print("!! the car was never on in this log -- no CAN was received at all.")
        print("   Everything below would be measured against a dead bus.")
        return 0
    # ... and only while it stayed powered. rx freezing mid-log is key-off, not
    # a fault, but it drags every average down if it is left in.
    live = []
    for r in alive:
        if live and r.get("eps_seen") == live[-1].get("eps_seen") \
                and r.get("cam_seen") == live[-1].get("cam_seen"):
            continue
        live.append(r)
    span = live[-1]["t"] - live[0]["t"] if len(live) > 1 else 0.0
    print("car powered from t+%.0fs for %.0fs\n" % (alive[0]["t"] - t0, span))

    def d(key):
        return live[-1].get(key, 0) - live[0].get(key, 0)

    def rate(key):
        return d(key) / span if span > 0 else 0.0

    last = live[-1]

    print("--- 1. is the panda still forwarding the camera to the car? ---")
    mode = last.get("panda_safety_mode", -1)
    print("  panda_safety_mode   %s%s" % (mode,
          "  (SAFETY_MAZDA - forwarding on)" if mode == 13 else
          "  <-- NOT MAZDA: forwarding is OFF, the car sees no camera at all"
          if mode >= 0 else "  (not logged - old run)"))
    print("  panda_remodes       %s%s" % (last.get("panda_remodes", "n/a"),
          "   <-- the watchdog had to re-arm the panda; it HAD dropped out"
          if last.get("panda_remodes") else ""))
    fwd2 = d("can_fwd2")
    rx2 = d("can_rx2")
    if "can_fwd2" in last:
        print("  cam->car forwarded  %d frames (%.1f/s) out of %d received on bus 2"
              % (fwd2, fwd2 / span if span else 0, rx2))
        if rx2 and fwd2 == 0:
            print("    <-- the camera is alive on bus 2 and NOTHING is reaching the car.")
    else:
        print("  cam->car forwarded  not logged (run predates the can_health poll)")
    print("  panda_faults        %s   heartbeat_lost %s"
          % (last.get("panda_faults"), last.get("panda_heartbeat_lost", "n/a")))

    print("\n--- 2. is our own 0x243 stream regular enough? ---")
    print("  tx rate             %.1f Hz  (%d frames)" % (rate("tx_frames"), d("tx_frames")))
    print("  worst gap, all run  %.1f ms   target %.1f ms"
          % (last.get("lkas_worst_ms_all", -1), last.get("lkas_target_ms", -1)))
    print("  late frames         %d of %d (%.1f%%)"
          % (d("lkas_late"), d("tx_frames"),
             100.0 * d("lkas_late") / d("tx_frames") if d("tx_frames") else 0))
    # waits are 0.5 ms ticks spent phasing a recv behind a send, so
    # ticks*0.5/1000/span is the FRACTION of wall time can_thread spends waiting.
    # Expressed that way it needs no recv count, which the log does not carry.
    _w = d("lkas_rx_waits")
    print("  can_recv cost       %s ms   phase wait %s of wall time"
          % (last.get("lkas_recv_ms", "n/a"),
             ("%.0f%%" % (100.0 * _w * 0.0005 / span)) if span > 0 else "n/a"))
    print("  frames the firmware rejected: %s (tx_blocked_seen %s, resyncs %s)"
          % (last.get("panda_tx_blocked"), last.get("tx_blocked_seen"),
             last.get("tx_resyncs")))
    band = {}
    for r in live:
        w = r.get("lkas_worst_ms")
        if w is not None:
            band[int(w) // 10 * 10] = band.get(int(w) // 10 * 10, 0) + 1
    tot = sum(band.values()) or 1
    for k in sorted(band):
        print("    %3d-%3d ms  %6d (%4.1f%%)" % (k, k + 9, band[k], 100.0 * band[k] / tot))

    print("\n--- 3. what is the real camera doing? ---")
    print("  0x243 seen on bus 2 %.1f Hz   (we transmit %.1f Hz on bus 0)"
          % (rate("cam_seen"), rate("tx_frames")))
    print("  0x440 seen on bus 2 %.1f Hz" % rate("cam_lane_seen"))
    print("  cam_age at end      %s s" % last.get("cam_age_s"))
    print("  rx dropped: panda queue %s, CAN2 FIFO %s, CAN0 FIFO %s"
          % (d("panda_rx_overflow") if "panda_rx_overflow" in last else "n/a",
             d("can_rx_lost2") if "can_rx_lost2" in last else "n/a",
             d("can_rx_lost0") if "can_rx_lost0" in last else "n/a"))
    print("  0x243 checksum vs mazdacan: ok %s bad %s  (non-zero angle seen %s)"
          % (last.get("ck_ok"), last.get("ck_bad"), last.get("ck_angle_seen")))

    # The camera's own 4-bit sequence number, differenced across the frames we
    # actually received. This is the difference between "the camera runs at
    # 16 Hz" and "the camera runs at 100 Hz and we see one frame in six", which
    # are opposite conclusions about whether our 50 Hz stream is too fast.
    gaps = last.get("cam_ctr_gaps")
    if gaps:
        tot_g = sum(gaps.values()) or 1
        top = sorted(gaps.items(), key=lambda kv: -kv[1])[:5]
        print("  CTR gap between the 0x243 frames we saw: %s"
              % ", ".join("+%s x%d (%.0f%%)" % (k, v, 100.0 * v / tot_g) for k, v in top))
        one = gaps.get("1", 0)
        if one > 0.9 * tot_g:
            print("    -> we receive the camera WHOLE. %.0f Hz is its real rate,"
                  % rate("cam_seen"))
            print("       so our %.0f Hz stream is faster than the frame it replaces."
                  % rate("tx_frames"))
        else:
            miss = sum(int(k) * v for k, v in gaps.items()) / tot_g
            print("    -> we see roughly 1 frame in %.1f. The camera's true rate is"
                  % miss)
            print("       about %.0f Hz and the low reading is host-side loss."
                  % (rate("cam_seen") * miss))

    # Census: two samples far enough apart to difference into a rate. Scanned
    # over `alive`, not `live`: the census rides on every 200th record, and the
    # dedup that builds `live` throws away any record where neither counter
    # moved -- which silently dropped most census samples.
    cens = [(r["t"], r["can_census"]) for r in alive if "can_census" in r]
    if len(cens) >= 2:
        (ta, a), (tb, b) = cens[0], cens[-1]
        dt = tb - ta
        print("\n  per-address rates over %.0f s (bus 2 is the camera segment;" % dt)
        print("  bus 0 is gated to 17 IDs by MAZDA_FILTER, so it is NOT the whole car bus):")
        for bus in sorted(b):
            print("    bus %s:" % bus)
            for addr in sorted(b[bus], key=lambda x: -(b[bus][x] - a.get(bus, {}).get(x, 0))):
                n = b[bus][addr] - a.get(bus, {}).get(addr, 0)
                if n <= 0:
                    continue
                print("      %-8s %7.1f Hz   %d frames" % (addr, n / dt if dt else 0, n))
    else:
        print("\n  (no can_census in this log -- run predates the census)")

    print("\n--- what the numbers mean ---")
    if "can_fwd2" in last and rx2 and fwd2 == 0:
        print("  The panda received the camera and forwarded NOTHING. That alone")
        print("  produces the cluster fault; nothing about our 0x243 matters until")
        print("  it is fixed. Check panda_safety_mode above.")
    elif mode not in (13, -1):
        print("  The panda was not in SAFETY_MAZDA. In SILENT the firmware sets")
        print("  disable_forwarding and the car sees no camera whatsoever.")
    else:
        cam_hz, tx_hz = rate("cam_seen"), rate("tx_frames")
        if cam_hz > 5 and tx_hz > 5 and cam_hz > 1.5 * tx_hz:
            print("  The real camera sends 0x243 at %.0f Hz and we replace it with a"
                  % cam_hz)
            print("  %.0f Hz stream. That is the remaining difference between our frame" % tx_hz)
            print("  and the one the EPS was built to receive -- raise --lkas-hz.")
        elif last.get("lkas_worst_ms_all", 0) > 2.5 * (last.get("lkas_target_ms") or 20):
            print("  Rate is right but the stream has holes (worst %.0f ms against a"
                  % last.get("lkas_worst_ms_all", 0))
            print("  %.0f ms period). The EPS faults on gaps, not on mean rate."
                  % (last.get("lkas_target_ms") or 20))
        else:
            print("  Forwarding is alive and the 0x243 stream is at rate and regular.")
            print("  If the cluster still shows the fault, it was raised BEFORE this")
            print("  run and latched: arm the stack first, then cycle the ignition.")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
