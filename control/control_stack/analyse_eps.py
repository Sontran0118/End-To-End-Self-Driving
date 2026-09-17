#!/usr/bin/env python3
"""Answer one question from a dashcam_web jsonl: does the EPS follow what we send?

    python3 analyse_eps.py /tmp/dashcam_<ts>.jsonl

The whole torque investigation reduces to whether LKAS_EFFECTIVE keeps tracking
LKAS_REQUEST as the request climbs, or flattens out. A flat top is the rack's own
clamp, and no STEER_MAX / ramp / firmware change moves it -- that is the case the
MoreTorque interceptor exists to solve. A straight line means the ceiling is still
on our side and the ramp work has somewhere to go.

Binned by request magnitude and reporting the MAX effective per bin on purpose:
the EPS has its own response lag, so at any instant `effective` may simply be on
its way up. The max over many samples in a bin is what the rack actually reached
for that level of ask; a mean would be dragged down by every transient.
"""
import json
import sys
from collections import defaultdict

BIN = 50            # counts per bucket


def main(path):
    rows = []
    bad = 0
    with open(path) as f:
        for line in f:
            try:
                rows.append(json.loads(line))
            except Exception:
                bad += 1
    if not rows:
        print("no records in %s" % path)
        return
    print("%s: %d records%s\n" % (path, len(rows), "  (%d unparsable)" % bad if bad else ""))

    def last(key, default=0):
        """Most recent non-null value of key. Counters are cumulative, so the
        last one is the run total."""
        for r in reversed(rows):
            if r.get(key) is not None:
                return r[key]
        return default

    # --- EPS acceptance ceiling ---------------------------------------------
    # Bin by what WE SENT (applied_torque) and read back what the EPS says it
    # RECEIVED (0x241 LKAS_REQUEST). Where those stop tracking is the rack's own
    # limit on how much LKAS authority it will accept.
    #
    # NOT eps_effective: that correlates only 0.73 with what we send and ranges
    # ABOVE the request (measured +-648 against a +-450 request), because it is
    # total rack output -- our command plus ordinary power-steering assist from
    # the driver's own input. It looks like the obvious signal and it is the
    # wrong one. eps_request correlates 0.97, which is what makes it usable.
    #
    # Mean, not max, per bin: applied_torque is instantaneous at snapshot time
    # while eps_request comes from the last 0x241, so individual pairs are
    # misaligned by up to a frame. That noise cancels in the mean and does not
    # in the max.
    buckets = defaultdict(lambda: {"n": 0, "req_sum": 0.0, "req_max": 0})
    seen_eps = 0
    for r in rows:
        if not r.get("lat_active"):
            continue
        applied = abs(r.get("applied_torque") or 0)
        if applied < 20:              # idle; says nothing about the ceiling
            continue
        seen_eps += 1
        req = abs(r.get("eps_request") or 0)
        b = buckets[(applied // BIN) * BIN]
        b["n"] += 1
        b["req_sum"] += req
        b["req_max"] = max(b["req_max"], req)

    if not seen_eps:
        print("NO lat_active records with applied torque -- nothing was ever\n"
              "commanded, so the ceiling cannot be measured. Check eps_seen > 0\n"
              "and that cruise was engaged.")
    else:
        print("EPS acceptance  (what WE sent -> what the rack says it RECEIVED)")
        print("  %-14s %7s %9s %9s %8s" % ("applied bin", "n", "req_mean", "req_max", "track"))
        ceiling = None
        for lo in sorted(buckets):
            b = buckets[lo]
            mid = lo + BIN / 2.0
            mean = b["req_sum"] / b["n"]
            print("  %-14s %7d %9.1f %9d %7.2f" %
                  ("%d-%d" % (lo, lo + BIN - 1), b["n"], mean, b["req_max"], mean / mid))
            # Tracking below 90% of what we sent, for a bin we sent >=2 BINs of:
            # the rack has stopped following and is holding its own limit.
            if ceiling is None and lo >= 2 * BIN and mean / mid < 0.90:
                ceiling = b["req_max"]
        print()
        if ceiling is not None:
            # A ceiling is only the RACK's if the panda passed the frames that
            # would have exceeded it. Under a jittery stream the panda's own
            # max_rt_delta becomes a hard ceiling (lateral.h:145 zeroes
            # rt_torque_last on every violation, so it never climbs), and the
            # result looks identical from here -- flat, symmetric, speed
            # independent. Reading that as an EPS property is exactly the wrong
            # call that was made on 2026-07-30, so refuse to make it again
            # without checking the rejection count first.
            rejects = last("panda_tx_blocked")
            print("  >>> CEILING OBSERVED AT ~%d counts." % ceiling)
            if rejects and rejects > 50:
                print("      *** NOT ATTRIBUTABLE TO THE EPS: %d panda rejections this\n"
                      "      run. Frames above the limit never reached the bus, so this\n"
                      "      number may just be mazda.h .max_rt_delta. Compare them --\n"
                      "      if they match, fix the tx jitter and re-run before drawing\n"
                      "      any conclusion about the rack." % rejects)
            else:
                print("      Only %d panda rejections, so frames were reaching the bus:\n"
                      "      this ceiling IS the rack. Raising STEER_MAX, max_torque or\n"
                      "      the ramp past it buys nothing." % (rejects or 0))
        else:
            print("  >>> NO CEILING FOUND in this range -- the rack tracked everything\n"
                  "      we sent. Push harder to find it.")

    # --- tx health ----------------------------------------------------------
    print("\ntx health")
    for k, label in (("lkas_hz", "achieved rate (Hz)"),
                     ("lkas_target_ms", "target interval (ms)"),
                     ("lkas_worst_ms_all", "worst interval this run (ms)"),
                     ("lkas_late", "late frames"),
                     ("panda_tx_blocked", "panda rejections"),
                     ("tx_resyncs", "resyncs"),
                     ("applied_peak", "peak applied torque"),
                     ("want_peak", "peak wanted torque"),
                     ("steer_max", "STEER_MAX"),
                     ("cam_trq_peak", "stock camera peak")):
        print("  %-28s %s" % (label, last(k)))

    lb = sum(1 for r in rows if r.get("lkas_block"))
    ho = sum(1 for r in rows if r.get("hands_off_5s"))
    mv = sum(1 for r in rows if (r.get("v_ego_kph") or 0) > 5)
    print("  %-28s %d / %d records" % ("LKAS_BLOCK asserted", lb, len(rows)))
    print("  %-28s %d / %d records" % ("hands-off 5s asserted", ho, len(rows)))
    print("  %-28s %d / %d records" % ("moving (>5 kph)", mv, len(rows)))

    fps = [r["fps"] for r in rows if r.get("fps")]
    if fps:
        print("  %-28s %.1f mean, %.1f min  (target 20.0)"
              % ("model fps", sum(fps) / len(fps), min(fps)))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1])
