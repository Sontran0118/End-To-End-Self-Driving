#!/usr/bin/env python3
"""Record what the REAL radar puts on the bus when stock ACC activates.

WHY THIS EXISTS. Alpha long sends 0x21b/0x21c that are correct by every test we
can run offline: the DBC decodes them to the right signals, the checksums and
counters are right, they go out on both buses, the panda accepts them, the bus
ACKs them, and long_tx_err stays 0. MEASURED 2026-08-11 from a standstill with
the radar in software standby:

    0x21b = 01 ff e1 80 06 80 ..   ACC_ACTIVE=1  ACC_SET_ALLOWED=1  ACCEL_CMD=+2.0
    0x21c = 0a 01 0b 20 00 00 10 00   CRZ_ACTIVE=1
    longActive=1  enabled=1  accel=+2.00

and the car did nothing at all -- engine held idle at 646 rpm, no gas, no brake,
acc_active flat 0. Across every log this port has, acc_active=1 appears 769
times and ALL 769 have the radar transmitting. Not once from our own frames.

So the question is no longer "are our frames well formed" -- they are. It is
"what does the radar send that we do not", and nothing has ever captured the
former to compare against. Every hypothesis tried so far (frame content,
checksums, counters, bus coverage, suppression method, session type, DTC state,
SET button handshake, engagement ordering, replaying the radar's own captured
frames) was inference from the DBC, and all of them failed.

WHAT THIS DOES. Nothing. It transmits not one frame. It puts the panda in Mazda
safety with the longitudinal param CLEAR, which leaves bus 0 <-> bus 2
forwarding intact and the radar completely untouched, then listens. You drive
and engage cruise the normal way; FCW, AEB and SBS stay live throughout, which
also makes this the safest test in this project.

WHY NOT SAFETY_SILENT. nooutput_init() returns disable_forwarding = true, so in
SILENT the panda stops relaying the forward camera to the car entirely and the
cluster raises the front camera fault. Mazda-with-param-0 keeps forwarding and
grants no transmit authority we then decline to use.

Usage:
    stop both dashcam_web roles first (they own the USB handle), then
    python3 capture_acc_activation.py [--seconds 300] [--out FILE]

Then engage cruise: MAIN, get above the set-speed floor, press SET. The script
prints the 0x21b/0x21c bytes on either side of the acc_active 0->1 edge and
writes every frame to JSONL for offline diffing.
"""
from __future__ import annotations

import argparse
import json
import struct
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import usb_panda  # noqa: E402

SAFETY_MAZDA = 13
SAFETY_PARAM_NONE = 0          # NOT 1 -- param 1 is alpha long, which permits tx

CRZ_INFO = 0x21B               # radar -> ACCEL_CMD, ACC_ACTIVE, ACC_SET_ALLOWED
CRZ_CTRL = 0x21C               # radar -> CRZ_ACTIVE, CRZ_AVAILABLE, DISTANCE_SETTING
PEDALS = 0x165                 # PCM   -> the acc_active bit we trigger on
CRZ_EVENTS = 0x21F             # set speed
CRZ_BTNS = 0x09D               # the driver's SET/RES press
WATCH = (CRZ_INFO, CRZ_CTRL, PEDALS, CRZ_EVENTS, CRZ_BTNS)

# PEDALS bit 3 is stock-ACC-active, the same bit mazda.h reads as cruise_engaged
# in its alpha-long branch. GET_BIT numbering: byte = i / 8, bit = i % 8.
ACC_ACTIVE_BYTE, ACC_ACTIVE_MASK = 0, 0x08

# Frames kept either side of the edge. The transition is the whole point, so
# capture enough before it to see the radar's run-up, not just the settled state.
PRE_FRAMES = 400
POST_FRAMES = 400


def acc_active(data: bytes) -> bool:
  return len(data) > ACC_ACTIVE_BYTE and bool(data[ACC_ACTIVE_BYTE] & ACC_ACTIVE_MASK)


def main() -> int:
  ap = argparse.ArgumentParser()
  ap.add_argument("--seconds", type=float, default=300.0)
  ap.add_argument("--out", default="/home/tran/drivelogs/acc_activation.jsonl")
  args = ap.parse_args()

  p = usb_panda.UsbPanda()
  # Mazda safety with the longitudinal bit CLEAR. Forwarding stays on, the radar
  # is not touched, and the tx table is the 3-entry stock one -- which we then
  # never use. Belt and braces: this script has no can_send call anywhere.
  p.control_write(0xDC, SAFETY_MAZDA, SAFETY_PARAM_NONE)
  time.sleep(0.2)

  hh = p.control_read(0xD2, 0, 0, 64)
  fmt = "<IIIIIIIIBBBBBHBBBHfBBHBHHB"
  h = struct.unpack(fmt, hh[:struct.calcsize(fmt)]) if hh else None
  if h:
    print("  panda safety mode=%d param=%d  (want %d / %d -- radar untouched, "
          "FCW/AEB live)" % (h[12], h[13], SAFETY_MAZDA, SAFETY_PARAM_NONE))

  print("  listening for %.0f s. Engage cruise normally: MAIN, get up to speed, "
        "press SET." % args.seconds)
  print("  NOTHING IS TRANSMITTED. The radar keeps running throughout.\n")

  out = open(args.out, "w")
  recent: list[dict] = []
  post_left = 0
  fired = False
  last_acc = False
  n = 0
  t_end = time.time() + args.seconds

  while time.time() < t_end:
    try:
      msgs = p.can_recv()
    except Exception as e:
      print("  can_recv failed:", e)
      break
    if not msgs:
      time.sleep(0.002)
      continue

    for addr, data, bus in ((m[0], bytes(m[1]), m[2]) for m in msgs):
      # Our own echoes come back tagged bus + 128. We send nothing, but filter
      # anyway so a stray frame can never be mistaken for the radar's.
      if bus >= 128 or addr not in WATCH:
        continue
      n += 1
      rec = {"t": round(time.time(), 4), "addr": addr, "bus": bus,
             "d": data.hex()}
      out.write(json.dumps(rec) + "\n")
      recent.append(rec)
      if len(recent) > PRE_FRAMES and not post_left:
        recent.pop(0)

      if addr == PEDALS:
        now_acc = acc_active(data)
        if now_acc and not last_acc and not fired:
          fired = True
          post_left = POST_FRAMES
          print("\n  *** acc_active 0 -> 1 at t=%.3f -- capturing %d more frames"
                % (rec["t"], POST_FRAMES))
        last_acc = now_acc

      if post_left:
        post_left -= 1
        if post_left == 0:
          t0 = rec["t"]
          print("\n  === THE RADAR'S OWN FRAMES ACROSS ACTIVATION ===")
          for a, name in ((CRZ_INFO, "0x21b CRZ_INFO"), (CRZ_CTRL, "0x21c CRZ_CTRL")):
            seen: list[tuple[float, str]] = []
            for r in recent:
              if r["addr"] == a and (not seen or seen[-1][1] != r["d"]):
                seen.append((r["t"], r["d"]))
            print("\n  %s -- %d distinct values" % (name, len(seen)))
            for t, d in seen[-24:]:
              mark = "  <-- activation" if abs(t - t0) < 0.25 else ""
              print("    %+7.3f s  %s%s" % (t - t0, " ".join(
                  d[i:i + 2] for i in range(0, len(d), 2)), mark))
          print("\n  ours, for comparison (from tonight's alpha-long run):")
          print("    standby  0x21b 01 ff e3 ff c0 00 XX XX   0x21c 02 01 01 ...")
          print("    ready    0x21b 01 ff e2 00 04 80 XX XX   0x21c 02 01 0b ...")
          print("    engaged  0x21b 01 ff e1 80 06 80 XX XX   0x21c 0a 01 0b 20 00 00 10 00")
          print("\n  full capture: %s (%d frames)" % (args.out, n))
          out.close()
          return 0

  out.close()
  if not fired:
    print("\n  acc_active never went 1 -- stock ACC was not engaged during the "
          "window, so there is nothing to compare. Re-run and press SET while "
          "moving, off both pedals.")
  print("  wrote %d frames to %s" % (n, args.out))
  return 0


if __name__ == "__main__":
  sys.exit(main())
