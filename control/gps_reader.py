#!/usr/bin/env python3
"""NMEA reader for the Jetson's GPS, as a background thread with a latest-fix cache.

MEASURED 2026-08-12 on this unit: /dev/ttyTHS1 at 38400 (NOT the 9600 default --
9600 returns framing garbage on this port, which is what "no GPS" looked like the
first time). Multi-constellation GPS + Galileo + BeiDou + QZSS, 11 satellites
used, HDOP 1.19, PDOP 2.36, 3D fix, ~14 sentences/s.

IT IS SILENT FOR THE FIRST MINUTE OR SO after power-up. The first probe of the
evening read zero bytes at every baud and looked like a dead module; the same
port read a valid fix minutes later. So absence of data at startup means "wait",
not "broken", and nothing downstream should treat a missing fix as a fault.

WHY A LATCH IS THE WRONG DEFAULT HERE. `set_speed_raw` in dashcam.py latches its
last CAN value forever and never invalidates, and a whole evening was lost to
reading a frozen field as a live one. This class does the opposite: `fix()`
returns None once the data is older than MAX_AGE_S, so a consumer cannot mistake
a stale position for a current one. A speed limit looked up from a minute-old
position is worse than no speed limit at all.
"""
from __future__ import annotations

import math
import threading
import time

DEFAULT_PORT = "/dev/ttyTHS1"
DEFAULT_BAUD = 38400

# Older than this and fix() reports None. The receiver emits at ~1 Hz per
# sentence type, so 3 s is three missed updates -- long enough not to flap,
# short enough that a consumer never acts on a position from a different road.
MAX_AGE_S = 3.0


def _dm_to_deg(value: str, hemi: str) -> float | None:
  """NMEA ddmm.mmmm -> signed decimal degrees."""
  if not value or not hemi:
    return None
  try:
    v = float(value)
  except ValueError:
    return None
  deg = int(v / 100.0)
  minutes = v - deg * 100.0
  out = deg + minutes / 60.0
  return -out if hemi in ("S", "W") else out


def _nmea_checksum_ok(line: str) -> bool:
  """Verify the *checksum, and require it.

  A corrupted position is far more dangerous than a missing one when the number
  feeds a speed target, and NMEA gives us a cheap integrity check -- so use it
  rather than trusting the line because it starts with a '$'.
  """
  if "*" not in line:
    return False
  body, _, csum = line[1:].partition("*")
  try:
    want = int(csum[:2], 16)
  except ValueError:
    return False
  got = 0
  for ch in body:
    got ^= ord(ch)
  return got == want


class GpsReader:
  """Reads NMEA in a daemon thread; `fix()` returns the latest or None."""

  def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD):
    self.port = port
    self.baud = baud
    self._lock = threading.Lock()
    self._fix: dict | None = None
    self._t = 0.0
    self._stop = False
    self.sentences = 0
    self.checksum_errors = 0
    self.last_error = ""
    self._thread: threading.Thread | None = None

  # ---- lifecycle -----------------------------------------------------------
  def start(self) -> "GpsReader":
    self._thread = threading.Thread(target=self._run, daemon=True)
    self._thread.start()
    return self

  def stop(self) -> None:
    self._stop = True

  # ---- access --------------------------------------------------------------
  def fix(self) -> dict | None:
    """Latest fix, or None if there is none or it is stale.

    Keys: lat, lon, speed_mps, heading_deg (may be None when stationary),
    sats, hdop, quality, alt_m, t.
    """
    with self._lock:
      if self._fix is None:
        return None
      if (time.time() - self._t) > MAX_AGE_S:
        return None
      return dict(self._fix)

  def age(self) -> float:
    """Seconds since the last accepted sentence; -1 if never."""
    with self._lock:
      return (time.time() - self._t) if self._t else -1.0

  def status(self) -> dict:
    f = self.fix()
    return {"have_fix": f is not None,
            "age_s": round(self.age(), 2),
            "sentences": self.sentences,
            "checksum_errors": self.checksum_errors,
            "sats": (f or {}).get("sats"),
            "hdop": (f or {}).get("hdop"),
            "last_error": self.last_error}

  # ---- internals -----------------------------------------------------------
  def _run(self) -> None:
    import serial
    ser = None
    partial: dict = {}
    while not self._stop:
      if ser is None:
        try:
          ser = serial.Serial(self.port, self.baud, timeout=1.0)
          self.last_error = ""
        except Exception as e:
          # Reopen rather than give up: the port disappears across a Jetson
          # suspend and across some USB resets, and a reader that dies on the
          # first error is a reader that is dead for the rest of the drive.
          self.last_error = "open: %s" % e
          time.sleep(2.0)
          continue
      try:
        raw = ser.readline()
      except Exception as e:
        self.last_error = "read: %s" % e
        try:
          ser.close()
        except Exception:
          pass
        ser = None
        continue
      if not raw:
        continue
      try:
        line = raw.decode("ascii", "ignore").strip()
      except Exception:
        continue
      if not line.startswith("$"):
        continue
      if not _nmea_checksum_ok(line):
        self.checksum_errors += 1
        continue
      self.sentences += 1
      self._consume(line, partial)

  def _consume(self, line: str, partial: dict) -> None:
    f = line.split("*")[0].split(",")
    kind = f[0][3:] if len(f[0]) >= 6 else ""

    # GGA carries fix quality, satellite count, HDOP and altitude.
    if kind == "GGA" and len(f) > 9:
      try:
        quality = int(f[6]) if f[6] else 0
      except ValueError:
        quality = 0
      if quality == 0:
        # No fix: drop whatever we had rather than keep serving it.
        with self._lock:
          self._fix = None
        return
      lat = _dm_to_deg(f[2], f[3])
      lon = _dm_to_deg(f[4], f[5])
      if lat is None or lon is None:
        return
      partial.update({
        "lat": lat, "lon": lon, "quality": quality,
        "sats": int(f[7]) if f[7] else 0,
        "hdop": float(f[8]) if f[8] else float("nan"),
        "alt_m": float(f[9]) if f[9] else 0.0,
      })
      self._publish(partial)

    # RMC carries ground speed and course. Speed is in KNOTS.
    elif kind == "RMC" and len(f) > 8:
      if f[2] != "A":          # V = warning, data not valid
        return
      try:
        knots = float(f[7]) if f[7] else 0.0
      except ValueError:
        knots = 0.0
      heading = None
      if f[8]:
        try:
          heading = float(f[8])
        except ValueError:
          heading = None
      # Course over ground is meaningless at a standstill and the receiver
      # reports whatever it last had, so do not publish it as current.
      partial["speed_mps"] = knots * 0.514444
      partial["heading_deg"] = heading if knots > 0.5 else None
      if "lat" in partial:
        self._publish(partial)

  def _publish(self, partial: dict) -> None:
    if "lat" not in partial or "lon" not in partial:
      return
    out = dict(partial)
    out.setdefault("speed_mps", 0.0)
    out.setdefault("heading_deg", None)
    out["t"] = time.time()
    with self._lock:
      self._fix = out
      self._t = out["t"]


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  """Great-circle distance in metres. Used for road-segment matching."""
  r = 6371000.0
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp = math.radians(lat2 - lat1)
  dl = math.radians(lon2 - lon1)
  a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
  return 2 * r * math.asin(math.sqrt(a))


if __name__ == "__main__":
  g = GpsReader().start()
  print("reading %s at %d ... ctrl-C to stop" % (g.port, g.baud))
  try:
    while True:
      time.sleep(1.0)
      fx = g.fix()
      if fx:
        print("  %.6f, %.6f  %5.1f kph  hdg %s  sats %s hdop %.2f  (age %.1fs)"
              % (fx["lat"], fx["lon"], fx["speed_mps"] * 3.6,
                 ("%3.0f" % fx["heading_deg"]) if fx["heading_deg"] is not None else " --",
                 fx["sats"], fx["hdop"], g.age()))
      else:
        print("  no fix  (%s)" % g.status())
  except KeyboardInterrupt:
    g.stop()
