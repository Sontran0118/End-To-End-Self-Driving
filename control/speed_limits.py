#!/usr/bin/env python3
"""Offline speed-limit lookup from a GPS position.

Loads the .npz built by build_speed_limit_index.py and answers "what is the
limit on the road I am on?" with numpy only -- no pyosmium, no network, no
per-query latency worth measuring. See that file for how the index is built.

MATCHING. The nearest road is not the road you are on: a service alley runs
parallel to the road at 8 m, and an overpass crosses it at 0 m. So the match
uses three filters together, and reports None when they disagree:

  distance  perpendicular distance to the segment, not to its endpoints
  heading   the segment must run roughly the way we are travelling
  class     a motorway match is preferred over a service road at similar range

WHEN IT RETURNS None IT MEANS None. There is no last-known-limit latch here, on
purpose. `set_speed_raw` in dashcam.py latches forever and an evening was lost
to reading a frozen value as live; a stale speed limit is worse, because it
feeds a throttle. If the position is old, or no segment matches, the caller gets
None and must decide for itself -- see speed_target.py, which holds the previous
limit for a bounded time and then gives up.
"""
from __future__ import annotations

import math
import os

import numpy as np

DEFAULT_INDEX = os.environ.get("OP_SPEED_LIMIT_INDEX",
                               "/home/tran/osm_data/speed_limits_ny.npz")

# Perpendicular distance beyond which a segment is not the road we are on.
# 25 m is wide enough for a divided highway's far carriageway to still match
# its own side, and narrow enough to reject the frontage road in most cases.
MAX_MATCH_M = 25.0

# Heading agreement. OSM segments are directed but two-way roads are driven both
# ways, so compare modulo 180 degrees.
MAX_HEADING_ERR_DEG = 50.0

# A road one class up is preferred even if slightly further away: at a junction
# the residential stub is often nearer than the arterial we are actually on.
# Metres of "credit" per class step (motorway=0 ... service=13).
CLASS_CREDIT_M = 1.2


class SpeedLimitIndex:
  def __init__(self, path: str = DEFAULT_INDEX):
    self.path = path
    self.ok = False
    self.error = ""
    try:
      z = np.load(path, allow_pickle=False)
    except Exception as e:
      self.error = "load %s: %s" % (path, e)
      return
    self.lat1 = z["lat1"]; self.lon1 = z["lon1"]
    self.lat2 = z["lat2"]; self.lon2 = z["lon2"]
    self.limit = z["limit"]; self.inferred = z["inferred"]; self.hwtype = z["hwtype"]
    self.cell_key = z["cell_key"]; self.cell_start = z["cell_start"]; self.cell_count = z["cell_count"]
    self.cell_deg = float(z["cell_deg"])
    self.highway_types = [str(s) for s in z["highway_types"]]
    self.ok = True

  # ---- internals -----------------------------------------------------------
  def _cell_slice(self, cy: int, cx: int):
    key = (np.int64(cy) << 32) | (np.int64(cx) & 0xFFFFFFFF)
    i = np.searchsorted(self.cell_key, key)
    if i >= len(self.cell_key) or self.cell_key[i] != key:
      return None
    s = int(self.cell_start[i])
    return s, s + int(self.cell_count[i])

  def _candidates(self, lat: float, lon: float):
    """Indices of segments in the 3x3 cell block around the point.

    3x3 rather than 1x1 because segments are bucketed by midpoint: one longer
    than a cell can have its midpoint next door to the point that lies on it.
    """
    cy = int(math.floor(lat / self.cell_deg))
    cx = int(math.floor(lon / self.cell_deg))
    parts = []
    for dy in (-1, 0, 1):
      for dx in (-1, 0, 1):
        sl = self._cell_slice(cy + dy, cx + dx)
        if sl:
          parts.append(np.arange(sl[0], sl[1]))
    if not parts:
      return None
    return np.concatenate(parts)

  # ---- query ---------------------------------------------------------------
  def lookup(self, lat: float, lon: float, heading_deg: float | None = None) -> dict | None:
    """Nearest matching road segment, or None.

    Returns {limit_mps, inferred, highway, distance_m, heading_err_deg}.
    """
    if not self.ok:
      return None
    idx = self._candidates(lat, lon)
    if idx is None or len(idx) == 0:
      return None

    # Local equirectangular projection: at a single GPS point the error over a
    # few hundred metres is far below the matching threshold, and it keeps the
    # whole query in cheap vector arithmetic.
    mlat = math.radians(lat)
    m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * mlat) + 1.175 * math.cos(4 * mlat)
    m_per_deg_lon = 111412.84 * math.cos(mlat) - 93.5 * math.cos(3 * mlat)

    x1 = (self.lon1[idx].astype(np.float64) - lon) * m_per_deg_lon
    y1 = (self.lat1[idx].astype(np.float64) - lat) * m_per_deg_lat
    x2 = (self.lon2[idx].astype(np.float64) - lon) * m_per_deg_lon
    y2 = (self.lat2[idx].astype(np.float64) - lat) * m_per_deg_lat

    # Perpendicular distance from the origin (our position) to each segment,
    # clamped to the segment rather than the infinite line.
    dx = x2 - x1
    dy = y2 - y1
    seg_len2 = dx * dx + dy * dy
    seg_len2 = np.where(seg_len2 < 1e-9, 1e-9, seg_len2)
    t = np.clip(-(x1 * dx + y1 * dy) / seg_len2, 0.0, 1.0)
    px = x1 + t * dx
    py = y1 + t * dy
    dist = np.sqrt(px * px + py * py)

    score = dist + CLASS_CREDIT_M * self.hwtype[idx].astype(np.float64)

    # Heading filter. Segment bearing is measured east-of-north to match the
    # NMEA course, and compared modulo 180 so the other direction of a two-way
    # road is not rejected.
    herr = np.zeros_like(dist)
    if heading_deg is not None:
      seg_bearing = np.degrees(np.arctan2(dx, dy)) % 360.0
      d = np.abs((seg_bearing - heading_deg + 180.0) % 360.0 - 180.0)
      herr = np.minimum(d, 180.0 - d)
      score = np.where(herr > MAX_HEADING_ERR_DEG, np.inf, score)

    best = int(np.argmin(score))
    if not np.isfinite(score[best]) or dist[best] > MAX_MATCH_M:
      return None

    j = int(idx[best])
    return {
      "limit_mps": float(self.limit[j]),
      "inferred": bool(self.inferred[j]),
      "highway": self.highway_types[int(self.hwtype[j])],
      "distance_m": float(dist[best]),
      "heading_err_deg": float(herr[best]) if heading_deg is not None else None,
    }

  def status(self) -> dict:
    if not self.ok:
      return {"ok": False, "error": self.error}
    return {"ok": True, "segments": int(len(self.limit)),
            "cells": int(len(self.cell_key)), "path": self.path}


if __name__ == "__main__":
  import sys
  ix = SpeedLimitIndex()
  print("index:", ix.status())
  if not ix.ok:
    sys.exit(1)
  if len(sys.argv) >= 3:
    la, lo = float(sys.argv[1]), float(sys.argv[2])
    hd = float(sys.argv[3]) if len(sys.argv) > 3 else None
    print("lookup %.6f, %.6f hdg=%s -> %s" % (la, lo, hd, ix.lookup(la, lo, hd)))
  else:
    # Live: read the GPS and print what road we are on.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import time

    from gps_reader import GpsReader
    g = GpsReader().start()
    print("live lookup, ctrl-C to stop")
    try:
      while True:
        time.sleep(1.0)
        fx = g.fix()
        if not fx:
          print("  no fix")
          continue
        r = ix.lookup(fx["lat"], fx["lon"], fx["heading_deg"])
        if r:
          print("  %.6f,%.6f -> %-14s %5.1f mph%s  %.1f m away"
                % (fx["lat"], fx["lon"], r["highway"], r["limit_mps"] / 0.44704,
                   "  (inferred)" if r["inferred"] else "", r["distance_m"]))
        else:
          print("  %.6f,%.6f -> no road matched" % (fx["lat"], fx["lon"]))
    except KeyboardInterrupt:
      g.stop()
