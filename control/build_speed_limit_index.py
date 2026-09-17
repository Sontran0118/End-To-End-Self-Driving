#!/usr/bin/env python3
"""Build a compact offline speed-limit index from an OSM .pbf extract.

RUN THIS ONCE, WITH THE OSM VENV, NOT SYSTEM PYTHON:

    /home/tran/osm_tools/bin/python build_speed_limit_index.py \\
        /home/tran/osm_data/new-york-latest.osm.pbf \\
        /home/tran/osm_data/speed_limits_ny.npz

pyosmium is deliberately confined to that venv: it is needed only to build the
index, and the control stack must not grow a dependency it cannot satisfy from
system python. The runtime side (`speed_limits.py`) loads the .npz with numpy
alone.

WHAT COMES OUT. Every drivable way is reduced to its individual segments -- a
pair of consecutive nodes -- and stored as flat numpy arrays:

    lat1 lon1 lat2 lon2   segment endpoints
    limit_mps             speed limit, metres/second
    inferred              1 if the limit came from the highway-type default
                          table rather than an explicit maxspeed tag
    hw_type               highway class code, for debugging and for letting a
                          consumer distinguish a motorway from a driveway

plus a cell index: segments are bucketed into a 0.01 degree grid (~1.1 km) so a
lookup touches a handful of candidates instead of millions.

WHY SEGMENTS RATHER THAN WAYS. A way can be kilometres long and bend through
many cells; matching a GPS point against whole ways gives wrong answers at every
curve. Segment-level matching lets the runtime compute a true perpendicular
distance to the road, which is what decides whether we are actually on it.

INFERRED LIMITS ARE MARKED, NOT HIDDEN. Most US residential streets carry no
maxspeed tag at all, so a defaults-by-highway-type table is the difference
between usable coverage and none. But a default is a guess about a road nobody
surveyed, and the consumer of this index is a throttle. The `inferred` flag is
carried all the way through so the control side can treat a guess differently
from a surveyed limit -- see `speed_limits.py`.
"""
from __future__ import annotations

import sys
import time

import numpy as np

try:
  import osmium
except ImportError:
  sys.exit("pyosmium missing. Run with /home/tran/osm_tools/bin/python")

# Ways we might drive on. Ordered roughly by importance; `service` is included
# because parking aisles and driveways are where a stop-and-go test happens, but
# it is given the lowest default and is easy to filter on hw_type downstream.
HIGHWAY_TYPES = [
  "motorway", "motorway_link", "trunk", "trunk_link",
  "primary", "primary_link", "secondary", "secondary_link",
  "tertiary", "tertiary_link", "unclassified", "residential",
  "living_street", "service",
]
HW_CODE = {name: i for i, name in enumerate(HIGHWAY_TYPES)}

# Fallback limits in mph, applied ONLY when maxspeed is absent. These are
# ordinary US defaults, not law for any particular jurisdiction, which is
# exactly why anything built from them is flagged `inferred`.
DEFAULT_MPH = {
  "motorway": 65, "motorway_link": 45,
  "trunk": 55, "trunk_link": 40,
  "primary": 45, "primary_link": 35,
  "secondary": 40, "secondary_link": 30,
  "tertiary": 35, "tertiary_link": 25,
  "unclassified": 30, "residential": 25,
  "living_street": 15, "service": 15,
}

MPH_TO_MPS = 0.44704
KPH_TO_MPS = 1.0 / 3.6
CELL_DEG = 0.01          # ~1.1 km in latitude


def parse_maxspeed(raw: str) -> float | None:
  """OSM maxspeed -> m/s, or None if it is not a usable number.

  Handles '55 mph', '50', '80 km/h', 'RU:urban'-style values (rejected), and the
  non-numeric sentinels ('signals', 'none', 'walk') that would otherwise parse
  as garbage. Returns None rather than guessing -- the caller falls back to the
  defaults table and marks the result inferred.
  """
  if not raw:
    return None
  s = raw.strip().lower()
  if s in ("none", "signals", "variable", "unposted"):
    return None
  if s == "walk":
    return 7 * MPH_TO_MPS
  mult = KPH_TO_MPS
  if "mph" in s:
    mult = MPH_TO_MPS
    s = s.replace("mph", "")
  elif "km/h" in s or "kmh" in s or "kph" in s:
    s = s.replace("km/h", "").replace("kmh", "").replace("kph", "")
  s = s.strip()
  # 'AT:urban', 'DE:motorway' and friends: a legal class, not a number.
  if not s or not s[0].isdigit():
    return None
  try:
    v = float(s.split()[0])
  except ValueError:
    return None
  if v <= 0 or v > 200:      # nonsense guard; 200 km/h covers any real posting
    return None
  return v * mult


class _Collector(osmium.SimpleHandler):
  def __init__(self):
    super().__init__()
    self.lat1: list[float] = []
    self.lon1: list[float] = []
    self.lat2: list[float] = []
    self.lon2: list[float] = []
    self.limit: list[float] = []
    self.inferred: list[int] = []
    self.hwtype: list[int] = []
    self.ways = 0
    self.tagged = 0
    self.t0 = time.time()

  def way(self, w):
    hw = w.tags.get("highway")
    if hw not in HW_CODE:
      return
    # A way closed to cars is worse than no data: it would hand the controller a
    # limit for a road it cannot be on.
    if w.tags.get("motor_vehicle") in ("no",) or w.tags.get("access") in ("no", "private"):
      return

    ms = parse_maxspeed(w.tags.get("maxspeed"))
    if ms is None:
      ms = DEFAULT_MPH[hw] * MPH_TO_MPS
      inferred = 1
    else:
      inferred = 0
      self.tagged += 1

    code = HW_CODE[hw]
    prev = None
    for n in w.nodes:
      if not n.location.valid():
        prev = None
        continue
      cur = (n.location.lat, n.location.lon)
      if prev is not None:
        self.lat1.append(prev[0]); self.lon1.append(prev[1])
        self.lat2.append(cur[0]);  self.lon2.append(cur[1])
        self.limit.append(ms)
        self.inferred.append(inferred)
        self.hwtype.append(code)
      prev = cur

    self.ways += 1
    if self.ways % 200000 == 0:
      print("  %d ways, %d segments, %.0f s"
            % (self.ways, len(self.limit), time.time() - self.t0), flush=True)


def main() -> int:
  if len(sys.argv) < 3:
    sys.exit("usage: build_speed_limit_index.py IN.osm.pbf OUT.npz")
  src, dst = sys.argv[1], sys.argv[2]

  print("reading %s ..." % src, flush=True)
  h = _Collector()
  # locations=True makes osmium resolve node coordinates for way members;
  # flex_mem keeps the node cache in RAM with a sparse fallback, which is what
  # fits a state-sized extract on this box.
  h.apply_file(src, locations=True, idx="flex_mem")

  n = len(h.limit)
  if n == 0:
    sys.exit("no drivable ways found -- wrong extract?")
  print("  %d ways -> %d segments (%d ways had an explicit maxspeed, %.1f%%)"
        % (h.ways, n, h.tagged, 100.0 * h.tagged / max(1, h.ways)), flush=True)

  lat1 = np.asarray(h.lat1, dtype=np.float32)
  lon1 = np.asarray(h.lon1, dtype=np.float32)
  lat2 = np.asarray(h.lat2, dtype=np.float32)
  lon2 = np.asarray(h.lon2, dtype=np.float32)
  limit = np.asarray(h.limit, dtype=np.float32)
  inferred = np.asarray(h.inferred, dtype=np.uint8)
  hwtype = np.asarray(h.hwtype, dtype=np.uint8)

  # --- grid index --------------------------------------------------------
  # Bucket by the segment MIDPOINT. A segment longer than a cell could then be
  # missed from a neighbouring cell, so the runtime searches a 3x3 block and the
  # builder splits nothing -- at 0.01 deg the vast majority of OSM segments are
  # far shorter than one cell, and the 3x3 search covers the rest.
  midlat = (lat1 + lat2) * 0.5
  midlon = (lon1 + lon2) * 0.5
  cy = np.floor(midlat / CELL_DEG).astype(np.int32)
  cx = np.floor(midlon / CELL_DEG).astype(np.int32)

  order = np.lexsort((cx, cy))
  cy_s, cx_s = cy[order], cx[order]
  # Unique cells and where each starts in the sorted order -> the runtime does a
  # binary search for the cell key and slices, no dict and no per-query hashing.
  keys = (cy_s.astype(np.int64) << 32) | (cx_s.astype(np.int64) & 0xFFFFFFFF)
  uniq, starts = np.unique(keys, return_index=True)
  counts = np.diff(np.append(starts, len(keys)))

  print("  %d grid cells at %.3f deg" % (len(uniq), CELL_DEG), flush=True)

  np.savez_compressed(
    dst,
    lat1=lat1[order], lon1=lon1[order], lat2=lat2[order], lon2=lon2[order],
    limit=limit[order], inferred=inferred[order], hwtype=hwtype[order],
    cell_key=uniq, cell_start=starts.astype(np.int64), cell_count=counts.astype(np.int32),
    cell_deg=np.float64(CELL_DEG),
    highway_types=np.array(HIGHWAY_TYPES),
  )
  import os
  print("wrote %s (%.1f MB) in %.0f s"
        % (dst, os.path.getsize(dst) / 1e6, time.time() - h.t0))
  return 0


if __name__ == "__main__":
  sys.exit(main())
