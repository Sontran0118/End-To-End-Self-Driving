#!/usr/bin/env python3
"""Shim -> /home/tran/openpilot_jetson/curvature_lib.py

This used to be a byte-identical copy of the module in openpilot_jetson, and which
one got imported depended on sys.path ordering: every consumer here does
`sys.path.insert(0, "/home/tran/openpilot_jetson")`, which lands ahead of the script's
own directory, so openpilot_jetson's copy won -- but only by that accident of ordering.
Two copies of the function that decides how hard the car steers is not something to
leave to path order, so this file now just re-exports the live one.

If you want to fork the behaviour, fork it deliberately: edit openpilot_jetson's copy,
or replace this shim with real code and say why in a comment.
"""
import sys

_LIVE = "/home/tran/openpilot_jetson"
if _LIVE not in sys.path:
    sys.path.insert(0, _LIVE)

# Load from the file by absolute path rather than `from curvature_lib import *`, which
# would re-enter THIS module and quietly resolve to itself.
import importlib.util as _ilu

_spec = _ilu.spec_from_file_location("_curvature_lib_live", _LIVE + "/curvature_lib.py")
_mod = _ilu.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

T_IDXS = _mod.T_IDXS
ModelAction = _mod.ModelAction
# The action-head branch of modeld.get_action_from_model, for models that emit
# the command themselves ("Rebellious Hope" and later). op_stream picks between
# this and ModelAction on whether the loaded ONNX declares an `action` slice.
DirectAction = _mod.DirectAction
ACTION_WIDTH = _mod.ACTION_WIDTH
path_to_curvature = _mod.path_to_curvature
get_curvature_from_plan = _mod.get_curvature_from_plan
get_accel_from_plan = _mod.get_accel_from_plan
curv_from_psis = _mod.curv_from_psis
smooth_value = _mod.smooth_value
MAX_CURVATURE = _mod.MAX_CURVATURE
MIN_SPEED = _mod.MIN_SPEED
MIN_STABLE_DELAY = _mod.MIN_STABLE_DELAY
MIN_LAT_CONTROL_SPEED = _mod.MIN_LAT_CONTROL_SPEED
LAT_SMOOTH_SECONDS = _mod.LAT_SMOOTH_SECONDS
LONG_SMOOTH_SECONDS = _mod.LONG_SMOOTH_SECONDS

if __name__ == "__main__":
    print(f"shim -> {_LIVE}/curvature_lib.py ; run that file directly for its self-test")
