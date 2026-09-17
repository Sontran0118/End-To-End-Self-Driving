# Top-level `cereal` shim.
#
# opendbc's car/structs.py does:
#     try:
#       from cereal import car
#     except ImportError:
#       capnp.remove_import_hook()
#       car = capnp.load(.../car.capnp)
#
# In this tree cereal lives at `openpilot.cereal`, so that import fails and the
# fallback loads car.capnp a SECOND time -- pycapnp then aborts the process with
# "Duplicate ID @0x8e2af1e708af8b8d" (a hard abort, not a Python exception, so it
# cannot be caught). That killed anything importing both openpilot.cereal and
# opendbc.car.structs, e.g. hardwared -> alertmanager -> events.
#
# Re-export the already-loaded schema objects so structs.py takes the `try` branch
# and shares one load. Keep this a thin alias: no second capnp.load anywhere.
from openpilot.cereal import car, log, custom  # noqa: F401

__all__ = ["car", "log", "custom"]
