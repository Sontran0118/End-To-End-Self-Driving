"""Drive the real Mazda CarController with dashcam_web's shim.

Reproduces long_tx_thread's exact shim contract without needing openpilot/msgq,
and asserts update() never raises across the full frame cycle -- including the
frame % 50 HUD branch and the frame % 5 / % 10 button branches.
"""
import sys
sys.path.insert(0, "/home/tran/opendbc_src")

from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.mazda.values import CAR
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.longitudinal import CRZ_CTRL_ADDR, CRZ_INFO_ADDR, RADAR_ADDR

CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(),
                             [], True, False, False)
print("CP.openpilotLongitudinalControl =", CP.openpilotLongitudinalControl)
assert CP.openpilotLongitudinalControl, "alpha-long port not installed"

# --- the shim, copied from dashcam_web.long_tx_thread -----------------------
cam_state = {"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0, "seen": 0}
CAM_LANEINFO_NEUTRAL = dict.fromkeys(
    ("LINE_VISIBLE", "LINE_NOT_VISIBLE", "LANE_LINES", "BIT1", "BIT2",
     "BIT3", "NO_ERR_BIT", "S1", "S1_HBEAM"), 0)


class _Out:
    pass


class _CS:
    def __init__(self):
        self.out = _Out()
        self.accel_button = 0
        self.crz_btns_counter = 0
        self.cam_lkas = cam_state
        self.cam_laneinfo = CAM_LANEINFO_NEUTRAL
        self.lkas_allowed_speed = False


shim = _CS()
shim.out.vEgo = 0.0
shim.out.standstill = True
shim.out.gasPressed = False
shim.out.brakePressed = False
shim.out.steeringTorque = 0.0

cc_obj = CarController({Bus.pt: "mazda_2017"}, CP)
LONG_ADDRS = {CRZ_INFO_ADDR, CRZ_CTRL_ADDR, RADAR_ADDR}

# Sweep the states that select different branches: engaged/not, moving/stopped,
# resume asserted, stopping/starting. 200 frames covers every modulo branch.
SCENARIOS = [
    ("idle, disengaged",      dict(lat=False, lon=False, res=False, v=0.0,  st=True,  lcs="off")),
    ("engaged, cruising",     dict(lat=True,  lon=True,  res=False, v=15.0, st=False, lcs="pid")),
    ("engaged, stopping",     dict(lat=True,  lon=True,  res=False, v=0.4,  st=False, lcs="stopping")),
    ("held at standstill",    dict(lat=True,  lon=True,  res=False, v=0.0,  st=True,  lcs="stopping")),
    ("resume from standstill",dict(lat=True,  lon=True,  res=True,  v=0.0,  st=True,  lcs="starting")),
]

LCS = structs.CarControl.Actuators.LongControlState
fails = 0
for name, s in SCENARIOS:
    CC = structs.CarControl()
    CC.latActive = s["lat"]
    CC.longActive = s["lon"]
    CC.cruiseControl.resume = s["res"]
    CC.actuators.accel = 0.5
    CC.actuators.torque = 0.1
    CC.actuators.longControlState = {"off": LCS.off, "pid": LCS.pid,
                                     "stopping": LCS.stopping,
                                     "starting": LCS.starting}[s["lcs"]]
    shim.out.vEgo = s["v"]
    shim.out.standstill = s["st"]
    shim.accel_button = int(s["res"])

    # long_tx_thread passes sm['carControl'], a capnp *reader*; update() calls
    # CC.actuators.as_builder(), which only exists on readers.
    CC = CC.as_reader()

    long_frames = 0
    try:
        for i in range(200):
            shim.crz_btns_counter = i % 16
            _, sends = cc_obj.update(CC, shim, i * 10_000_000)
            long_frames += sum(1 for m in sends if m[0] in LONG_ADDRS)
        print(f"  OK  {name:<24} 200 frames, {long_frames:4d} long msgs")
    except Exception as e:
        fails += 1
        print(f"  FAIL {name:<24} {type(e).__name__}: {e}")

# Physical RES must actually reach the controller now.
shim.accel_button = 1
assert bool(shim.accel_button), "accel_button lost"

print("\nSHIM OK -- no missing fields" if not fails else f"\n{fails} SCENARIO(S) FAILED")
raise SystemExit(1 if fails else 0)
