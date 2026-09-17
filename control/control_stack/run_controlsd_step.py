#!/usr/bin/env python3
"""END-TO-END compute test: drive controlsd through one control step with
synthetic inputs and read the steering command it produces.

Publishes all of controlsd's SubMaster inputs (carState, modelV2, plans, etc.),
sets it engaged with a curved desired path, runs state_control(), and prints
the actuator output. Proves the FULL control loop computes on the Jetson.

No car, no transmit."""
import os, time, math
os.environ.setdefault("PARAMS_ROOT", "/tmp/op_params")

from openpilot.common.params import Params
import openpilot.cereal.messaging as messaging
from opendbc.car.structs import car as car_struct
from opendbc.car.mazda.values import CAR
from opendbc.car.mazda.interface import CarInterface

# --- CarParams for CX-5 ---
CP = CarInterface.get_non_essential_params(CAR.MAZDA_CX5_2022)
Params().put("CarParams", CP.to_bytes())

from openpilot.selfdrive.controls.controlsd import Controls
from cereal import log

# --- publishers for everything controlsd subscribes to ---
topics = ['liveDelay', 'liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
          'liveCalibration', 'livePose', 'longitudinalPlan', 'lateralManeuverPlan',
          'carState', 'carOutput', 'driverMonitoringState', 'onroadEvents', 'driverAssistance']
pm = messaging.PubMaster(topics)

controls = Controls()
time.sleep(0.5)


def pub(name, fill):
    if name == "onroadEvents":
        m = messaging.new_message(name, 0)
    else:
        m = messaging.new_message(name)
        fill(getattr(m, name))
    pm.send(name, m)


# --- fill each message with plausible engaged, moving, curving-road state ---
V = 20.0  # 20 m/s ~ 72 km/h (above Mazda's 52 kph LKAS enable)

def f_carState(cs):
    cs.vEgo = V
    cs.vEgoRaw = V
    cs.steeringAngleDeg = 2.0
    cs.steeringTorque = 0.0
    cs.steeringPressed = False
    cs.gearShifter = car_struct.CarState.GearShifter.drive
    cs.cruiseState.enabled = True
    cs.cruiseState.available = True

def f_liveParameters(lp):
    lp.steerRatio = 15.5
    lp.stiffnessFactor = 1.0
    lp.angleOffsetDeg = 0.0
    lp.roll = 0.0
    lp.valid = True

def f_liveDelay(ld):
    try: ld.lateralDelay = 0.2
    except Exception: pass

def f_liveTorqueParameters(tp):
    tp.useParams = True
    # MAZDA_CX9_2021 tune (CX5_2022 substitutes to it). Was 2.5 / 0.1.
    tp.latAccelFactorFiltered = 1.7601682915983443
    tp.latAccelOffsetFiltered = 0.0
    tp.frictionCoefficientFiltered = 0.17713792194297195

def f_modelV2(md):
    md.frameId = 1
    # a gently curving path to the LEFT -> should command left steering
    N = 33
    pos = md.position
    pos.x = [float(i * V * 0.1) for i in range(N)]         # forward
    pos.y = [float(0.002 * (i ** 2)) for i in range(N)]     # curve left
    pos.z = [0.0] * N
    # orientation / curvature hints if present
    try:
        md.action.desiredCurvature = 0.01
    except Exception:
        pass

def f_selfdriveState(ss):
    ss.enabled = True
    ss.active = True
    ss.state = log.SelfdriveState.OpenpilotState.enabled

def f_liveCalibration(lc):
    lc.rpyCalib = [0.0, 0.0, 0.0]
    try: lc.calStatus = log.LiveCalibrationData.Status.calibrated
    except Exception: pass

def f_livePose(lpz):
    try:
        lpz.orientationNED.x = 0.0
    except Exception:
        pass

def f_longitudinalPlan(lpn):
    pass

def f_lateralManeuverPlan(lm):
    pass

def f_carOutput(co):
    pass

def f_driverMonitoringState(dm):
    try: dm.awarenessStatus = 1.0
    except Exception: pass

def f_onroadEvents(oe):
    pass

def f_driverAssistance(da):
    pass

fills = {
    'carState': f_carState, 'liveParameters': f_liveParameters, 'liveDelay': f_liveDelay,
    'liveTorqueParameters': f_liveTorqueParameters, 'modelV2': f_modelV2,
    'selfdriveState': f_selfdriveState, 'liveCalibration': f_liveCalibration,
    'livePose': f_livePose, 'longitudinalPlan': f_longitudinalPlan,
    'lateralManeuverPlan': f_lateralManeuverPlan, 'carOutput': f_carOutput,
    'driverMonitoringState': f_driverMonitoringState, 'onroadEvents': f_onroadEvents,
    'driverAssistance': f_driverAssistance,
}

# publish a few rounds so SubMaster marks everything valid/updated
for _ in range(5):
    for name, fill in fills.items():
        pub(name, fill)
    time.sleep(0.05)
    controls.sm.update(10)

print("engaged:", controls.sm['selfdriveState'].enabled)
print("vEgo:", controls.sm['carState'].vEgo, "m/s")

# run the control computation
CC, lac_log = controls.state_control()

act = CC.actuators
print("\n=== controlsd produced a CarControl ===")
print("  latActive:", CC.latActive)
print("  torque:            %.4f" % act.torque); print("  torqueOutputCan:   %.4f" % act.torqueOutputCan)
print("  steeringAngleDeg:  %.3f" % act.steeringAngleDeg)
print("  curvature:         %.5f" % act.curvature)
print("\n=== CONTROL LOOP COMPUTES END-TO-END (no transmit) ===")
