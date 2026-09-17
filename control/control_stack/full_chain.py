#!/usr/bin/env python3
"""FULL CHAIN, live: IMX477 -> supercombo -> proper curvature -> modelV2 on bus
-> controlsd -> steering command. Everything real, nothing transmitted to the car.

This is the complete openpilot lateral pipeline running on the Jetson for the CX-5.
Usage: python3 full_chain.py [--synthetic] [-n N]
"""
import sys, os, time, math
os.environ.setdefault("PARAMS_ROOT", "/tmp/op_params")
sys.path.insert(0, "/home/tran/openpilot_jetson")

import numpy as np
import openpilot.cereal.messaging as messaging
from curvature_lib import path_to_curvature, T_IDXS
from opendbc.car.mazda.values import CAR
from opendbc.car.mazda.interface import CarInterface
from openpilot.common.params import Params
from cereal import log
from opendbc.car.structs import car as car_struct

# --- car params ---
CP = CarInterface.get_non_essential_params(CAR.MAZDA_CX5_2022)
Params().put("CarParams", CP.to_bytes())
from openpilot.selfdrive.controls.controlsd import Controls

synthetic = "--synthetic" in sys.argv
n = 5
if "-n" in sys.argv:
    n = int(sys.argv[sys.argv.index("-n")+1])

runner = None
if not synthetic:
    from op_stream import SupercomboRunner
    runner = SupercomboRunner()
    print("supercombo loaded — FULL LIVE CHAIN")
else:
    print("synthetic mode")

# publishers for ALL controlsd inputs
topics = ['liveDelay','liveParameters','liveTorqueParameters','modelV2','selfdriveState',
          'liveCalibration','livePose','longitudinalPlan','lateralManeuverPlan',
          'carState','carOutput','driverMonitoringState','onroadEvents','driverAssistance']
pm = messaging.PubMaster(topics)
controls = Controls()
time.sleep(0.5)

V = 20.0
frame_id = 0

_CAM = None


def grab():
    """Persistent 1080p NV12 capture (see supercombo_publisher.grab_frame for why).

    The old per-frame `gst-launch ... num-buffers=1 ! jpegenc ! filesink` + imread
    cost ~15s a frame, captured at 720p (focal 709 vs medmodel's 910, i.e. the road
    branch upsampled 0.78x), and put a lossy JPEG round-trip in front of the model."""
    global _CAM
    if _CAM is None:
        # Road-metered AE, not op_camera.Camera: Argus meters the whole frame, so a
        # bright sky drags the road down (measured sky p50 184 / road p50 69 in one
        # frame whose overall mean looked fine at 101). CameraAE leaves Argus's fast
        # loop alone and re-aims it with a slow exposurecompensation loop over the
        # road band only.
        from op_camera_ae import CameraAE
        _CAM = CameraAE(auto_exposure=True)
    return _CAM.read()

def publish_all(out, curv, act=None):
    global frame_id
    def send(name, fn):
        if name == "onroadEvents":
            m = messaging.new_message(name, 0)
        else:
            m = messaging.new_message(name); fn(getattr(m, name))
        pm.send(name, m)

    def cs(x):
        x.vEgo=V; x.vEgoRaw=V; x.steeringAngleDeg=0.0; x.steeringPressed=False
        x.gearShifter=car_struct.CarState.GearShifter.drive
        x.cruiseState.enabled=True; x.cruiseState.available=True
    def lp(x):
        x.steerRatio=15.5; x.stiffnessFactor=1.0; x.angleOffsetDeg=0.0; x.roll=0.0; x.valid=True
    def tp(x):
        # MAZDA_CX9_2021 tune (CX5_2022 substitutes to it). Was 2.5 / 0.1.
        x.useParams=True; x.latAccelFactorFiltered=1.7601682915983443; x.latAccelOffsetFiltered=0.0; x.frictionCoefficientFiltered=0.17713792194297195
    def ss(x):
        x.enabled=True; x.active=True; x.state=log.SelfdriveState.OpenpilotState.enabled
    def lc(x):
        x.rpyCalib=[0.0,0.0,0.0]
    def mv(md):
        global frame_id
        md.frameId=frame_id; frame_id+=1
        path=out.get("path_xyz")
        if path is not None:
            md.position.x=[float(path[i][0]) for i in range(33)]
            md.position.y=[float(path[i][1]) for i in range(33)]
            md.position.z=[float(path[i][2]) for i in range(33)]
            md.position.t=[float(t) for t in T_IDXS]
        try:
            md.action.desiredCurvature=float(curv)
            # The other two fields modeld fills on this message. On the current
            # weights they are the model's own action head, not a re-derivation
            # from the plan; publishing them keeps modelV2 here meaning what it
            # means on a comma three even though this port steers only.
            if act is not None:
                md.action.desiredAcceleration=float(act["desiredAcceleration"])
                md.action.shouldStop=bool(act["shouldStop"])
        except Exception: pass

    send('carState', cs); send('liveParameters', lp); send('liveTorqueParameters', tp)
    send('selfdriveState', ss); send('liveCalibration', lc); send('modelV2', mv)
    for t in ('liveDelay','livePose','longitudinalPlan','lateralManeuverPlan',
              'carOutput','driverMonitoringState','onroadEvents','driverAssistance'):
        send(t, lambda x: None)

print("\n%-6s %-12s %-12s %s" % ("frame","path_x_end","curvature","steer_torque"))
print("-"*52)
for i in range(n):
    if synthetic:
        out = {"path_xyz": np.array([[j*V*0.1, 0.002*j*j, 0.0] for j in range(33)])}
    else:
        f = grab()
        if f is None: print("no frame"); continue
        # v_ego is required for the stock action (desiredCurvature = psi/(v*t)); this
        # harness has no CAN, so V is its stated constant-speed assumption.
        out = runner.step(f, v_ego=V)

    # STOCK when we have a real action; legacy fit only for --synthetic, which fakes
    # path_xyz alone and has neither an action head nor plan columns.
    act = out.get("action")
    curv = (act["desiredCurvature"] if act is not None
            else path_to_curvature(out.get("path_xyz"), V))

    for _ in range(4):
        publish_all(out, curv, act)
        time.sleep(0.03)
        controls.sm.update(10)

    CC, lac_log = controls.state_control()
    px_end = float(out["path_xyz"][-1][0])
    print("%-6d %-12.2f %-12.5f %.4f" % (i, px_end, curv, CC.actuators.torque))

if _CAM is not None:
    _CAM.close()          # the sensor stays open now, so it has to be released

print("\n=== FULL LIVE CHAIN: camera -> supercombo -> curvature -> controlsd -> torque ===")
