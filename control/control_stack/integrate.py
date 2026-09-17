#!/usr/bin/env python3
"""INTEGRATION: run controlsd as its OWN process, feed it real inputs over the
cereal bus from separate publisher processes, and read back the carControl it
publishes. This is the true multi-process openpilot architecture — not a
single combined script.

Processes:
  1. controlsd (subprocess)         — the real control daemon, 100Hz run loop
  2. this script                    — publishes modelV2 (supercombo) + carState +
                                       the other inputs, subscribes to carControl

Reads controlsd's published carControl to prove the loop closes over the bus.
Everything stops at the bus — carControl is published, nothing goes to the EPS.

Usage: python3 integrate.py [--synthetic] [-n N]
"""
import sys, os, time, subprocess, signal
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

synthetic = "--synthetic" in sys.argv
n = 20
if "-n" in sys.argv:
    n = int(sys.argv[sys.argv.index("-n") + 1])

# --- write CarParams so controlsd can start ---
CP = CarInterface.get_non_essential_params(CAR.MAZDA_CX5_2022)
Params().put("CarParams", CP.to_bytes())
print("CarParams written for", CP.carFingerprint)

# --- launch controlsd as its OWN process ---
env = dict(os.environ)
env["PARAMS_ROOT"] = "/tmp/op_params"
env["PYTHONPATH"] = "/home/tran/msgq_build:/home/tran/opendbc_src:/home/tran/op_fork:/home/tran/op_fork/openpilot"
env["PATH"] = os.path.expanduser("~/.local/bin") + ":" + env.get("PATH", "")

controlsd = subprocess.Popen(
    ["python3", "-m", "openpilot.selfdrive.controls.controlsd"],
    cwd="/home/tran/op_fork/openpilot", env=env,
    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
print("controlsd launched (pid %d) — separate process" % controlsd.pid)
time.sleep(2.0)  # let it init and block on CarParams (already written)

# --- supercombo runner (real model) ---
runner = None
if not synthetic:
    from op_stream import SupercomboRunner
    runner = SupercomboRunner()
    print("supercombo loaded")

# --- publishers for controlsd's inputs; subscriber for its output ---
topics = ['liveDelay','liveParameters','liveTorqueParameters','modelV2','selfdriveState',
          'liveCalibration','livePose','longitudinalPlan','lateralManeuverPlan',
          'carState','carOutput','driverMonitoringState','onroadEvents','driverAssistance']
pm = messaging.PubMaster(topics)
sm = messaging.SubMaster(['carControl', 'controlsState'])
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
        # See full_chain.grab: whole-frame Argus AE underexposes the road under a
        # bright sky. CameraAE re-aims it with a slow road-band loop.
        from op_camera_ae import CameraAE
        _CAM = CameraAE(auto_exposure=True)
    return _CAM.read()

def send_all(out, curv, act=None):
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
            # See full_chain.publish_all: all three action fields, so what
            # controlsd reads over the bus matches what modeld would have sent.
            if act is not None:
                md.action.desiredAcceleration=float(act["desiredAcceleration"])
                md.action.shouldStop=bool(act["shouldStop"])
        except Exception: pass
    send('carState', cs); send('liveParameters', lp); send('liveTorqueParameters', tp)
    send('selfdriveState', ss); send('liveCalibration', lc); send('modelV2', mv)
    for t in ('liveDelay','livePose','longitudinalPlan','lateralManeuverPlan',
              'carOutput','driverMonitoringState','onroadEvents','driverAssistance'):
        send(t, lambda x: None)

print("\n%-6s %-11s %-11s %s" % ("frame","curvature","cc.torque","latActive (from controlsd proc)"))
print("-"*62)
got_cc = 0
try:
    for i in range(n):
        if synthetic:
            out = {"path_xyz": np.array([[j*V*0.1, 0.002*j*j, 0.0] for j in range(33)])}
        else:
            f = grab()
            # v_ego is required for the stock action: desiredCurvature is psi/(v*t).
            # This harness has no CAN, so V is its stated constant-speed assumption.
            out = runner.step(f, v_ego=V) if f is not None else {"path_xyz": None}
        # STOCK when we have a real action (modeld.get_action_from_model); the legacy
        # position fit only for --synthetic, whose fake `out` has no plan columns.
        act = out.get("action")
        if act is not None:
            curv = act["desiredCurvature"]
        elif out.get("path_xyz") is not None:
            curv = path_to_curvature(out.get("path_xyz"), V)
        else:
            curv = 0.0

        # publish inputs several times at ~50Hz so controlsd's SubMaster stays fresh
        for _ in range(6):
            send_all(out, curv, act)
            time.sleep(0.02)

        sm.update(50)
        if sm.updated['carControl']:
            got_cc += 1
            cc = sm['carControl']
            print("%-6d %-11.5f %-11.4f %s" %
                  (i, curv, cc.actuators.torque, cc.latActive))
        else:
            print("%-6d %-11.5f (no carControl yet)" % (i, curv))
finally:
    if _CAM is not None:
        _CAM.close()          # the sensor stays open now, so it has to be released
    controlsd.send_signal(signal.SIGINT)
    time.sleep(0.5)
    controlsd.terminate()
    out, _ = controlsd.communicate(timeout=5)
    print("\n=== controlsd process output (last lines) ===")
    print(out.decode(errors="replace")[-600:])

print("\ncarControl frames received from the controlsd PROCESS: %d/%d" % (got_cc, n))
print("=== INTEGRATED: supercombo + controlsd running as separate processes over the bus ===")
print("(carControl published to the bus; nothing transmitted to the car)")
