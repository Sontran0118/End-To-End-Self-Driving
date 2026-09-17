#!/usr/bin/env python3
"""SUPERCOMBO -> modelV2 PUBLISHER shim.

Runs the real supercombo model on live IMX477 frames and publishes its output
onto the cereal bus as `modelV2`, so controlsd consumes the REAL model instead
of synthetic data. This is the missing link between the model port and the
control stack.

Mapping (SupercomboRunner.step() -> modelV2):
  path_xyz (33,3)      -> position.x/y/z + t
  lane_lines (4,33,2)  -> laneLines[i].x/y/z
  road_edges (2,33,2)  -> roadEdges[i].x/y/z
  action               -> action.desiredCurvature (what controlsd steers on)
                          + desiredAcceleration + shouldStop

Read-only + publish. No transmit to car.
Usage: python3 supercombo_publisher.py [--synthetic] [-n N]
"""
import sys, os, time, math, subprocess

os.environ.setdefault("PARAMS_ROOT", "/tmp/op_params")
sys.path.insert(0, "/home/tran/openpilot_jetson")

import numpy as np
import openpilot.cereal.messaging as messaging
sys.path.insert(0,"/home/tran/openpilot_jetson")
from curvature_lib import T_IDXS as _T_IDXS, path_to_curvature

# openpilot model time indices — 33 points, matching the model's own plan axis.
# This file used to carry its own hardcoded table ending at 10.185 s: a ~1.85%
# stretched copy of the real one, which labelled every published path point with a
# time the model did not predict it for. Taken from curvature_lib now, where it is
# built from ModelConstants' own index_function and asserted against it.
T_IDXS = [float(t) for t in _T_IDXS]


def path_curvature(path_xyz, v_ego=20.0):
    return path_to_curvature(path_xyz, v_ego)


def fill_modelV2(md, out):
    path = out.get("path_xyz")
    lanes = out.get("lane_lines")
    edges = out.get("road_edges")
    N = 33

    md.frameId = fill_modelV2.frame
    fill_modelV2.frame += 1

    if path is not None:
        px = [float(path[i][0]) for i in range(N)]
        py = [float(path[i][1]) for i in range(N)]
        pz = [float(path[i][2]) for i in range(N)]
        md.position.x = px
        md.position.y = py
        md.position.z = pz
        md.position.t = T_IDXS

    # lane lines (4)
    if lanes is not None:
        ll = md.init('laneLines', 4)
        for k in range(4):
            ll[k].x = [float(lanes[k][i][0]) for i in range(N)]
            ll[k].y = [float(lanes[k][i][1]) for i in range(N)]
            ll[k].z = [0.0] * N
            ll[k].t = T_IDXS
        lp = out.get("lane_prob")
        if lp is not None:
            md.laneLineProbs = [float(x) for x in lp[:4]] if len(lp) >= 4 else [0.5]*4

    # road edges (2)
    if edges is not None:
        re = md.init('roadEdges', 2)
        for k in range(2):
            re[k].x = [float(edges[k][i][0]) for i in range(N)]
            re[k].y = [float(edges[k][i][1]) for i in range(N)]
            re[k].z = [0.0] * N
            re[k].t = T_IDXS

    # The action fields controlsd consumes. STOCK derivation (modeld.get_action_from_model)
    # whenever the runner gave us one -- from the plan's yaw/yaw-rate at the actuator
    # delay, not a quadratic fit to `path`. The legacy fit is the --synthetic fallback,
    # where `out` is a hand-built path with no plan columns.
    #
    # desiredAcceleration and shouldStop were never filled here at all; stock modeld
    # publishes all three on modelV2.action, so longitudinal saw nothing but the
    # capnp default.
    try:
        act = out.get("action")
        if act is not None:
            md.action.desiredCurvature = float(act["desiredCurvature"])
            md.action.desiredAcceleration = float(act["desiredAcceleration"])
            md.action.shouldStop = bool(act["shouldStop"])
        else:
            md.action.desiredCurvature = float(path_curvature(path, out.get("v_ego", 20.0)))
    except Exception:
        pass


fill_modelV2.frame = 0


_CAM = None


def grab_frame():
    """One persistent 1080p NV12 capture, opened on first use.

    Replaces a per-frame `gst-launch nvarguscamerasrc num-buffers=1 ! ... ! jpegenc
    ! filesink` + imread. That was wrong three ways at once, all of them landing on
    the model's input:

      * ~15s per frame of process spawn + Argus daemon init + sensor start +
        teardown (which is why the old subprocess.run carried a 15s timeout). The
        model step is ~29ms.
      * 1280x720, so op_frame looked up CAPTURE_FOCAL_PX[(1280,720)] = 709 px
        against medmodel's 910 -- the road branch fed a 0.78x UPSAMPLED image, the
        exact defect the capture path was rewritten to remove.
      * a lossy JPEG round-trip (jpegenc -> file -> imread), i.e. 4:2:0 subsampling
        and DCT ringing applied to the frame before the model ever sees it, on top
        of the chroma subsampling pack_six already does.

    op_camera.Camera keeps the sensor open, streams NV12 at full 1920x1080, and runs
    Argus's AE across the whole exposure envelope."""
    global _CAM
    if _CAM is None:
        # See full_chain.grab: whole-frame Argus AE underexposes the road under a
        # bright sky. CameraAE re-aims it with a slow road-band loop.
        from op_camera_ae import CameraAE
        _CAM = CameraAE(auto_exposure=True)
    return _CAM.read()


def close_camera():
    global _CAM
    if _CAM is not None:
        _CAM.close()
        _CAM = None


def main():
    synthetic = "--synthetic" in sys.argv
    n = 10
    if "-n" in sys.argv:
        n = int(sys.argv[sys.argv.index("-n") + 1])

    pm = messaging.PubMaster(['modelV2'])
    sm = messaging.SubMaster(['modelV2'])

    runner = None
    if not synthetic:
        from op_stream import SupercomboRunner
        runner = SupercomboRunner()
        print("supercombo loaded — publishing REAL model output as modelV2")
    else:
        print("synthetic mode — publishing a curved fake path")

    time.sleep(0.5)
    for i in range(n):
        if synthetic:
            V = 20.0
            fake_path = np.array([[j * V * 0.1, 0.002 * j * j, 0.0] for j in range(33)])
            out = {"path_xyz": fake_path, "lane_lines": None, "road_edges": None}
        else:
            frame = grab_frame()
            if frame is None:
                print("  (no frame)"); continue
            # v_ego is required for the stock action (desiredCurvature = psi/(v*t)).
            # This shim has no CAN, so 20.0 is its stated constant-speed assumption --
            # previously the same 20.0 was buried as a default argument in
            # path_curvature() and applied silently.
            out = runner.step(frame, v_ego=20.0)

        msg = messaging.new_message('modelV2')
        fill_modelV2(msg.modelV2, out)
        pm.send('modelV2', msg)

        # confirm it's on the bus + show the curvature controlsd will use
        time.sleep(0.05)
        sm.update(50)
        curv = msg.modelV2.action.desiredCurvature
        print("frame %2d: published modelV2, desiredCurvature=%.5f  (received=%s)" %
              (i, curv, sm.updated['modelV2']))

    print("\n=== supercombo -> modelV2 on the bus. controlsd can now consume it. ===")


if __name__ == "__main__":
    try:
        main()
    finally:
        close_camera()   # the sensor stays open now, so it has to be released
