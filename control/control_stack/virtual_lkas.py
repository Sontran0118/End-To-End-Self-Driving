#!/usr/bin/env python3
"""VIRTUAL LKAS pipeline — supercombo -> steering -> 0x243, PRINTED ONLY.

Runs the full model->control->CAN-frame chain on LIVE IMX477 frames and PRINTS
the 0x243 CAM_LKAS frames the model would produce. It NEVER transmits to the
car. There is deliberately no can_send in this file.

Chain:
  IMX477 frame -> supercombo (path_xyz, lane_lines, road_edges, lane_prob)
    -> lane-center + path lateral error at a lookahead
    -> desired steering torque (PD)
    -> pack into 0x243 with reverse-engineered rolling counter + checksum
    -> PRINT (compare vs the real camera frames we captured)

Usage:
  python3 virtual_lkas.py            # synthetic drift (no camera)
  python3 virtual_lkas.py --cam      # live camera + real model
  python3 virtual_lkas.py --cam -n 40
"""
import sys, time, math, subprocess

# --- 0x243 packer: counter + checksum from the reverse-engineered format ------
def mazda_checksum(b):
    # Verified against capture: 08 00 09 60 02 00 00 -> checksum 0xB6.
    #   sum(b0..b6) = 0x73;  (K - 0x73) & 0xFF = 0xB6  =>  K = 0x29
    return (0x29 - (sum(b[:7]) & 0xFF)) & 0xFF

def build_0x243(counter, steer_torque):
    """Idle template 08 00 09 60 02 00 00 <ck>, torque overlaid for display.
    Counter + checksum are byte-accurate to the real camera; the torque field
    position is a DEMO placeholder (exact packing = opendbc create_steering_control)."""
    b = bytearray([((counter & 0xF) << 4) | 0x8, 0x00, 0x09, 0x60, 0x02, 0x00, 0x00, 0x00])
    t = max(-1024, min(1023, int(steer_torque))) & 0x7FF
    b[2] = (b[2] & 0xF0) | ((t >> 7) & 0x0F)   # visible torque nibble (demo)
    b[7] = mazda_checksum(b)
    return bytes(b)

# --- lateral control: model output -> steering torque -------------------------
def compute_torque(out):
    """Use lane center + path to get a lateral error at ~10m lookahead, then PD.
    out: dict from SupercomboRunner.step()."""
    path = out.get("path_xyz")
    lanes = out.get("lane_lines")      # (4,33,2) : [ll_far, ll_near, rl_near, rl_far], (x,y)?
    laneprob = out.get("lane_prob")
    LA = 10  # lookahead index (~10 points ahead)

    lateral_err = 0.0
    used = "path"
    # Prefer lane centering when both near lane lines are confident
    if lanes is not None and laneprob is not None and len(lanes) >= 4:
        # inner lane lines are indices 1 (left-near) and 2 (right-near)
        try:
            left_y = lanes[1][LA][1]
            right_y = lanes[2][LA][1]
            pL = laneprob[1] if len(laneprob) > 1 else 0.0
            pR = laneprob[2] if len(laneprob) > 2 else 0.0
            if pL > 0.3 and pR > 0.3:
                center = (left_y + right_y) / 2.0
                lateral_err = center      # want center at y=0 (ego centered)
                used = "lanes(p%.2f/%.2f)" % (pL, pR)
        except Exception:
            pass
    if used == "path" and path is not None and len(path) > LA:
        lateral_err = path[LA][1]

    # path heading over the near segment (for the derivative term)
    heading = 0.0
    if path is not None and len(path) > LA:
        dx = float(path[LA][0] - path[0][0])
        dy = float(path[LA][1] - path[0][1])
        heading = math.atan2(dy, dx) if abs(dx) > 1e-6 else 0.0

    k1, k2 = 150.0, 300.0
    torque = -(k1 * float(lateral_err) + k2 * heading)
    return torque, float(lateral_err), used


# No CAN in this harness, so there is no real speed. 20 m/s is the same stated
# assumption integrate.py and full_chain.py make. It matters because op_stream.step()
# only feeds the online calibrator when v_ego is not None -- call it without and rpy
# stays (0,0,0) forever, so the input warp never gets corrected.
V_EGO_ASSUMED = 20.0

_CAM = None


def grab_frame():
    """Persistent 1080p NV12 capture, opened on first use.

    Replaces a per-frame `gst-launch nvarguscamerasrc num-buffers=1 ! ... ! jpegenc
    ! filesink` + imread, which cost ~15s a frame (process spawn + Argus init +
    sensor start + teardown, hence the old 15s timeout), captured at 1280x720 so
    op_frame looked up CAPTURE_FOCAL_PX[(1280,720)] = 709 px against medmodel's 910
    -- the road branch fed a 0.78x UPSAMPLED image -- and put a lossy JPEG
    round-trip in front of the model on top of that."""
    global _CAM
    if _CAM is None:
        # See full_chain.grab: whole-frame Argus AE underexposes the road under a
        # bright sky. CameraAE re-aims it with a slow road-band loop.
        from op_camera_ae import CameraAE
        _CAM = CameraAE(auto_exposure=True)
    return _CAM.read()


def main():
    use_cam = "--cam" in sys.argv
    n = 20
    if "-n" in sys.argv:
        n = int(sys.argv[sys.argv.index("-n") + 1])

    runner = None
    if use_cam:
        sys.path.insert(0, "/home/tran/openpilot_jetson")
        from op_stream import SupercomboRunner
        runner = SupercomboRunner()
        print("supercombo loaded — LIVE IMX477 -> model -> 0x243 (print only)")

    print("\n=== VIRTUAL LKAS  (no transmit) ===")
    print("real idle frame for reference: 08000960020000b6\n")
    print("%-4s %-9s %-9s %-16s %s" % ("cnt", "torque", "lat_err", "0x243", "source"))
    print("-" * 62)

    counter = 0
    for i in range(n):
        if use_cam:
            frame = grab_frame()
            if frame is None:
                print("  (no frame)"); time.sleep(0.2); continue
            out = runner.step(frame, v_ego=V_EGO_ASSUMED)
            torque, lat, src = compute_torque(out)
        else:
            lat = 0.6 * math.sin(i * 0.5)
            fake = [[j * 1.0, lat * (j / 10.0), 0.0] for j in range(33)]
            out = {"path_xyz": fake}
            torque, lat, src = compute_torque(out)
            src = "synthetic"

        fr = build_0x243(counter, torque)
        print("%-4X %-9.1f %-9.2f %-16s %s" % (counter, torque, lat, fr.hex(), src))
        counter = (counter + 1) & 0xF

    print("\n(no frames transmitted — virtual/print-only)")


if __name__ == "__main__":
    try:
        main()
    finally:
        if _CAM is not None:
            _CAM.close()   # the sensor stays open now, so it has to be released
