import sys, cv2, numpy as np
sys.path.insert(0, "/home/tran/openpilot_jetson")
from op_stream import SupercomboRunner

r = SupercomboRunner()

# /tmp/vf.jpg was written by the per-frame `jpegenc ! filesink` grab that every
# harness here used to do; nothing writes it now, so fall back to a live capture.
f = cv2.imread("/tmp/vf.jpg")
if f is None:
    from op_camera_ae import CameraAE
    _cam = CameraAE(auto_exposure=True)
    f = _cam.read()
    _cam.close()

# v_ego is required for out["action"] to be populated at all -- desiredCurvature is
# psi/(v*t), so step() returns None for it rather than inventing a speed. This script
# exists to ask whether the runner exposes an action field; without a speed the answer
# looks like "no".
out = r.step(f, v_ego=20.0)

print("=== step() output keys ===")
for k, v in out.items():
    shape = getattr(v, "shape", None)
    print("  %-16s %s" % (k, shape if shape is not None else type(v).__name__))

# Does the runner expose the RAW output vector? Check for action / desired_curvature.
print("\n=== looking for curvature/action in output ===")
for key in out:
    if "curv" in key.lower() or "action" in key.lower() or "desire" in key.lower():
        print("  FOUND:", key, "=", out[key])

# The supercombo output_slices metadata names every field. Print any the runner kept.
for attr in ("output_slices", "raw", "raw_output", "outputs", "action", "desired_curvature"):
    if hasattr(r, attr):
        v = getattr(r, attr)
        print("  runner.%s: %s" % (attr, type(v).__name__))

# Path-derived vs any direct field: show the path near the car for comparison
if "path_xyz" in out:
    p = out["path_xyz"]
    print("\npath_xyz first 3:", p[:3].tolist())
    # openpilot-style curvature from the path (orientation-based would need orientation output)
    i = 10
    x, y = float(p[i][0]), float(p[i][1])
    print("path lookahead[%d] x=%.2f y=%.3f -> approx curv 2y/x^2 = %.5f" %
          (i, x, y, (2*y/(x*x)) if abs(x) > 1 else 0.0))

# Is there an orientation output we should use for a proper curvature?
if "orientation" in out:
    print("\norientation present:", out["orientation"].shape)
