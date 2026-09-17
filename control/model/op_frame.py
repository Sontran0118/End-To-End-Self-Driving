#!/usr/bin/env python3
"""openpilot camera -> supercombo model input, for a SINGLE wide camera on the Jetson.

Produces BOTH model vision inputs from one IMX477 frame:
  img      [1,12,128,256] uint8 -- medmodel  frame (fl 910, cy 47.6)   ~ +/-15.7 deg H
  big_img  [1,12,128,256] uint8 -- sbigmodel frame (fl 455, cy 151.8)  ~ +/-29.4 deg H

comma uses two physical cameras (a 40 deg road cam and a 119 deg fisheye). With one
87 deg lens both windows can be synthesised correctly -- they are just two different
crops of the same ray bundle -- so this is geometrically exact, not an approximation.
What is lost vs stock is angular resolution on the road branch, nothing else.

Fixed here vs the previous version (each was wrong on every frame):
  1. Y-quadrant channel order. openpilot packs [even/even, ODD/even, EVEN/odd, odd/odd]
     (compile_modeld.py frames_to_tensor); this packed [ee, eo, oe, oo] -- channels 1
     and 2 transposed.
  2. YUV range. cv2.COLOR_BGR2YUV_I420 emits studio swing (Y 16..235). comma's ISP is
     CAM_COLOR_SPACE_BT601_FULL (spectra.cc). Every frame arrived at 86% contrast with
     a +16 pedestal. COLOR_BGR2YUV is the full-range conversion.
  3. big_img was a byte copy of img, i.e. the wide branch was fed the narrow warp.
  4. Temporal spacing. The two stacked frames are sampled frame_skip=4 apart, NOT
     consecutively: openpilot's img_q is frame_skip*(n_frames-1)+1 = 5 deep and is
     sampled [::4]. At MODEL_RUN_FREQ=20 that is 200 ms, and MODEL_CONTEXT_FREQ=5 is
     commented "model_trained_fps". This model has no v_ego input -- the frame pair is
     its ONLY speed cue -- so feeding 50 ms scaled every motion estimate by 4x.
  5. Focal length. Was derived from the marketing FOV applied to the 16:9 capture
     diagonal. See CAPTURE_FOCAL_PX below.
  6. Sampling. openpilot warps nearest-neighbour (warp_perspective_tinygrad rounds);
     this used INTER_LINEAR.
  7. Lens distortion is now correctable. openpilot's warp is a pure pinhole homography,
     fine for their rectilinear 40 deg road cam, wrong for an 87 deg M12. Undistortion
     is folded into the same remap table, so it is free at runtime.

Usage:
    fr = ModelFrameInput(cam_w=1920, cam_h=1080)
    fr.set_calibration(rpy)          # whenever op_calibrate updates
    img, big_img = fr.push(bgr)
"""
import collections
import numpy as np
import cv2

# --- openpilot model geometry (common/transformations/model.py) ---------------
MODEL_W, MODEL_H = 512, 256
MEDMODEL_INPUT_SIZE = (MODEL_W, MODEL_H)   # kept for callers that import it
MEDMODEL_CY = 47.6
MEDMODEL_FL = 910.0
SBIGMODEL_FL = 455.0

medmodel_intrinsics = np.array([
    [MEDMODEL_FL, 0.0, 0.5 * MODEL_W],
    [0.0, MEDMODEL_FL, MEDMODEL_CY],
    [0.0, 0.0, 1.0]])

sbigmodel_intrinsics = np.array([
    [SBIGMODEL_FL, 0.0, 0.5 * MODEL_W],
    [0.0, SBIGMODEL_FL, 0.5 * (256 + MEDMODEL_CY)],
    [0.0, 0.0, 1.0]])

# device <-> view frame (transformations/camera.py)
device_frame_from_view_frame = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
view_frame_from_device_frame = device_frame_from_view_frame.T


def rot_from_euler(rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def get_view_frame_from_calib_frame(roll, pitch, yaw, height):
    device_from_calib = rot_from_euler([roll, pitch, yaw])
    view_from_calib = view_frame_from_device_frame.dot(device_from_calib)
    return np.hstack((view_from_calib, [[0], [height], [0]]))


_E = get_view_frame_from_calib_frame(0, 0, 0, 0)
# model pixel -> ray in the calibrated frame
calib_from_medmodel = np.linalg.inv(np.dot(medmodel_intrinsics, _E)[:, :3])
calib_from_sbigmodel = np.linalg.inv(np.dot(sbigmodel_intrinsics, _E)[:, :3])

# --- camera intrinsics -------------------------------------------------------
# Arducam 12MP IMX477 motorized-focus HQ camera, M12 lens (B0271/B0272 family).
# Vendor FOV is 100(D)/87(H)/71(V). Those three are mutually consistent with a single
# pinhole focal of 2126 px across the full 4056 px sensor width (they reproduce
# 87.3/71.1/100.0), so the datasheet gives the focal directly -- to ~1%, no target
# needed. 2126 px * 1.55 um = 3.30 mm.
#
# NOTE the vendor numbers land exactly on the pinhole formula, which means they were
# computed from the nominal lens, not measured. They say nothing about distortion --
# set DIST below once you have measured it (plumb-line fit on any straight edge).
IMX477_FULL_WIDTH = 4056
IMX477_FULL_FOCAL_PX = 2126.0

# Focal per capture mode. The IMX477 has no native 1920x1080: its 1080p-class readouts
# are 2028x1128 (2x2 binned) and 2024x1142 (scaled), both FULL FOV, so the Jetson
# 1920x1080@60 mode is a ~5% crop of the binned full-FOV frame -> half the full focal.
#
# VERIFY THIS ONCE: capture the same scene at 3840x2160 and at 1920x1080. Identical
# framing => binned, 1063 px is right. Visibly zoomed in => it is a 1:1 centre crop,
# focal is 2126 px and the horizontal FOV is only 48.6 deg, which is 10 deg/side short
# of what big_img needs -- in that case you must capture 3840x2160.
CAPTURE_FOCAL_PX = {
    (4032, 3040): 2126.0,   # full sensor
    (3840, 2160): 2126.0,   # 16:9 crop, full width
    (1920, 1080): 1063.0,   # 2x2 binned full-FOV crop  <-- recommended capture mode
    (1280, 720):   709.0,   # 1080p scaled 2/3: UNDERSAMPLES the road branch (0.78x)
}

# Radial/tangential distortion (k1, k2, p1, p2, k3), OpenCV convention, in normalised
# camera coords. All-zero = pure pinhole = what openpilot assumes. An 87 deg M12 lens
# is NOT pinhole; measure k1/k2 and put them here.
DIST = np.zeros(5)


def focal_for_capture(cam_w, cam_h):
    """Focal in pixels for a capture size, from the mode table, else scaled by width."""
    f = CAPTURE_FOCAL_PX.get((cam_w, cam_h))
    if f is not None:
        return f
    return IMX477_FULL_FOCAL_PX * (cam_w / IMX477_FULL_WIDTH)


def camera_intrinsics(cam_w, cam_h, focal_px=None):
    """Principal point is assumed to be the image centre. An error there is a pure
    rotation, and op_calibrate learns and absorbs exactly that as pitch/yaw -- so it
    does not need measuring. Focal is a SCALE and nothing absorbs it."""
    f = focal_px if focal_px is not None else focal_for_capture(cam_w, cam_h)
    return np.array([[f, 0.0, cam_w / 2.0],
                     [0.0, f, cam_h / 2.0],
                     [0.0, 0.0, 1.0]])


def _build_remap(calib_from_model, calib_euler, K, dist, out_w=MODEL_W, out_h=MODEL_H):
    """Map each model-frame pixel to a source-image pixel, including lens distortion.

    A pure homography (what openpilot does) is only valid for a rectilinear lens. Here
    the model pixel is turned into a ray, the ray is distorted, and only then
    projected -- so a real M12 barrel profile inverts correctly. Precomputed once per
    calibration update; at runtime this is a single cv2.remap.
    """
    u, v = np.meshgrid(np.arange(out_w, dtype=np.float64),
                       np.arange(out_h, dtype=np.float64))
    pts = np.stack([u.ravel(), v.ravel(), np.ones(u.size)], 0)      # (3,N)

    ray_calib = calib_from_model @ pts                               # calib frame
    ray_view = view_frame_from_device_frame @ rot_from_euler(calib_euler) @ ray_calib
    z = ray_view[2]
    with np.errstate(divide='ignore', invalid='ignore'):
        xn = ray_view[0] / z
        yn = ray_view[1] / z

    k1, k2, p1, p2, k3 = dist
    r2 = xn * xn + yn * yn
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    xd = xn * radial + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn)
    yd = yn * radial + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn

    px = K[0, 0] * xd + K[0, 2]
    py = K[1, 1] * yd + K[1, 2]
    # rays behind the camera must not wrap around onto a valid pixel
    bad = ~(z > 0) | ~np.isfinite(px) | ~np.isfinite(py)
    px = np.where(bad, -1.0, px)
    py = np.where(bad, -1.0, py)
    return px.reshape(out_h, out_w).astype(np.float32), \
           py.reshape(out_h, out_w).astype(np.float32)


def pack_six(bgr_model_frame):
    """512x256 BGR in the model frame -> the model's 6 channels for one frame.

    Channel order is openpilot's frames_to_tensor: 4 Y quadrants as
    [even/even, ODD/even, EVEN/odd, odd/odd], then U, then V.
    YUV is BT.601 FULL range, matching comma's ISP (CAM_COLOR_SPACE_BT601_FULL).
    """
    yuv = cv2.cvtColor(bgr_model_frame, cv2.COLOR_BGR2YUV)   # full range, 3ch
    y = yuv[:, :, 0]
    # chroma to half resolution by 2x2 average (the ISP resamples; nearest would alias)
    u = yuv[:, :, 1].astype(np.uint16)
    v = yuv[:, :, 2].astype(np.uint16)
    u = ((u[0::2, 0::2] + u[0::2, 1::2] + u[1::2, 0::2] + u[1::2, 1::2] + 2) // 4).astype(np.uint8)
    v = ((v[0::2, 0::2] + v[0::2, 1::2] + v[1::2, 0::2] + v[1::2, 1::2] + 2) // 4).astype(np.uint8)
    return np.stack([y[0::2, 0::2],    # even row, even col
                     y[1::2, 0::2],    # ODD  row, even col
                     y[0::2, 1::2],    # even row, ODD  col
                     y[1::2, 1::2],    # odd  row, odd  col
                     u, v], 0).astype(np.uint8)


class ModelFrameInput:
    """One camera -> both supercombo vision inputs, with openpilot's temporal spacing.

    Holds a frame_skip*(n_frames-1)+1 = 5 deep ring of packed frames and stacks
    ring[0] with ring[-1], i.e. 200 ms apart when pushed at 20 Hz -- matching
    run_policy's shift_and_sample(img_q, ..., sample_skip).
    """
    N_FRAMES = 2
    FRAME_SKIP = 4          # MODEL_RUN_FREQ // MODEL_CONTEXT_FREQ = 20 // 5

    def __init__(self, cam_w, cam_h, focal_px=None, dist=None, calib_euler=(0.0, 0.0, 0.0)):
        self.cam_w, self.cam_h = int(cam_w), int(cam_h)
        self.focal_px = focal_px
        self.K = camera_intrinsics(self.cam_w, self.cam_h, focal_px)
        self.dist = np.asarray(DIST if dist is None else dist, dtype=np.float64)
        self.ring = collections.deque(maxlen=self.FRAME_SKIP * (self.N_FRAMES - 1) + 1)
        self.calib_euler = None
        self.set_calibration(calib_euler)

    def set_calibration(self, calib_euler, force=False):
        """Rebuild both remap tables. Cheap enough to call whenever rpy moves."""
        calib_euler = tuple(float(x) for x in calib_euler)
        if not force and self.calib_euler is not None and \
                np.allclose(calib_euler, self.calib_euler, atol=1e-6):
            return
        self.calib_euler = calib_euler
        self.map_med = _build_remap(calib_from_medmodel, calib_euler, self.K, self.dist)
        self.map_big = _build_remap(calib_from_sbigmodel, calib_euler, self.K, self.dist)

    def _warp(self, bgr, maps):
        # INTER_NEAREST to match openpilot's warp_perspective_tinygrad, which rounds to
        # the nearest source pixel rather than interpolating.
        return cv2.remap(bgr, maps[0], maps[1], interpolation=cv2.INTER_NEAREST,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

    def push(self, bgr):
        """Push one camera frame. Returns (img, big_img), each (1,12,128,256) uint8."""
        h, w = bgr.shape[:2]
        if (w, h) != (self.cam_w, self.cam_h):
            self.cam_w, self.cam_h = w, h
            self.K = camera_intrinsics(w, h, self.focal_px)
            self.set_calibration(self.calib_euler, force=True)
            self.ring.clear()
        six_med = pack_six(self._warp(bgr, self.map_med))
        six_big = pack_six(self._warp(bgr, self.map_big))
        if not self.ring:                        # prime: repeat, so the first outputs
            for _ in range(self.ring.maxlen):    # are a zero-motion pair, not garbage
                self.ring.append((six_med, six_big))
        else:
            self.ring.append((six_med, six_big))
        old, new = self.ring[0], self.ring[-1]
        img = np.concatenate([old[0], new[0]], 0)[None]
        big = np.concatenate([old[1], new[1]], 0)[None]
        return img, big


# --- compatibility shim for the single-shot harnesses (op_run/op_pipeline) ----
def frame_to_model_input(bgr, calib_euler=(0.0, 0.0, 0.0), prev_yuv=None, focal_px=None):
    """Single-frame path. Correct packing/range/warp, but NO temporal spacing -- it
    stacks the immediately previous frame. Use ModelFrameInput for streaming."""
    h, w = bgr.shape[:2]
    K = camera_intrinsics(w, h, focal_px)
    maps = _build_remap(calib_from_medmodel, calib_euler, K, DIST)
    warped = cv2.remap(bgr, maps[0], maps[1], interpolation=cv2.INTER_NEAREST,
                       borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
    six = pack_six(warped)
    prev = prev_yuv if prev_yuv is not None else six
    return np.concatenate([prev, six], 0)[None], six


if __name__ == "__main__":
    import math
    d = np.degrees

    print("=== model frames the camera must cover (rel. calibrated forward axis) ===")
    for nm, fl, cy in (("img     (medmodel )", MEDMODEL_FL, MEDMODEL_CY),
                       ("big_img (sbigmodel)", SBIGMODEL_FL, 0.5 * (256 + MEDMODEL_CY))):
        print(f"  {nm}: +/-{d(math.atan(MODEL_W/2/fl)):.1f} deg H, "
              f"{d(math.atan(cy/fl)):.1f} up / {d(math.atan((MODEL_H-cy)/fl)):.1f} down, "
              f"needs source focal >= {fl:.0f} px")

    print("\n=== IMX477 + 100(D)/87(H)/71(V) M12 lens, per capture mode ===")
    need_big_half = d(math.atan(MODEL_W / 2 / SBIGMODEL_FL))
    for (w, h), f in sorted(CAPTURE_FOCAL_PX.items()):
        hf = d(math.atan(w / 2 / f)) * 2
        print(f"  {w}x{h}: f={f:7.1f}px  HFOV={hf:5.1f}  "
              f"med {f/MEDMODEL_FL:4.2f}x {'OK' if f >= MEDMODEL_FL else 'UNDERSAMPLED':12s}  "
              f"big {f/SBIGMODEL_FL:4.2f}x {'OK' if hf/2 >= need_big_half else 'FOV SHORT'}")

    print("\n=== channel order vs openpilot frames_to_tensor ===")
    # reproduce compile_modeld.py:104-109 exactly on a synthetic YUV420 buffer
    H, W = MODEL_H, MODEL_W
    rng = np.random.default_rng(0)
    frames = rng.integers(0, 256, (H * 3 // 2, W), dtype=np.uint8)
    ref = np.concatenate([frames[0:H:2, 0::2], frames[1:H:2, 0::2],
                          frames[0:H:2, 1::2], frames[1:H:2, 1::2],
                          frames[H:H + H // 4].reshape(H // 2, W // 2),
                          frames[H + H // 4:H + H // 2].reshape(H // 2, W // 2)],
                         0).reshape(6, H // 2, W // 2)
    y = frames[:H]
    ours = np.stack([y[0::2, 0::2], y[1::2, 0::2], y[0::2, 1::2], y[1::2, 1::2]], 0)
    old = np.stack([y[0::2, 0::2], y[0::2, 1::2], y[1::2, 0::2], y[1::2, 1::2]], 0)
    print(f"  new packing matches openpilot: {np.array_equal(ours, ref[:4])}")
    print(f"  old packing matched openpilot: {np.array_equal(old, ref[:4])}")

    print("\n=== YUV range (comma's ISP is BT601 FULL) ===")
    white = np.full((8, 8, 3), 255, np.uint8)
    black = np.zeros((8, 8, 3), np.uint8)
    print(f"  new  COLOR_BGR2YUV:      white Y={cv2.cvtColor(white, cv2.COLOR_BGR2YUV)[...,0].max()}"
          f"  black Y={cv2.cvtColor(black, cv2.COLOR_BGR2YUV)[...,0].min()}   (want 255 / 0)")
    print(f"  old  COLOR_BGR2YUV_I420: white Y={cv2.cvtColor(white, cv2.COLOR_BGR2YUV_I420)[:8,:].max()}"
          f"  black Y={cv2.cvtColor(black, cv2.COLOR_BGR2YUV_I420)[:8,:].min()}   (studio swing)")

    print("\n=== temporal spacing ===")
    fr = ModelFrameInput(1920, 1080)
    print(f"  ring depth={fr.ring.maxlen} = frame_skip*(n_frames-1)+1, stacks ring[0] with ring[-1]")
    print(f"  at 20 Hz that is {fr.FRAME_SKIP * 50} ms apart (MODEL_CONTEXT_FREQ=5 'model_trained_fps')")

    demo = (rng.random((1080, 1920, 3)) * 255).astype(np.uint8)
    img, big = fr.push(demo)
    print(f"\n  push() -> img {img.shape} {img.dtype}, big_img {big.shape} {big.dtype}")
    print(f"  img and big_img differ (separate warps): {not np.array_equal(img, big)}")
