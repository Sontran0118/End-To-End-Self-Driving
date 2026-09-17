#!/usr/bin/env python3
"""Complete streaming openpilot supercombo runner on the Jetson — WITH recurrent feedback.
Mirrors openpilot modeld: rolling feature queue (frame_skip=4 x 24 = 96 deep), the model's
hidden_state fed back each frame, subsampled to the 24-frame features_buffer. Processes a stream
of frames (from disk or camera) and decodes the driving plan each step.

  class SupercomboRunner: .step(bgr_frame) -> dict(path_xyz, lane_lines, lead_prob, pose, ...)
"""
import numpy as np, tensorrt as trt, torch, sys, os, base64, pickle, collections
sys.path.insert(0, "/home/tran/openpilot_jetson"); sys.path.insert(0, "/home/tran/openpilot_jetson/modeld")
from op_frame import ModelFrameInput
from op_calibrate import Calibrator
from curvature_lib import ACTION_WIDTH, DirectAction, ModelAction
from constants import ModelConstants as MC, Plan

# --- which driving model -----------------------------------------------------
# Default is "Rebellious Hope" (commaai/openpilot #38475, 2026-07-27), the current
# weights for any device without comma's USB GPU accessory. It differs from the
# model this port started on in two ways that matter here:
#
#   1. It emits an `action` head -- the steering and acceleration command itself,
#      as a trained output. Before, modeld reconstructed both from the plan's yaw
#      and speed columns. See curvature_lib.DirectAction.
#   2. It reordered every output field (lane_lines 117 -> 0, hidden_state
#      1064 -> 2066) and grew the vector 2576 -> 2580. Nothing here holds a
#      literal offset -- the layout is read from the ONNX's output_slices
#      metadata -- but an engine paired with the WRONG ONNX would decode lane
#      lines as the plan and never raise, so __init__ checks the two agree.
#
# Both are fp16 engines. fp32 is kept only as a numerical reference; on this board
# the fp16 build runs ~4.7x faster (4.5 ms vs 21.1 ms) for the same weights, which
# is the difference between having and not having headroom at 20 Hz.
_M = "/home/tran/openpilot_jetson/models/"
MODELS = {
    "rh":       (_M + "supercombo_rh_fp16.trt",  _M + "driving_supercombo_rh.onnx"),
    "rh_fp32":  (_M + "supercombo_rh_fp32.trt",  _M + "driving_supercombo_rh.onnx"),
    "old":      (_M + "supercombo_old_fp16.trt", _M + "driving_supercombo.onnx"),
    "old_fp32": (_M + "supercombo_fp32.trt",     _M + "driving_supercombo.onnx"),
}
MODEL = os.environ.get("OP_MODEL", "rh")
if MODEL not in MODELS:
    raise SystemExit(f"op_stream: OP_MODEL={MODEL!r} unknown; pick one of {sorted(MODELS)}")
# OP_ENGINE/OP_ONNX override outright, for a build that is not in the table yet.
ENGINE = os.environ.get("OP_ENGINE") or MODELS[MODEL][0]
ONNX   = os.environ.get("OP_ONNX")   or MODELS[MODEL][1]
FRAME_SKIP = MC.MODEL_RUN_FREQ // MC.MODEL_CONTEXT_FREQ    # 20//5 = 4
FB_LEN = 24                                                # features_buffer temporal length

# action_t = [lat_action_t, long_action_t], SECONDS. modeld.py builds these as
#   lat  = liveDelay.lateralDelay + LAT_SMOOTH_SECONDS(0.0) + DT_MDL + DT_MDL/2
#   long = CP.longitudinalActuatorDelay + LONG_SMOOTH_SECONDS(0.3) + DT_MDL + DT_MDL/2
# DT_MDL 0.05. This input was previously fed as zeros, i.e. "zero actuator latency",
# which is not a value the model was ever trained with.
#
# LATERAL: 0.10 -> 0.20. The 0.10 was lagd's initial_lag for MAZDA_CX5_2022 -- a
# fleet default, not this rack. The real number was MEASURED on this CX-5
# (2026-07-30, 10.3k records above 20 kph) by cross-correlating LKAS_REQUEST
# against LKAS_EFFECTIVE inside 0x241, which carries both in the SAME frame and
# so isolates the rack's own lag from anything our CAN pipeline contributes:
# peak r=0.887 at lag 4 records = 200 ms. See opendbc_src interface.py, which
# already carries 0.2, and latcontrol_torque.py's JERK_GAIN note.
#
# THIS FILE WAS THE HALF THAT WAS LEFT BEHIND. The controller side was already
# correct: dashcam_web publishes liveDelay.lateralDelay = CP.steerActuatorDelay
# (0.2), so LatControlTorque picks the right past setpoint out of
# lat_accel_request_buffer. But op_stream replaces modeld, so the model's OWN
# action_t input came from here -- and at 0.175 the model was planning its
# command for 100 ms sooner than the rack can deliver it. The controller then
# compensated for a lag the model had not planned around, which is the phase
# split that produced the 1.8 Hz limit cycle (torque crossing zero 1.84 times/s
# against a command crossing 0.87 times/s).
# OP_STEER_DELAY is the ONE place the lateral delay is set. It used to be split:
# this file hardcoded 0.20 here (-> LAT_ACTION_T 0.275) while controlsd took
# CP.steerActuatorDelay (0.20) + LAT_SMOOTH_SECONDS (0.0) = 0.200. Two different
# compensations for the same physical lag, and neither matched the loop: the
# measured limit cycle runs at 1.37 Hz, a 0.365 s half-period, so the real
# lag through camera -> model -> rack -> yaw -> camera is ~0.365 s.
# Under-compensated phase at this gain is what sustains the oscillation.
#
# interface.py:60 already says 0.2 is "the rack's lag alone ... a floor rather
# than a fitted optimum" -- the wheel-to-lateral-accel response is the rest.
# dashcam_web reads the same env var for liveDelay.lateralDelay, so the two
# sides can no longer disagree. Raise toward 0.30 to close the phase gap.
STEER_DELAY   = float(os.environ.get("OP_STEER_DELAY", 0.20))
LAT_ACTION_T  = STEER_DELAY + 0.0 + 0.05 + 0.025
# LONGITUDINAL: unchanged and still the fleet default -- there is no longitudinal
# actuation on this port to measure it against. NOTE for alpha long: the Mazda
# longitudinal port sets longitudinalActuatorDelay = 0.36, so this becomes
# 0.36 + 0.3 + 0.05 + 0.025 = 0.735 the moment openpilotLongitudinalControl is on.
LONG_ACTION_T = 0.15 + 0.3 + 0.05 + 0.025      # 0.525

# --- manual mount trim, degrees, applied ON TOP of the learned calibration ----
# OP_CALIB_TRIM_DEG="roll,pitch,yaw". Defaults to no trim.
#
# Why this exists: op_calibrate learns roll/pitch/yaw from the model's own pose
# odometry, and that is the right correction for a rotated mount -- but it CANNOT
# see two things that both put the car off centre:
#
#   1. the camera being mounted off the vehicle's centreline. The model centres
#      the CAMERA in the lane, so a camera d metres left of centre parks the car
#      d metres right of centre, and pose odometry looks perfectly clean the
#      whole time (a lateral offset is not a rotation).
#   2. anything downstream that biases the car sideways at equilibrium -- road
#      crown, alignment pull -- since liveParameters.roll is published as 0 here,
#      so latcontrol_torque's roll compensation is inert.
#
# A yaw trim is the lever for both. It rotates the model's window, so the model
# perceives a lateral error it does not have and settles the car off its own
# notion of centre by roughly L_eff * yaw, with L_eff ~ 15-30 m (the path
# follower's heading/position gain ratio). At the +1 deg = 18.6 px scale of this
# camera that is ~0.26-0.52 m per degree. The car ends up PARALLEL to the lane
# and offset, not driving at an angle, so this is a position trim, not a hack.
#
# Sign: +yaw shifts the sampled window RIGHT in the source image, which moves the
# car LEFT in the lane. Trim AFTER calibration converges, never instead of it.
_TRIM_ENV = os.environ.get("OP_CALIB_TRIM_DEG", "")
try:
    CALIB_TRIM = np.radians([float(x) for x in _TRIM_ENV.split(",")]) if _TRIM_ENV \
                 else np.zeros(3)
    assert CALIB_TRIM.shape == (3,)
except Exception:
    print(f"op_stream: ignoring malformed OP_CALIB_TRIM_DEG={_TRIM_ENV!r}, want 'roll,pitch,yaw'")
    CALIB_TRIM = np.zeros(3)
if CALIB_TRIM.any():
    print(f"op_stream: manual calibration trim {np.degrees(CALIB_TRIM).round(3)} deg")

def softmax(x,axis=-1): x=x-x.max(axis,keepdims=True); e=np.exp(x); return e/e.sum(axis,keepdims=True)
def sigmoid(x): return 1/(1+np.exp(-x))
def safe_exp(x): return np.exp(np.clip(x,-np.inf,11))

def load_slices(onnx_path=None):
    return load_model_meta(onnx_path)[0]

def load_model_meta(onnx_path=None):
    """(output_slices, declared output width) read off the ONNX.

    The width comes back too so the caller can check the TensorRT engine it just
    deserialised was built from THIS graph -- see the pairing check in __init__.
    """
    import onnx
    path=onnx_path or ONNX
    m=onnx.load(path, load_external_data=False)
    width=None
    for o in m.graph.output:
        if o.name in ("outputs","output"):
            width=int(o.type.tensor_type.shape.dim[-1].dim_value)
    for p in m.metadata_props:
        if p.key=="output_slices":
            return pickle.loads(base64.b64decode(p.value)), width
    raise RuntimeError(f"no output_slices metadata in {path}")

class SupercomboRunner:
    def __init__(self, engine=None, onnx=None):
        engine, onnx = engine or ENGINE, onnx or ONNX
        with open(engine,"rb") as f, trt.Runtime(trt.Logger(trt.Logger.ERROR)) as rt:
            self.eng=rt.deserialize_cuda_engine(f.read())
        self.ctx=self.eng.create_execution_context(); self.dev="cuda"; self.st=torch.cuda.Stream()
        self.slices,self._onnx_width=load_model_meta(onnx)
        self.engine_path, self.onnx_path = engine, onnx
        # A model with an `action` slice commands directly; one without it has the
        # command derived from its plan, as openpilot did before July 2026. Decided
        # by the file on disk, not by a flag anyone has to keep in sync.
        self.has_action = "action" in self.slices
        # recurrent feature queue: FRAME_SKIP*FB_LEN deep (96), each (512,), newest last
        self.feat_q=collections.deque([np.zeros(MC.FEATURE_LEN,np.float32)]*(FRAME_SKIP*FB_LEN),
                                      maxlen=FRAME_SKIP*FB_LEN)
        # camera -> both vision inputs. Built on the first frame, once its size is known;
        # holds the 5-deep ring that gives the stacked pair openpilot's 200 ms spacing.
        self.frames=None
        # online camera calibration (openpilot calibrationd port): learns roll/pitch/yaw
        # from the model's own pose odometry, then feeds it back into the input warp.
        self.calibrator=Calibrator(persist=True)
        # calib_euler is the EFFECTIVE warp angle (learned + trim); the dashboard's
        # calib_rpy reads it, so a trim shows up there. calibrator.rpy_unclipped,
        # which calib_rpy_raw_deg reports, stays the pure learned mount estimate.
        self.calib_euler=tuple(self.calibrator.rpy_smooth + CALIB_TRIM)
        # stock modeld.get_action_from_model, incl. the prev_action smoothing state.
        # Only runs when step() is given a real v_ego. Both branches need one: the
        # plan branch divides by it (psi/(v*t)), the action branch divides by its
        # SQUARE, so a guessed speed scales the steering command by that guess's
        # error either way -- worse on the head.
        self.action=DirectAction() if self.has_action else ModelAction()
        # bind IO tensors once
        self.io={}; self.out_name=None
        for i in range(self.eng.num_io_tensors):
            n=self.eng.get_tensor_name(i); shp=tuple(self.eng.get_tensor_shape(n))
            dt=torch.float16 if self.eng.get_tensor_dtype(n)==trt.float16 else (
               torch.uint8 if self.eng.get_tensor_dtype(n)==trt.uint8 else torch.float32)
            t=torch.zeros(shp,dtype=dt,device=self.dev); self.io[n]=t
            self.ctx.set_tensor_address(n,t.data_ptr())
            if self.eng.get_tensor_mode(n)==trt.TensorIOMode.OUTPUT: self.out_name=n
        # ENGINE/ONNX PAIRING. The two models' fields sit at completely different
        # offsets (lane_lines 117 -> 0, hidden_state 1064 -> 2066), so a mismatched
        # pair decodes every head into the wrong one and nothing raises -- the car
        # would steer on whatever happens to sit where the plan used to be. The
        # engine carries no layout of its own, so the only thing that can catch it
        # is the output width, and that only separates them because the widths
        # differ (2576 vs 2580). Compare against the ONNX's DECLARED width rather
        # than the last slice's stop: both models pad two floats past their last
        # named field, so "big enough to hold the slices" passes in one direction
        # even when the pair is wrong.
        n_out=int(np.prod(self.eng.get_tensor_shape(self.out_name)))
        if self._onnx_width is not None and n_out!=self._onnx_width:
            raise RuntimeError(
                f"engine/ONNX mismatch: {engine} outputs {n_out} floats, "
                f"{onnx} declares {self._onnx_width}. These are different models -- "
                f"decoding one with the other's layout gives no error and no valid output.")
        print(f"op_stream: {os.path.basename(engine)} + {os.path.basename(onnx)} "
              f"({n_out} outputs, action head: {'yes' if self.has_action else 'no'})")

    def _features_buffer(self):
        # subsample the deep queue every FRAME_SKIP -> (24,512) -> (1,24,512)
        arr=np.stack(list(self.feat_q),0)[::FRAME_SKIP]         # (24,512)
        return arr[None].astype(np.float16)

    @torch.no_grad()
    def step(self, bgr, desire=None, traffic=(1,0), v_ego=None,
             lat_action_t=LAT_ACTION_T, long_action_t=LONG_ACTION_T):
        # warp the frame using the CURRENT learned calibration (closes the loop:
        # a correctly-warped frame -> the model reports near-zero residual motion).
        if self.frames is None:
            self.frames=ModelFrameInput(bgr.shape[1], bgr.shape[0], calib_euler=self.calib_euler)
        else:
            self.frames.set_calibration(self.calib_euler)
        # img and big_img are the SAME camera warped into the two different model
        # frames (medmodel fl 910 / sbigmodel fl 455) -- not a byte copy of each other.
        img,big_img=self.frames.push(bgr)
        feed={"img":img,"big_img":big_img,
              "features_buffer":self._features_buffer(),
              "desire_pulse":(desire if desire is not None else np.zeros((1,25,8),np.float16)),
              "traffic_convention":np.array([traffic],np.float16),
              "action_t":np.array([[lat_action_t,long_action_t]],np.float16)}
        for n,t in self.io.items():
            if n in feed: t.copy_(torch.from_numpy(np.ascontiguousarray(feed[n])).to(self.dev))
        with torch.cuda.stream(self.st): self.ctx.execute_async_v3(self.st.cuda_stream)
        self.st.synchronize()
        out=self.io[self.out_name].float().cpu().numpy().reshape(-1)
        # FEED BACK: append this frame's hidden_state to the recurrent queue
        hs=out[self.slices['hidden_state']].astype(np.float32)
        self.feat_q.append(hs)
        dec=self._decode(out)
        # ONLINE CALIBRATION: feed the model's pose odometry when we have a speed.
        if v_ego is not None:
            self.calibrator.handle(dec["pose"], v_ego)
            self.calib_euler=tuple(self.calibrator.rpy_smooth + CALIB_TRIM)
            # STOCK ACTION: desiredCurvature/Acceleration/shouldStop, exactly as
            # modeld derives them. On a model with an `action` head that is the head
            # itself; without one it is reconstructed from the plan's yaw and speed
            # columns. Stays None without a speed rather than defaulting to one,
            # since the command scales with the speed it is divided by.
            dec["action"]=self.action.update(
                dec["action_raw"] if self.has_action else dec["plan"],
                v_ego, lat_action_t, long_action_t)
        else:
            dec["action"]=None
        return dec

    def _decode(self,out):
        # MDN outputs pack [mu (n), std (n)]; take the mu half. plan 990 -> mu 495 = 33x15.
        o=out.astype(np.float32); s=self.slices
        def mu(name, per): return o[s[name]][:per]
        plan=mu('plan', MC.IDX_N*MC.PLAN_WIDTH).reshape(MC.IDX_N,MC.PLAN_WIDTH)
        ll=mu('lane_lines', 4*MC.IDX_N*2).reshape(4,MC.IDX_N,2)
        re_=mu('road_edges', 2*MC.IDX_N*2).reshape(2,MC.IDX_N,2)
        # THE ACTION HEAD, on models that have one. 4 floats = 2 mu + 2 std.
        #   action_raw[0]  lateral acceleration, m/s^2 -- NOT curvature. Divided by
        #                  v_ego^2 downstream; see curvature_lib.DirectAction.
        #   action_raw[1]  longitudinal acceleration, m/s^2
        # Kept on the returned dict in raw form as well as through DirectAction,
        # because the raw lateral accel is the quantity worth watching: it is
        # directly comparable to the lateral-accel limit the controller clips to,
        # in the same units, before any speed estimate has been applied to it.
        act_raw = (mu('action', ACTION_WIDTH) if self.has_action else None)
        act_std = (safe_exp(o[s['action']][ACTION_WIDTH:2*ACTION_WIDTH])
                   if self.has_action else None)
        return dict(
            # the full (33,15) mu plan. The action columns (yaw, yaw rate) are model
            # OUTPUTS -- stock steers on them directly, so anything that only forwards
            # path_xyz has thrown away what the controller actually wants.
            plan=plan,
            path_xyz=plan[:,Plan.POSITION],
            path_vel=plan[:,Plan.VELOCITY],
            path_accel=plan[:,Plan.ACCELERATION],
            plan_yaw=plan[:,Plan.T_FROM_CURRENT_EULER][:,2],
            plan_yaw_rate=plan[:,Plan.ORIENTATION_RATE][:,2],
            # (2,) [lateral accel, longitudinal accel] straight off the head, and
            # the model's own uncertainty on each. None on a model without one.
            action_raw=act_raw, action_std=act_std,
            lane_lines=ll, road_edges=re_,
            lane_prob=sigmoid(o[s['lane_lines_prob']]),
            # lead: 144 floats = 72 mu + 72 std, mu reshaping to
            # (LEAD_MHP_SELECTION=3, LEAD_TRAJ_LEN=6, LEAD_WIDTH=4) -- three lead
            # slots, each predicted at t = 0,2,4,6,8,10 s as [x, y, v, a].
            # x/y are metres in the SAME frame as path_xyz (y positive LEFT;
            # radard flips it to get radarState.yRel). radard consumes slot 0 as
            # leadOne and slot 1 as leadTwo; slot 2 goes unused.
            lead=mu('lead', 3 * MC.LEAD_TRAJ_LEN * MC.LEAD_WIDTH).reshape(
                3, MC.LEAD_TRAJ_LEN, MC.LEAD_WIDTH),
            lead_prob=sigmoid(o[s['lead_prob']]),
            pose=o[s['pose']][:6],
            hidden_state=o[s['hidden_state']],
            # What the model thinks it is DOING, as a distribution over the 8
            # Desire values (none, turnLeft, turnRight, laneChangeLeft,
            # laneChangeRight, keepLeft, keepRight, null). Softmax, matching
            # openpilot's parser -- these are logits, not probabilities.
            #
            # This closes the lane-change loop: DesireHelper leaves
            # laneChangeStarting only when laneChangeLeft+laneChangeRight drops
            # below 0.02, i.e. when the MODEL says the manoeuvre is finished.
            # Without it the state machine has no completion signal and can only
            # fall out on the 10 s LANE_CHANGE_TIME_MAX timeout.
            #
            # NB: this file is the one dashcam_web.py actually imports.
            # jetson_port/model/op_stream.py is a MIRROR -- dashcam_web puts
            # the control/ directory on sys.path, but op_stream lives in
            # that dir's model/ subdirectory, so `import op_stream` resolves
            # HERE instead. Keep the two in step or edits silently do nothing.
            desire_state=softmax(o[s['desire_state']][:MC.DESIRE_LEN]),
        )

if __name__=="__main__":
    import cv2,glob,time
    r=SupercomboRunner()
    # stream: replay /tmp/hd.jpg N times to show the recurrent buffer filling + timing
    frame=cv2.imread("/tmp/hd.jpg")
    if frame is None: frame=(np.random.rand(720,1280,3)*255).astype(np.uint8)
    print(f"FRAME_SKIP={FRAME_SKIP}, feat_q depth={FRAME_SKIP*FB_LEN}, features_buffer=(1,{FB_LEN},512)")
    t0=time.time(); N=60
    for i in range(N):
        d=r.step(frame, v_ego=20.0)   # a speed is required for the stock action path
    hz=N/(time.time()-t0)
    print(f"streaming with recurrent feedback: {hz:.1f} Hz over {N} frames")
    print(f"path reach: {d['path_xyz'][-1,0]:.1f}m  final lateral: {d['path_xyz'][-1,1]:+.2f}m")
    # stock action vs the legacy position fit, on this frame's real plan
    from curvature_lib import path_to_curvature
    a=d['action']
    print(f"action (stock): desiredCurvature={a['desiredCurvature']:+.6f} "
          f"desiredAcceleration={a['desiredAcceleration']:+.3f} shouldStop={a['shouldStop']}")
    if r.has_action:
        # The head's own numbers, and what the OLD derivation would have made of
        # the same frame's plan. They are two estimates of one quantity, so a
        # persistent gap between them is the model disagreeing with its own plan --
        # worth seeing, not worth averaging.
        raw, std = d['action_raw'], d['action_std']
        print(f"  head (raw): lat_accel={raw[0]:+.3f} m/s2 (+/- {std[0]:.3f})  "
              f"long_accel={raw[1]:+.3f} m/s2 (+/- {std[1]:.3f})")
        print(f"  same plan through the OLD plan-derived path: "
              f"{ModelAction().update(d['plan'], 20.0, LAT_ACTION_T, LONG_ACTION_T)['desiredCurvature']:+.6f}")
    print(f"  legacy path_to_curvature on the same plan: "
          f"{path_to_curvature(d['path_xyz'], 20.0):+.6f}")
    print(f"lane_prob: {d['lane_prob'].round(2)}  lead_prob: {d['lead_prob'].round(2)}")
    print(f"hidden_state norm (should be nonzero + evolving): {np.linalg.norm(d['hidden_state']):.2f}")
    # verify the recurrent queue actually filled with real features (not zeros)
    qnorm=np.linalg.norm(np.stack(list(r.feat_q),0),axis=1)
    print(f"feat_q norms: first={qnorm[0]:.1f} last={qnorm[-1]:.1f} (last should be >0 = recurrent working)")
    print("RECURRENT TEMPORAL MODEL COMPLETE: hidden_state feeds back into features_buffer.")
