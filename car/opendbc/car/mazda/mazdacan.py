import os

from opendbc.car.mazda.values import Buttons, MazdaFlags

# COPY THE CAMERA'S LANE-VISIBILITY BITS INTO OUR 0x243, instead of forcing 0.
#
# THE FAULT THIS TARGETS: "front camera sensor" on the instrument cluster. It is
# a known openpilot issue on the CX-5 and NOT specific to this port or to alpha
# long -- commaai/opendbc#1155, commaai/openpilot#25525 and #30506, all on 2022
# and 2024 Signature trims running stock openpilot. #25525 records that it clears
# on disengage/re-engage, which matches what this car does.
#
# THE MECHANISM: we replace the camera's 0x243 but FORWARD its 0x440
# CAM_LANEINFO untouched. With LINE_NOT_VISIBLE and LDW pinned to 0, the cluster
# receives our 0x243 asserting "lines visible, no lane-departure warning" right
# next to the camera's own lane data saying otherwise. A sensor-plausibility
# check has every reason to fault on that. The suggested community fix is to make
# LINE_NOT_VISIBLE match the camera, which is what this does.
#
# WHY IT IS OFF BY DEFAULT -- this is a trade, not a free win. Pinning these to 0
# is almost certainly deliberate upstream: openpilot ignores the factory camera's
# lane confidence because it has its own model. Copy the bit and, when the stock
# camera loses the lines (rain, low sun, faded paint), we relay "lines not
# visible" -- and the EPS may drop LKAS torque with it. That trades a clean dash
# for losing steering exactly when conditions are worst.
#
# So A/B it rather than assuming: watch lkas_block and eps_req vs eps_eff. Fault
# gone and lkas_block still 0 is a win; lkas_block starting to fire is the cost,
# and reverting needs no rebuild.
_COPY_CAM_LINES = os.environ.get("OP_LKAS_COPY_CAM_LINES", "0") == "1"


def create_steering_control(packer, CP, frame, apply_torque, lkas, steering_angle=0):

  tmp = apply_torque + 2048

  lo = tmp & 0xFF
  hi = tmp >> 8

  # copy values from camera
  b1 = int(lkas["BIT_1"])
  er1 = int(lkas["ERR_BIT_1"])
  # These two are pinned to 0 upstream despite the comment above -- see the note
  # at _COPY_CAM_LINES. The checksum below already folds lnv and ldw in, so
  # copying them needs no other change.
  lnv = int(lkas["LINE_NOT_VISIBLE"]) if _COPY_CAM_LINES else 0
  ldw = int(lkas["LDW"]) if _COPY_CAM_LINES else 0
  er2 = int(lkas["ERR_BIT_2"])

  # Some older models do have these, newer models don't.
  # Either way, they all work just fine if set to zero -- which is why the stock
  # default for `steering_angle` is 0 and every stock caller leaves it there.
  #
  # The Jetson port passes the model's desired steering-wheel angle here (raw DBC
  # units, 12-bit two's-complement-ish via the +2048 offset). ANGLE_ENABLED stays
  # 0: this is the angle *report* field, not a request to switch the EPS into
  # angle control, and nothing in the panda safety model validates it.
  #
  # NOTE on the checksum below: it is reverse-engineered, and the `ahi == 1`
  # correction shows it was only ever fitted against captures. With
  # steering_angle == 0, ahi is always 2; other values exercise ahi 0/1/3, which
  # are NOT validated against a real camera. Clamp here so a bad model output can
  # never push the field (and therefore the checksum) outside the DBC range.
  steering_angle = int(max(-2048, min(2047, steering_angle)))
  b2 = 0

  tmp = steering_angle + 2048
  ahi = tmp >> 10
  amd = (tmp & 0x3FF) >> 2
  amd = (amd >> 4) | ((amd & 0xF) << 4)
  alo = (tmp & 0x3) << 2

  ctr = frame % 16
  # bytes:     [    1  ] [ 2 ] [             3               ]  [           4         ]
  csum = 249 - ctr - hi - lo - (lnv << 3) - er1 - (ldw << 7) - (er2 << 4) - (b1 << 5)

  # bytes      [ 5 ] [ 6 ] [    7   ]
  csum = csum - ahi - amd - alo - b2

  if ahi == 1:
    csum = csum + 15

  if csum < 0:
    if csum < -256:
      csum = csum + 512
    else:
      csum = csum + 256

  csum = csum % 256

  values = {}
  if CP.flags & MazdaFlags.GEN1:
    values = {
      "LKAS_REQUEST": apply_torque,
      "CTR": ctr,
      "ERR_BIT_1": er1,
      "LINE_NOT_VISIBLE": lnv,
      "LDW": ldw,
      "BIT_1": b1,
      "ERR_BIT_2": er2,
      "STEERING_ANGLE": steering_angle,
      "ANGLE_ENABLED": b2,
      "CHKSUM": csum
    }

  return packer.make_can_msg("CAM_LKAS", 0, values)


def create_alert_command(packer, cam_msg: dict, ldw: bool, steer_required: bool):
  values = {s: cam_msg[s] for s in [
    "LINE_VISIBLE",
    "LINE_NOT_VISIBLE",
    "LANE_LINES",
    "BIT1",
    "BIT2",
    "BIT3",
    "NO_ERR_BIT",
    "S1",
    "S1_HBEAM",
  ]}
  values.update({
    # TODO: what's the difference between all these? do we need to send all?
    "HANDS_WARN_3_BITS": 0b111 if steer_required else 0,
    "HANDS_ON_STEER_WARN": steer_required,
    "HANDS_ON_STEER_WARN_2": steer_required,

    # TODO: right lane works, left doesn't
    # TODO: need to do something about L/R
    "LDW_WARN_LL": 0,
    "LDW_WARN_RL": 0,
  })
  return packer.make_can_msg("CAM_LANEINFO", 0, values)


def create_button_cmd(packer, CP, counter, button):

  can = int(button == Buttons.CANCEL)
  res = int(button == Buttons.RESUME)
  # SET_PLUS / SET_MINUS were declared in Buttons and then never reachable: both
  # were pinned to 0 below, so create_button_cmd could only ever emit CANCEL or
  # RESUME. The DBC carries the fields (SET_P/SET_P_INV, SET_M/SET_M_INV) and the
  # panda permits CRZ_BTNS in BOTH safety tables with check_relay=false, so this
  # was a gap in the packer alone.
  #
  # WHY IT MATTERS: it is the only route to longitudinal that does NOT suppress the
  # radar. Alpha long mutes the radar to take 0x21b/0x21c, and the radar is the
  # FCW/AEB/SBS ECU -- so the cluster malfunction under alpha long is TRUE and
  # cannot be removed without lying about AEB. Moving the factory ACC's set speed
  # with these buttons leaves the radar running: it keeps following, keeps braking,
  # keeps its safety functions, and the dash has nothing to report.
  setp = int(button == Buttons.SET_PLUS)
  setm = int(button == Buttons.SET_MINUS)

  if CP.flags & MazdaFlags.GEN1:
    values = {
      "CAN_OFF": can,
      "CAN_OFF_INV": (can + 1) % 2,

      "SET_P": setp,
      "SET_P_INV": (setp + 1) % 2,

      "RES": res,
      "RES_INV": (res + 1) % 2,

      "SET_M": setm,
      "SET_M_INV": (setm + 1) % 2,

      "DISTANCE_LESS": 0,
      "DISTANCE_LESS_INV": 1,

      "DISTANCE_MORE": 0,
      "DISTANCE_MORE_INV": 1,

      "MODE_X": 0,
      "MODE_X_INV": 1,

      "MODE_Y": 0,
      "MODE_Y_INV": 1,

      "BIT1": 1,
      "BIT2": 1,
      "BIT3": 1,
      "CTR": (counter + 1) % 16,
    }

    return packer.make_can_msg("CRZ_BTNS", 0, values)
