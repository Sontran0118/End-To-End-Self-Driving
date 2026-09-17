"""Mazda alpha longitudinal: silence the radar, then speak in its place.

Vendored verbatim from yummydirtx/opendbc @ mazda-longitudinal-upstream (the work
described in community.sunnypilot.ai thread 1482, on-car results 2026-03-20, stop
and go solved 2026-04-23). Kept as a file rather than a patch because it is a new
module with no upstream counterpart at opendbc 8ddffb37, and because the branch it
came from is one force-push away from disappearing.

HOW IT WORKS. The factory radar at UDS 0x764 owns longitudinal: it sends 0x21b
CRZ_INFO (carrying ACCEL_CMD) and 0x21c CRZ_CTRL, and the PCM obeys them. Put the
radar into a UDS programming session and hold it there with tester-present, and it
stops transmitting -- at which point those two frames are ours. Positive ACCEL_CMD
accelerates, negative brakes.

WHAT IT COSTS. The radar is also FCW, AEB and SBS. All of them are gone while it
is suppressed, and the cluster will show malfunctions saying so. Camera-based
low-speed SCBS survives; radar AEB does not.
"""
from __future__ import annotations

from enum import StrEnum

from opendbc.can.dbc import DBC
from opendbc.can.packer import set_value
from opendbc.car import make_tester_present_msg, uds
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery


MAZDA_LONG_DBC = DBC("mazda_2017")

RADAR_ADDR = 0x764
RADAR_BUS = 0

CRZ_INFO_ADDR = 0x21B
CRZ_CTRL_ADDR = 0x21C

# The radar's own not-engaged CRZ_INFO, VERIFIED byte-for-byte against this car
# 2026-08-12: the radar's idle frame is
#     01 ff e3 ff c0 80 08 d5
# with byte 6 the CTR1 counter and byte 7 the checksum, so bytes 0-5 are the
# template. Note byte 5 = 0x80, i.e. MYSTERY_BIT (47|1) is SET at idle -- the
# version of this template carried at 019555a9 had 0x00 there and did not match.
# CRZ_INFO_TEMPLATE (engaged) sets the same bit, which is the hint that it is not
# an engaged-only field.
#
# That same capture also validated _compute_inverted_sum_checksum against real
# hardware for the first time: sum(bytes 0..6) & 0xff = 0x2a, 0xff - 0x2a = 0xd5,
# which is exactly the byte the radar sent.
CRZ_INFO_STANDBY_TEMPLATE = bytes.fromhex("01ffe3ffc0800000")
CRZ_INFO_TEMPLATE = bytes.fromhex("01ffe20006800000")

LONG_COMMAND_STEP = 2
TESTER_PRESENT_STEP = 50

ACCEL_CMD_MAX = 2000.0
ACCEL_CMD_MIN = -2000.0
HOLD_BRAKE_CMD_TARGET = -1024.0
HOLD_LATCHED_CMD_TARGET = -1.0
NEAR_STOP_BRAKE_CMD_TARGET = -750.0
NEAR_STOP_ENTRY_SPEED = 1.0
ACTIVE_STOP_CHECKSUM_BIAS = 0x04

# Stock Mazda longitudinal is not using one global raw-command scale across all
# speeds. Keep more authority at low/mid speed, and soften the map at highway
# speed where the single-scale version feels jerky.
ACCEL_SCALE_UP_BP = (0.0, 4.2, 11.1, 22.2)
ACCEL_SCALE_UP_V = (1000.0, 1000.0, 950.0, 800.0)
ACCEL_SCALE_DOWN_BP = (0.0, 1.4, 5.6, 22.2)
ACCEL_SCALE_DOWN_V = (1200.0, 1000.0, 925.0, 950.0)


class MazdaLongitudinalProfile(StrEnum):
  STANDBY = "standby"
  ENGAGED_CRUISE = "engaged_cruise"
  ENGAGED_FOLLOW = "engaged_follow"
  STOP_GO_HOLD = "stop_go_hold"
  STOP_GO_HOLD_LATCHED = "stop_go_hold_latched"


CRZ_CTRL_TEMPLATES: dict[MazdaLongitudinalProfile, bytes] = {
  MazdaLongitudinalProfile.STANDBY: bytes.fromhex("02010b0000000000"),
  MazdaLongitudinalProfile.ENGAGED_CRUISE: bytes.fromhex("0a018b2000001000"),
  MazdaLongitudinalProfile.ENGAGED_FOLLOW: bytes.fromhex("0a018b4000001000"),
  MazdaLongitudinalProfile.STOP_GO_HOLD: bytes.fromhex("0a018b6000001000"),
  MazdaLongitudinalProfile.STOP_GO_HOLD_LATCHED: bytes.fromhex("0a018b8000001000"),
}


def _get_signal(message_name: str, signal_name: str):
  return MAZDA_LONG_DBC.name_to_msg[message_name].sigs[signal_name]


def _patch_signal(message_name: str, raw: bytes, signal_name: str, value: float) -> bytes:
  sig = _get_signal(message_name, signal_name)
  encoded = int(round((value - sig.offset) / sig.factor))
  if encoded < 0:
    encoded = (1 << sig.size) + encoded

  dat = bytearray(raw)
  set_value(dat, sig, encoded)
  return bytes(dat)


def _compute_inverted_sum_checksum(raw: bytes, checksum_index: int = 7) -> int:
  return (0xFF - (sum(raw[i] for i in range(len(raw)) if i != checksum_index) & 0xFF)) & 0xFF


def _update_crz_info_checksum(raw: bytes, bias: int = 0) -> bytes:
  dat = bytearray(raw)
  dat[7] = (_compute_inverted_sum_checksum(dat) + bias) & 0xFF
  return bytes(dat)


def clip(value: float, lower: float, upper: float) -> float:
  return min(max(value, lower), upper)


def _interp_scale(v_ego: float, bp: tuple[float, ...], values: tuple[float, ...]) -> float:
  if v_ego <= bp[0]:
    return values[0]
  if v_ego >= bp[-1]:
    return values[-1]

  for i in range(1, len(bp)):
    if v_ego <= bp[i]:
      x0, x1 = bp[i - 1], bp[i]
      y0, y1 = values[i - 1], values[i]
      ratio = (v_ego - x0) / (x1 - x0)
      return y0 + (y1 - y0) * ratio

  return values[-1]


def accel_to_accel_cmd(accel: float, v_ego: float) -> int:
  scale = _interp_scale(v_ego, ACCEL_SCALE_UP_BP, ACCEL_SCALE_UP_V) if accel >= 0.0 else _interp_scale(v_ego, ACCEL_SCALE_DOWN_BP, ACCEL_SCALE_DOWN_V)
  return int(round(clip(accel * scale, ACCEL_CMD_MIN, ACCEL_CMD_MAX)))


def hold_brake_accel() -> float:
  # Stock HOLD keeps a strong negative CRZ_INFO command alive through the
  # active stop/hold phase until the chassis hold latch takes over.
  # Keep the raw target approximately constant as scales change.
  return HOLD_BRAKE_CMD_TARGET / ACCEL_SCALE_DOWN_V[0]


def hold_latched_accel() -> float:
  # Once the chassis hold latch takes over, stock CRZ_INFO.ACCEL_CMD relaxes
  # back near zero and the stop bits clear.
  return HOLD_LATCHED_CMD_TARGET / ACCEL_SCALE_DOWN_V[0]


def near_stop_brake_accel(v_ego: float) -> float:
  # Stock stop-to-hold ramps into the final HOLD brake command before true
  # standstill, rather than waiting until the speed bit drops to zero.
  ratio = clip(v_ego / NEAR_STOP_ENTRY_SPEED, 0.0, 1.0)
  target = HOLD_BRAKE_CMD_TARGET + (NEAR_STOP_BRAKE_CMD_TARGET - HOLD_BRAKE_CMD_TARGET) * ratio
  return target / ACCEL_SCALE_DOWN_V[0]


def build_crz_info(accel: float, counter: int, long_active: bool, hold_request: bool, v_ego: float,
                   hold_latched: bool = False, acc_set_allowed: bool = True,
                   resume_unlatching: bool = False) -> bytes:
  # NOT ENGAGED -> send the radar's IDLE frame, not an engaged frame carrying a
  # zero command. These are different statements and the PCM sees the difference.
  #
  # MEASURED 2026-08-12 on this car, LONGDIFF at the transmit point, same instant:
  #     radar = 01 ff e3 ff c0 80 08 d5     <- idle: "no request"
  #     ours  = 01 ff e2 00 04 80 00 99     <- ACCEL_CMD encoded as exactly 0.0
  # Bytes 0-4 of the radar's idle frame are byte-for-byte CRZ_INFO_STANDBY_TEMPLATE
  # below. The engaged template with ACCEL_CMD=0 is NOT the same message: 0 is a
  # live command for zero acceleration, where 0xffc0 is the sentinel that says
  # nothing is being commanded at all.
  #
  # This branch existed at 019555a9 (go-no-malfunction) and was lost in the
  # 2026-08-11 revert to c94780cb, which restored the pristine module and kept
  # only the MSG_1 quad fix on top. Since that revert the port has announced a
  # live zero command in every not-engaged frame it has ever sent -- including
  # every frame of both A/B runs on 2026-08-12, neither of which engaged.
  # Prefer the radar's OWN idle frame over the template. MYSTERY_BIT (byte 5
  # bit 7) was 0x80 in one capture of this car and 0x00 in the next minutes
  # later, so it is not constant and no hardcoded template tracks it. Same
  # reasoning, and the same mechanism, as _OBSERVED_CRZ_CTRL above.
  #
  # This is still a SNAPSHOT: dashcam_web only feeds set_observed_crz_info while
  # we are not yet transmitting, so what gets replayed is whatever the radar was
  # saying at arming. That is strictly better than a guess from another car, and
  # it is bounded -- but it is not live, and a field that moves on a timescale
  # longer than the arming window will still go stale.
  if not long_active:
    base = _OBSERVED_CRZ_INFO.get(False) or CRZ_INFO_STANDBY_TEMPLATE
    raw = _patch_signal("CRZ_INFO", base, "CTR1", counter % 16)
    return _update_crz_info_checksum(raw)

  stopping_active = hold_request and not hold_latched
  raw = _patch_signal("CRZ_INFO", CRZ_INFO_TEMPLATE, "ACCEL_CMD", accel_to_accel_cmd(accel, v_ego))
  raw = _patch_signal("CRZ_INFO", raw, "ACC_ACTIVE", int(long_active))
  raw = _patch_signal("CRZ_INFO", raw, "ACC_SET_ALLOWED", int(acc_set_allowed))
  raw = _patch_signal("CRZ_INFO", raw, "CRZ_ENDED", 0)
  raw = _patch_signal("CRZ_INFO", raw, "STOPPING_MAYBE", int(stopping_active))
  raw = _patch_signal("CRZ_INFO", raw, "STOPPING_MAYBE2", int(stopping_active))
  raw = _patch_signal("CRZ_INFO", raw, "RESUME_UNLATCHING_MAYBE", int(resume_unlatching))
  raw = _patch_signal("CRZ_INFO", raw, "CTR1", counter % 16)
  checksum_bias = ACTIVE_STOP_CHECKSUM_BIAS if stopping_active else 0
  return _update_crz_info_checksum(raw, bias=checksum_bias)


def select_profile(long_active: bool, lead_visible: bool, hold_request: bool,
                   crz_hold_latched: bool) -> MazdaLongitudinalProfile:
  if not long_active:
    return MazdaLongitudinalProfile.STANDBY
  if hold_request and crz_hold_latched:
    return MazdaLongitudinalProfile.STOP_GO_HOLD_LATCHED
  if hold_request:
    return MazdaLongitudinalProfile.STOP_GO_HOLD
  if lead_visible:
    return MazdaLongitudinalProfile.ENGAGED_FOLLOW
  return MazdaLongitudinalProfile.ENGAGED_CRUISE


# DRIVER'S FOLLOW-DISTANCE SETTING, observed from the radar rather than assumed.
#
# Every CRZ_CTRL template below hardcodes DISTANCE_SETTING = 2 (byte 2 bits 4..2,
# DBC bits 20..18). MEASURED 2026-08-11 against the radar's own frame, captured
# immediately before suppression:
#
#     radar 0x21c = 02 01 13 00 00 00 00 00   -> DISTANCE_SETTING = 4
#     ours  0x21c = 02 01 0b 00 00 00 00 00   -> DISTANCE_SETTING = 2
#
# That was the ONLY differing byte in the whole frame. So the moment we take over,
# the cluster is told the follow distance jumped from what the driver selected to
# 2 -- and it is the cluster's own displayed setting, so it has every reason to
# disagree with itself. The front camera warning flickering on and off appeared
# exactly when this frame started coming from us.
#
# Set from the pre-suppression capture; falls back to the template value when we
# never saw the radar (car asleep at launch), which is the old behaviour.
_OBSERVED_DISTANCE_SETTING: int | None = None


# The radar's byte 2 is STATE, not a constant. MEASURED 2026-08-11 on the same car
# minutes apart:
#
#     0x01  cruise unavailable : CRZ_AVAILABLE=0, DISTANCE_SETTING=0
#     0x13  cruise available   : CRZ_AVAILABLE=1, DISTANCE_SETTING=4
#     0x0b  OUR template       : CRZ_AVAILABLE=1, DISTANCE_SETTING=2  (always)
#
# So every frame we send asserts "cruise is available" even when the car says it is
# not, and asserts a follow distance the driver never chose. Both are fields the
# cluster displays, so it is being handed a contradiction continuously -- which is
# a far better fit for an intermittent warning than a static mismatch would be.
#
# Mirror the observed patterns instead: remember the radar's byte 2 for each state
# and reproduce whichever one matches reality now.
_OBSERVED_BYTE2: dict[bool, int] = {}


# FULL observed frames, not just byte 2. Patching one byte at a time does not
# converge -- each capture exposed another field the radar drives and our template
# pins. MEASURED 2026-08-11, captures of the radar's own 0x21c on one car minutes
# apart:
#
#     02 01 13 00 00 00 00 00      cruise available, DISTANCE_SETTING 4
#     02 01 01 00 00 00 00 00      cruise unavailable
#     03 00 01 00 00 01 00 00      later, same nominal state
#     02 01 0b 00 00 00 00 00      OURS, unconditionally
#
# Bytes 0, 1, 2 and 5 all move. Byte 0 carries MSG_1/MSG_1_INV -- a value and its
# inverse, i.e. a consistency pair a receiver can verify -- so a frozen guess there
# is exactly what a checking PCM would reject.
#
# So stop guessing the base frame. Use the radar's own most recent one and apply
# only the DELTA the profile needs. The delta is computed against the STANDBY
# template, so what gets applied is precisely "what changes when openpilot engages"
# -- the stop-and-go phase bits that took the community a month to derive -- while
# every field we have no business inventing keeps the radar's value.
_OBSERVED_CRZ_CTRL: dict[bool, bytes] = {}


# The same idea for CRZ_INFO, and the call site has been there the whole time:
# dashcam_web.py's can_thread calls set_observed_crz_info() on every radar 0x21b
# seen before we start transmitting -- inside a bare `except Exception: pass`.
# This function did not exist, so that call raised AttributeError on every frame
# and was silently swallowed. MEASURED CONSEQUENCE 2026-08-12: 0x21c reproduced
# the radar byte-for-byte (its observed path is live) while 0x21b never has.
#
# Keyed on ACC_ACTIVE (33|1 -> byte 4, bit 1), so the idle frame and an engaged
# one do not overwrite each other.
_OBSERVED_CRZ_INFO: dict[bool, bytes] = {}


def set_observed_crz_info(raw: bytes) -> None:
  """Remember the radar's whole CRZ_INFO, keyed on the ACC_ACTIVE bit."""
  if len(raw) >= 8:
    _OBSERVED_CRZ_INFO[bool((raw[4] >> 1) & 1)] = bytes(raw)


def observed_crz_info(acc_active: bool = False) -> bytes | None:
  return _OBSERVED_CRZ_INFO.get(bool(acc_active))


def set_observed_crz_ctrl(raw: bytes) -> None:
  """Remember the radar's whole CRZ_CTRL, keyed on the CRZ_AVAILABLE bit."""
  if len(raw) >= 8:
    key = bool((raw[2] >> 1) & 1)
    _OBSERVED_CRZ_CTRL[key] = bytes(raw)
    _OBSERVED_BYTE2[key] = raw[2]


def _apply_profile_delta(observed: bytes, profile: MazdaLongitudinalProfile) -> bytes:
  """Radar's frame + (profile - STANDBY). Fields the profile does not touch stay."""
  base = CRZ_CTRL_TEMPLATES[MazdaLongitudinalProfile.STANDBY]
  want = CRZ_CTRL_TEMPLATES[profile]
  out = bytearray(observed)
  for i in range(min(len(out), len(base), len(want))):
    delta = base[i] ^ want[i]          # exactly the bits the profile changes
    if delta:
      out[i] = (out[i] & ~delta) | (want[i] & delta)
  return bytes(out)


_CRUISE_AVAILABLE = False


def set_cruise_available(available: bool) -> None:
  """The car's REAL cruise availability, from a source we do not generate.

  Under alpha long we own 0x21c, so reading availability back off that frame would
  be reading our own output. PEDALS/acc_armed is the PCM's own report and is what
  the panda gates on, so the two stay in agreement.
  """
  global _CRUISE_AVAILABLE
  _CRUISE_AVAILABLE = bool(available)


def set_observed_distance_setting(value: int | None) -> None:
  """Record the driver's follow-distance setting as the radar reported it."""
  global _OBSERVED_DISTANCE_SETTING
  if value is not None and 0 <= value <= 7:
    _OBSERVED_DISTANCE_SETTING = int(value)


def distance_setting_from_frame(raw: bytes) -> int | None:
  """DISTANCE_SETTING out of a raw CRZ_CTRL payload (DBC 20|3@0+)."""
  if len(raw) < 3:
    return None
  return (raw[2] >> 2) & 0x07


def build_crz_ctrl(long_active: bool, lead_visible: bool, hold_request: bool, hold_latched: bool,
                   crz_hold_latched: bool = False, crz_hold_passive: bool = False,
                   crz_resume_active: bool = False) -> bytes:
  # Stock stop-and-go progresses through multiple CRZ_CTRL stop phases. Mirror
  # that sequence so the synthetic path keeps the same latch states as stock.
  lead_visible = lead_visible or hold_request or hold_latched or crz_hold_latched or crz_hold_passive
  raw = CRZ_CTRL_TEMPLATES[select_profile(long_active, lead_visible, hold_request, crz_hold_latched)]
  # Mirror the radar's byte 2 for the state we are actually in. Only the bits the
  # radar owns there are taken (CRZ_AVAILABLE and DISTANCE_SETTING); the profile
  # templates keep everything else, since those encode the stop-and-go phases that
  # took the community a month to get right.
  observed_full = _OBSERVED_CRZ_CTRL.get(_CRUISE_AVAILABLE)
  observed = _OBSERVED_BYTE2.get(_CRUISE_AVAILABLE)
  if observed_full is not None:
    # Preferred: the radar's own frame, with only the profile's bits changed.
    raw = _apply_profile_delta(
        observed_full,
        select_profile(long_active, lead_visible, hold_request, crz_hold_latched))
  elif observed is not None:
    dat = bytearray(raw)
    MASK = 0x1E                      # bit1 CRZ_AVAILABLE + bits4..2 DISTANCE_SETTING
    dat[2] = (dat[2] & ~MASK) | (observed & MASK)
    raw = bytes(dat)
  elif _OBSERVED_DISTANCE_SETTING is not None:
    dat = bytearray(raw)
    dat[2] = (dat[2] & ~0x1C) | ((_OBSERVED_DISTANCE_SETTING & 0x07) << 2)
    raw = bytes(dat)
  raw = _patch_signal("CRZ_CTRL", raw, "CRZ_ACTIVE", int(long_active))
  raw = _patch_signal("CRZ_CTRL", raw, "ACC_ACTIVE_2", int(long_active and not crz_hold_passive))
  raw = _patch_signal("CRZ_CTRL", raw, "DISABLE_TIMER_1", 0)
  raw = _patch_signal("CRZ_CTRL", raw, "DISABLE_TIMER_2", 0)
  raw = _patch_signal("CRZ_CTRL", raw, "RADAR_HAS_LEAD", int(lead_visible))
  # Stock resume transitions rely on more than the coarse 0x21c templates. The
  # live radar path preserves these fields automatically, but the synthetic path
  # has to set them explicitly to match passive hold (distance 4), active
  # stop-go / resume (distance 3 + ACC_GAS_MAYBE2), and follow (distance 2).
  if crz_hold_passive or crz_hold_latched:
    raw = _patch_signal("CRZ_CTRL", raw, "RADAR_LEAD_RELATIVE_DISTANCE", 4)
    raw = _patch_signal("CRZ_CTRL", raw, "ACC_GAS_MAYBE2", 0)
  elif hold_request or crz_resume_active:
    raw = _patch_signal("CRZ_CTRL", raw, "RADAR_LEAD_RELATIVE_DISTANCE", 3)
    raw = _patch_signal("CRZ_CTRL", raw, "ACC_GAS_MAYBE2", 1)
  elif not lead_visible:
    # NO LEAD MEANS NO LEAD DISTANCE. Without this the frame contradicts itself:
    # RADAR_HAS_LEAD says 0 while RADAR_LEAD_RELATIVE_DISTANCE says 1, i.e.
    # "there is no lead, and the lead is 1 away".
    #
    # It comes from upstream's ENGAGED_CRUISE template (0a018b2000001000), whose
    # byte 3 is 0x20 -- and _apply_profile_delta ORs that onto the radar's own
    # frame, so it survives even when we are otherwise reproducing the radar
    # byte-for-byte. MEASURED 2026-08-12 against the real activation in
    # drivelogs/acc_activation.GROUNDTRUTH.jsonl:
    #     radar engaged : 0b 00 13 00 00 01 10 00
    #     ours  engaged : 0b 00 13 20 00 01 10 00
    #                              ^^ the only differing byte
    # The radar sends 0 here whenever it is not tracking a lead. Every engaged
    # frame this port has ever sent carried the contradiction, which is exactly
    # the kind of thing a PCM cross-check would reject -- and the PCM ignoring
    # our commands while accepting the radar's is the open symptom.
    raw = _patch_signal("CRZ_CTRL", raw, "RADAR_LEAD_RELATIVE_DISTANCE", 0)
  # THE REDUNDANCY QUAD, asserted last so no template or branch above can leave
  # it wrong. MSG_1 / MSG_1_INV / MSG_1_COPY / MSG_1_INV_COPY is a bit plus its
  # inverse, duplicated -- the shape a receiver uses to reject a corrupted frame.
  #
  # MEASURED 2026-08-11 from a passive capture of this car's radar across a stock
  # ACC activation (jetson_port/capture_acc_activation.py, ground truth kept at
  # drivelogs/acc_activation.GROUNDTRUTH.jsonl). Every distinct 0x21c the radar
  # sent in that window:
  #
  #   03 00 01 00 00 01 00 00  n=1502  MAIN off
  #   03 00 13 00 00 01 00 00  n=130   MAIN on, ready
  #   0b 00 13 00 00 01 10 00  n=72    ENGAGED
  #
  # MSG_1=1 and MSG_1_INV=1 on all 1903 non-zero frames, in every state. The
  # CRZ_CTRL_TEMPLATES above carry MSG_1=0 with MSG_1_INV_COPY=1 -- the inverse on
  # both halves of the pair.
  #
  # This file usually derives 0x21c from the radar's own observed frame, which
  # inherits the quad for free; these templates are the fallback for when no
  # observation was captured, and that fallback was silently wrong. Assert it here
  # so the derived path and the fallback path cannot disagree.
  #
  # NB: the PCM ignoring our frames is CONSISTENT with failing this check but not
  # proven by it -- MEASURED the same evening, longActive=1 with ACCEL_CMD=+2.0
  # from a standstill left the engine at idle 646 rpm and acc_active flat 0. Still
  # unverified on the car: the drive ended before a SET press could be tried.
  raw = _patch_signal("CRZ_CTRL", raw, "MSG_1", 1)
  raw = _patch_signal("CRZ_CTRL", raw, "MSG_1_INV", 1)
  raw = _patch_signal("CRZ_CTRL", raw, "MSG_1_INV_COPY", 0)
  return raw


def create_longitudinal_messages(bus: int, accel: float, counter: int, long_active: bool,
                                 lead_visible: bool, *, hold_request: bool = False,
                                 crz_ctrl_hold_request: bool | None = None,
                                 hold_latched: bool = False, crz_hold_latched: bool = False,
                                 crz_hold_passive: bool = False,
                                 crz_resume_active: bool = False,
                                 crz_info_resume_unlatching: bool = False,
                                 v_ego: float = 0.0) -> list[CanData]:
  if crz_ctrl_hold_request is None:
    crz_ctrl_hold_request = hold_request

  return [
    CanData(CRZ_INFO_ADDR, build_crz_info(accel, counter, long_active, hold_request, v_ego,
                                          hold_latched=hold_latched,
                                          resume_unlatching=crz_info_resume_unlatching), bus),
    CanData(CRZ_CTRL_ADDR, build_crz_ctrl(long_active, lead_visible, crz_ctrl_hold_request, hold_latched,
                                          crz_hold_latched=crz_hold_latched,
                                          crz_hold_passive=crz_hold_passive,
                                          crz_resume_active=crz_resume_active), bus),
  ]


def create_radar_tester_present(bus: int = RADAR_BUS) -> CanData:
  return make_tester_present_msg(RADAR_ADDR, bus, suppress_response=True)


def _uds_request(can_recv, can_send, bus: int, addr: int, request: bytes, response: bytes,
                 *, timeout: float = 0.1) -> bool:
  query = IsoTpParallelQuery(can_send, can_recv, bus, [(addr, None)], [request], [response])
  return len(query.get_data(timeout)) > 0


def enter_radar_programming_session(can_recv, can_send, bus: int = RADAR_BUS, addr: int = RADAR_ADDR,
                                    retry: int = 5) -> bool:
  request = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.PROGRAMMING])
  response = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, uds.SESSION_TYPE.PROGRAMMING])

  for attempt in range(retry):
    try:
      if _uds_request(can_recv, can_send, bus, addr, request, response):
        carlog.warning(f"mazda radar programming session enabled on {hex(addr)}")
        return True
    except Exception:
      carlog.exception("mazda radar programming session exception")
    carlog.error(f"mazda radar programming session retry ({attempt + 1})")

  carlog.error("mazda radar programming session failed")
  return False


# CommunicationControl's communicationType byte: bit 0 = normal communication
# messages, bit 1 = network management. 0x01 is normal only, which is what
# carries 0x21b/0x21c. Network management is left alone deliberately -- silencing
# that is how you make other modules decide the radar has dropped off the bus,
# which is the fault we are trying to avoid in the first place.
_COMM_TYPE_NORMAL = 0x01
# ENGINE_DATA. Used only as an "is the car actually awake" witness: a silent 0x21b
# means nothing if the radar has no power, and every silence test in this file has
# to be able to tell those two apart.
_ENGINE_DATA_ADDR = 0x202


def enter_radar_standby(can_recv, can_send, bus: int = RADAR_BUS, addr: int = RADAR_ADDR,
                        retry: int = 5, disable_dtc: bool = True) -> bool:
  """Silence the radar with CommunicationControl instead of a programming session.

  WHY THIS EXISTS. enter_radar_programming_session() works -- the radar does stop
  transmitting and 0x21b/0x21c become ours -- but it is the heaviest tool ISO
  14229 offers, and it is very likely why the dash lights up. To every other
  module on the bus, an ECU in programmingSession is one that is BEING REFLASHED:
  it stops answering normal requests, so the camera, the cluster and the PCM each
  log their own communication DTC against it and report a malfunction.

  0x28 CommunicationControl is the service actually meant for this. With
  ENABLE_RX_DISABLE_TX the radar keeps listening, keeps running its application
  and keeps answering diagnostics -- it simply stops putting its normal messages
  on the wire. From the network's point of view it is present and healthy, which
  is the difference that should keep the fault off the cluster.

  0x85 ControlDTCSetting OFF is sent first, which is what a workshop tool does
  before any procedure expected to upset other modules: it asks the ECU not to
  store fault codes while this is going on.

  WHAT THIS DOES NOT CHANGE: the radar still cannot command FCW, AEB or SBS,
  because it cannot transmit. Those functions are gone either way -- that is
  inherent to taking over its messages, not a property of how it was silenced.
  What may improve is whether the car COMPLAINS about it.

  0x28 is manufacturer-specific and the radar is free to NAK it. Each step is
  checked separately so the caller learns which one failed rather than getting a
  bare False, and the caller can fall back to the programming session.
  """
  # A LADDER, LEAST INVASIVE FIRST.
  #
  # The first version assumed 0x28 needs an extendedDiagnosticSession, and gave
  # up when that was refused. MEASURED 2026-08-09 on this car: 0x10 03 got NO
  # REPLY AT ALL, five attempts, while 0x10 02 (programming) was accepted
  # instantly -- "06 50 02 00 19 01 f4 00" on the 0x76C tap. So the radar does
  # answer UDS; it just does not offer the session we asked for. Assuming one
  # session is the only route to 0x28 is what made standby unreachable.
  #
  # So try each rung and report which one the radar actually accepts:
  #   1. bare 0x28 in whatever session it is already in. Many ECUs allow
  #      CommunicationControl in the default session, and if this works we have
  #      changed no session state at all -- the least invasive outcome available.
  #   2. extendedDiagnosticSession, then 0x28. The textbook route.
  #   3. safetySystemDiagnosticSession, then 0x28. A radar is a safety-system
  #      ECU, so this is the session it is most likely to actually implement.
  #
  # Any rung that lands leaves the radar present and answering on the bus, which
  # is the whole point. If all three fail, the caller falls back to the
  # programming session and we are no worse off than before.
  cc_req = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL,
                  uds.CONTROL_TYPE.ENABLE_RX_DISABLE_TX, _COMM_TYPE_NORMAL])
  cc_resp = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL + 0x40,
                   uds.CONTROL_TYPE.ENABLE_RX_DISABLE_TX])

  def _try_session(session: int, label: str) -> bool:
    req = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, session])
    resp = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, session])
    try:
      # Longer than the 0.1 s default: the radar advertised P2 = 0x0019 (25 ms)
      # but P2* = 0x01F4 (500 ms), so anything answered with a response-pending
      # needs room. A timeout here reads as "unsupported" and skips a rung that
      # might have worked.
      if _uds_request(can_recv, can_send, bus, addr, req, resp, timeout=0.3):
        carlog.warning(f"mazda radar {label} session accepted on {hex(addr)}")
        return True
    except Exception:
      carlog.exception(f"mazda radar {label} session exception")
    carlog.error(f"mazda radar {label} session refused")
    return False

  def _try_comm_control() -> bool:
    for attempt in range(retry):
      try:
        if _uds_request(can_recv, can_send, bus, addr, cc_req, cc_resp, timeout=0.3):
          return True
      except Exception:
        carlog.exception("mazda radar standby exception")
      carlog.error(f"mazda radar standby retry ({attempt + 1})")
    return False

  def _try_comm_control_canonical(quiet_window: float = 1.2) -> bool:
    """openpilot's own disable_ecu form: 28 83 01, FIRE AND FORGET.

    THIS IS THE ONE THAT MATTERS, and the reason every rung above reported "no
    reply at all" instead of a NAK.

    opendbc/car/disable_ecu.py -- the helper openpilot uses to silence radars
    across Honda Bosch, Hyundai and others -- sends:

        com_cont_req      = b'\\x28\\x83\\x01'
        COM_CONT_RESPONSE = b''          # NO response expected
        query.get_data(0)                # timeout 0

    Two differences from _try_comm_control above, both decisive:

      0x83 vs 0x01 -- controlType 3 (disableRxAndTx) with bit 7 set. Bit 7 is
      suppressPosRspMsgIndication: it tells the ECU NOT to answer. So a radar that
      accepts this command is REQUIRED BY THE PROTOCOL to stay silent, and the
      code above -- which waits for a positive response with a 0.3 s timeout and
      retries five times -- scores a perfectly executed command as a failure.

      MEASURED 2026-08-11: the 0x76c tap saw exactly two frames in a whole run,
      both positive responses to 10 02, and NOTHING for any 0x28 attempt -- not
      even a negative response. "No NAK and no ACK" is precisely what a suppressed
      response looks like. We may have been silencing this radar all along and
      throwing the result away.

    So: send it the canonical way and DO NOT wait for an answer. Whether it worked
    is decided by watching 0x21b go quiet, which is the only honest test available
    when the protocol forbids a reply.
    """
    # GENTLEST CONTROL TYPE FIRST.
    #
    # 0x83 is controlType 3, disableRxAndTx: the radar stops LISTENING as well as
    # transmitting. That is what disable_ecu uses, because on Honda Bosch and
    # Hyundai it is what works -- but it is more than this job needs. We only want
    # 0x21b/0x21c to stop; nothing requires the radar to go deaf.
    #
    # MEASURED 2026-08-11: 28 83 01 silences this radar and the front camera then
    # reports a sensor fault -- which the programming-session route did NOT provoke,
    # and that route leaves the radar able to receive. A camera that talks to the
    # radar and gets nothing back is a plausible read of that difference.
    #
    # 0x81 is controlType 1, enableRxDisableTx, with the same suppressPosRsp bit
    # (0x80) that makes the canonical form work at all. Radar keeps listening, keeps
    # answering diagnostics, and simply stops putting its normal messages on the
    # wire -- exactly what this file's docstring describes as the goal.
    #
    # 0x83 stays as the fallback: it is the proven form, and a silenced radar with
    # one camera complaint still beats no suppression at all.
    # ONE attempt per form, not `retry`. Each miss costs a full _radar_went_quiet
    # window, and at an ignition edge that time is the difference between silencing
    # the radar before the other modules see it and after. If the first send does
    # not land, retrying the identical bytes is unlikely to change the answer --
    # falling through to the next form is both faster and more informative.
    _tries = 1 if _os.environ.get("OP_RADAR_NO_FASTPATH") != "1" else retry
    for label, canonical in (("28 81 01 (enableRxDisableTx)", b'\x28\x81\x01'),
                             ("28 83 01 (disableRxAndTx)", b'\x28\x83\x01')):
      for attempt in range(_tries):
        try:
          # Empty expected-response list, zero timeout: send and move on. A raised
          # exception here is still worth knowing about -- it means the frame never
          # reached the wire, which is different from "sent and ignored".
          IsoTpParallelQuery(can_send, can_recv, bus, [(addr, None)],
                             [canonical], [b'']).get_data(0)
          carlog.warning(f"mazda radar sent canonical {label} (no reply expected)")
          if _radar_went_quiet(quiet_window):
            carlog.warning(f"mazda radar {label} took effect -- 0x21b stopped")
            return True
          carlog.error(f"mazda radar {label} sent but 0x21b still running")
          break        # this form does not work here; try the next one
        except Exception:
          carlog.exception("mazda radar canonical standby exception")
        carlog.error(f"mazda radar canonical standby retry ({attempt + 1})")
    return False

  def _radar_went_quiet(window: float = 1.2) -> bool:
    """Did 0x21b actually stop? The only test available for a suppressed response.

    Nothing of ours is on 0x21b at this point -- long_tx_thread starts later -- so
    any frame here is the radar still driving. Deliberately short: this runs inside
    the ladder, and a rung that has not worked should fall through quickly rather
    than stall startup.

    Counts frames rather than sampling once. A radar goes briefly quiet WHILE it
    handles a UDS request and then resumes, so a single instantaneous look can read
    that pause as success.

    SILENCE ONLY COUNTS IF THE CAR IS AWAKE. An unpowered radar is silent too, so
    with the ignition off this test cannot tell a successful takeover from a car
    that is simply asleep -- and it would answer True to both.

    MEASURED 2026-08-11, first run of this code: it reported "STANDBY via default
    (canonical 28 83 01) -- 0x21b stopped" while the caller, one line later, said
    "INCONCLUSIVE -- the car is asleep (0x202 0 frames over 3.0 s)". The rung was
    scored a success against a radar that had no power. Requiring ENGINE_DATA in
    the SAME window is the same guard the caller already applies, and for the same
    reason.

    TESTER-PRESENT IS PUMPED THROUGHOUT, and without it this check is a trap.
    disable_ecu's own docstring is explicit: "The ECU will stay silent as long as
    openpilot keeps sending Tester Present." CommunicationControl is held by the
    session, and the session lapses in seconds once tester-present stops.

    MEASURED 2026-08-11: after 28 83 01 in the programming session, a 1.2 s window
    with no tester-present saw 0x21b completely silent -- and the caller's own check
    moments later counted 23 frames over 2.5 s with the ignition alive. The radar
    accepted the command, went quiet, then timed the session out and resumed. Read
    without this, that pair of results looks like the radar rejecting standby; it is
    actually standby working and being dropped on the floor.

    So keep the session alive here, and keep the window long enough to outlast the
    lapse that was previously being measured.
    """
    import time as _time
    deadline = _time.monotonic() + window
    seen = 0
    ign = 0
    tp_last = 0.0
    while _time.monotonic() < deadline:
      now = _time.monotonic()
      if now - tp_last >= 0.1:          # 10 Hz, same rate long_tx_thread uses
        tp_last = now
        try:
          can_send([make_tester_present_msg(addr, bus, suppress_response=True)])
        except Exception:
          carlog.exception("mazda radar quiet-check tester-present exception")
      try:
        for packet in (can_recv() or []):
          for msg in packet:
            _a = getattr(msg, "address", None)
            if _a == CRZ_INFO_ADDR:
              seen += 1
            elif _a == _ENGINE_DATA_ADDR:
              ign += 1
      except Exception:
        carlog.exception("mazda radar quiet-check exception")
        return False
      if seen:
        return False        # still transmitting -- no point waiting out the window
      _time.sleep(0.01)
    if not ign:
      carlog.error("mazda radar quiet-check INCONCLUSIVE -- no ENGINE_DATA, car asleep")
      return False
    return True

  def _try_dtc_off() -> None:
    # Advisory only: a NAK means the radar logs whatever it would have logged
    # anyway, which is not a reason to abandon a working standby.
    if not disable_dtc:
      return
    req = bytes([uds.SERVICE_TYPE.CONTROL_DTC_SETTING, uds.DTC_SETTING_TYPE.OFF])
    resp = bytes([uds.SERVICE_TYPE.CONTROL_DTC_SETTING + 0x40, uds.DTC_SETTING_TYPE.OFF])
    try:
      if _uds_request(can_recv, can_send, bus, addr, req, resp, timeout=0.3):
        carlog.warning(f"mazda radar DTC setting OFF on {hex(addr)}")
      else:
        carlog.error("mazda radar DTC setting OFF refused (continuing)")
    except Exception:
      carlog.exception("mazda radar DTC setting exception (continuing)")

  # RUNG 4 EXISTS BECAUSE THE FIRST THREE ASK THE WRONG ECU STATE.
  #
  # MEASURED 2026-08-09 and again 2026-08-11, five attempts each:
  #     0x10 03 extended        -> NO REPLY AT ALL
  #     0x10 04 safety-system   -> refused
  #     bare 0x28 (default)     -> refused
  #     0x10 02 PROGRAMMING     -> accepted instantly, "06 50 02 00 19 01 f4 00"
  #
  # So every rung above tries CommunicationControl in a session this radar will not
  # enter, and the one session it DOES enter was only ever used as the fallback --
  # where 0x28 is never asked for. The programming session was reached hundreds of
  # times today and standby was never once requested inside it.
  #
  # That matters because the two are not the same thing to the rest of the car. A
  # radar held in a programming session looks like an ECU mid-reflash: absent, and
  # other modules fault on it. 0x28 ENABLE_RX_DISABLE_TX leaves it present,
  # answering diagnostics and running its application -- it just stops putting its
  # normal messages on the wire. Same silence on 0x21b/0x21c, very different story
  # for the cluster, which is the entire difference between a dash full of errors
  # and a clean one.
  #
  # Costs nothing to try: if 0x28 is refused here too, we are sitting in the
  # programming session, which is exactly where the fallback would have put us.
  # FCW/AEB/SBS are gone either way -- that is inherent to taking the radar's
  # frames, not to how it was silenced.
  # FAST PATH -- the known-good route, tried first and alone.
  #
  # The ladder below is a SEARCH, and searching costs time this job does not have.
  # MEASURED 2026-08-11 from an ignition edge: 5.55 s from "suppressing NOW" to the
  # radar going quiet, spent almost entirely on rungs whose outcome is already
  # known -- five bare-0x28 retries, an extended session that never replies, a
  # safety-system session that is refused, then five more retries of the
  # response-expecting 28 01 01 that cannot succeed by design.
  #
  # Five and a half seconds is an eternity at wake. The whole point of suppressing
  # at the ignition edge is to silence the radar BEFORE the cluster, camera and PCM
  # enumerate it; broadcasting normally for 5.5 s hands them every chance to do so,
  # which is the likely reason the malfunction still appeared on a run that was
  # otherwise correctly timed.
  #
  # So: programming session, then 28 81 01, and nothing else. Both are measured to
  # work on this radar every time. The full ladder still runs if this misses, so a
  # different radar (or a firmware change) degrades to the old search rather than
  # failing outright. OP_RADAR_NO_FASTPATH=1 forces the search for A/B.
  # POLL UNTIL THE RADAR IS READY, do not ask once and give up.
  #
  # MEASURED 2026-08-11 on an ignition edge, the very first request came back:
  #     iso-tp query bad response: 0x7f 10 22
  # 0x7F = negative response, 0x10 = DiagnosticSessionControl, NRC 0x22 =
  # conditionsNotCorrect. The radar is not ignoring us at wake -- it is explicitly
  # answering "not yet", because it has only just powered up.
  #
  # Asking once and falling through to the full ladder turned that "not yet" into a
  # 5.54 s handshake, because the ladder reaches the programming session only after
  # exhausting every rung that cannot work. Those 5.5 s are exactly the window in
  # which the cluster, camera and PCM enumerate a healthy radar -- which is what the
  # ignition-edge timing exists to prevent.
  #
  # So retry tightly and take the session the instant it is granted. The radar
  # answers 0x22 while booting and accepts within a second or so; polling at 20 Hz
  # catches that transition rather than sleeping through it.
  import os as _os
  if _os.environ.get("OP_RADAR_NO_FASTPATH") != "1":
    _time = __import__("time")
    _t_fast = _time.monotonic()
    _fast_deadline = _t_fast + float(_os.environ.get("OP_RADAR_FASTPATH_S", "6.0"))
    _got = False
    _tries = 0
    while _time.monotonic() < _fast_deadline:
      _tries += 1
      if _try_session(uds.SESSION_TYPE.PROGRAMMING, "programming (fast path)"):
        _got = True
        carlog.warning("mazda radar programming session granted after %.2f s "
                       "(%d attempts) -- radar answers 0x22 conditionsNotCorrect "
                       "until it has booted" % (_time.monotonic() - _t_fast, _tries))
        break
      _time.sleep(0.05)
    if _got:
      # 0.35 s, not the 1.2 s default. The radar runs ~43 Hz, so a window this
      # short still expects ~15 frames if it is alive -- ample to tell running from
      # stopped -- and the check returns the instant it sees one, so a failure
      # costs almost nothing. The caller re-verifies over a full 3 s afterwards
      # anyway; this only has to be good enough to pick the right rung.
      if _try_comm_control_canonical(quiet_window=0.35):
        carlog.warning("mazda radar STANDBY via fast path in %.2f s "
                       "-- tx disabled, ECU still present"
                       % (__import__("time").monotonic() - _t_fast))
        return True
    carlog.error("mazda radar fast path missed -- falling back to the full ladder")

  for session, label in ((None, "default (no session change)"),
                         (uds.SESSION_TYPE.EXTENDED_DIAGNOSTIC, "extended"),
                         (uds.SESSION_TYPE.SAFETY_SYSTEM_DIAGNOSTIC, "safety-system"),
                         (uds.SESSION_TYPE.PROGRAMMING, "programming")):
    if session is not None and not _try_session(session, label):
      continue
    _try_dtc_off()
    if _try_comm_control():
      carlog.warning(f"mazda radar STANDBY via {label} -- tx disabled, ECU still present")
      return True
    # The response-expecting form failed. Before giving up on this session, try the
    # canonical suppressed-response form -- which cannot report success on its own,
    # so ask the bus instead: if 0x21b stops, the radar took it.
    # _try_comm_control_canonical now verifies each form itself (it has to -- it
    # tries two, and only the one that actually silences the radar should win), so
    # no second _radar_went_quiet() here.
    if _try_comm_control_canonical():
      carlog.warning(f"mazda radar STANDBY via {label} (canonical CommunicationControl) "
                     "-- 0x21b stopped, ECU still present")
      return True
    carlog.error(f"mazda radar standby via {label} failed")

  carlog.error("mazda radar standby failed on every route -- falling back to programming session")
  return False


def exit_radar_standby(can_recv, can_send, bus: int = RADAR_BUS, addr: int = RADAR_ADDR) -> bool:
  """Undo enter_radar_standby: transmit back on, DTC storage back on, default session.

  Every step is attempted even if an earlier one fails -- leaving the radar mute
  because the DTC request happened to NAK would be a much worse outcome than a
  noisy log. Re-enabling transmission is the one that matters, so it goes first.
  """
  ok = True

  cc_req = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL,
                  uds.CONTROL_TYPE.ENABLE_RX_ENABLE_TX, _COMM_TYPE_NORMAL])
  cc_resp = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL + 0x40,
                   uds.CONTROL_TYPE.ENABLE_RX_ENABLE_TX])
  try:
    if not _uds_request(can_recv, can_send, bus, addr, cc_req, cc_resp):
      carlog.error("mazda radar tx re-enable refused")
      ok = False
  except Exception:
    carlog.exception("mazda radar tx re-enable exception")
    ok = False

  dtc_req = bytes([uds.SERVICE_TYPE.CONTROL_DTC_SETTING, uds.DTC_SETTING_TYPE.ON])
  dtc_resp = bytes([uds.SERVICE_TYPE.CONTROL_DTC_SETTING + 0x40, uds.DTC_SETTING_TYPE.ON])
  try:
    _uds_request(can_recv, can_send, bus, addr, dtc_req, dtc_resp)
  except Exception:
    carlog.exception("mazda radar DTC setting ON exception")

  # Dropping to the default session also ends standby on its own in most
  # implementations, so this is both the tidy-up and a backstop for the above.
  try:
    request_radar_default_session(can_recv, can_send, bus, addr)
  except Exception:
    carlog.exception("mazda radar default session exception")

  return ok


def request_radar_default_session(can_recv, can_send, bus: int = RADAR_BUS, addr: int = RADAR_ADDR) -> bool:
  request = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.DEFAULT])
  response = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, uds.SESSION_TYPE.DEFAULT])

  try:
    return _uds_request(can_recv, can_send, bus, addr, request, response)
  except Exception:
    carlog.exception("mazda radar default session exception")
    return False
