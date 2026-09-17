#!/usr/bin/env python3
"""BUTTON-ENGAGEMENT CONSISTENCY TEST — host shim vs opendbc vs panda firmware.

Under alpha long (pcmCruise = False) three separate pieces of code independently
decide "the driver asked to engage" from the same 0x09d CRZ_BTNS frame:

  1. dashcam_web.py's carstate_thread  -> carState.buttonEnable   (the host shim)
  2. MazdaCarState.update_button_enable -> the same field upstream
  3. mazda.h's CRZ_BTNS branch          -> controls_allowed        (the firmware)

They MUST agree. If the host arms and the panda does not, openpilot believes it
is engaged while every CRZ_CTRL frame is silently dropped -- which on this car
is indistinguishable from the failure that took two evenings to diagnose. If the
panda arms and the host does not, the hardware gate is open with nothing driving
it.

The rule for this car is RESUME on PRESS, SET on RELEASE. It is asymmetric, it
is not what hyundai_common does (falling edge of both), and that is exactly why
it is worth pinning down.

Transmits nothing. Run:
  PYTHONPATH=/home/tran/opendbc_src python3 button_engage_test.py
"""
import sys

sys.path.insert(0, "/home/tran/opendbc_src")

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerPanda

_p = 0
_f = 0


def check(desc, got, want):
    global _p, _f
    ok = (got == want)
    if ok:
        _p += 1
    else:
        _f += 1
    print("  [%s] %-56s want=%-5s got=%s" % ("PASS" if ok else "FAIL", desc, want, got))


# --- 1. the host shim's rule, transcribed from dashcam_web.py carstate_thread --
def host_button_enable(prev, now):
    events = [(k, now[k]) for k in now if now[k] != prev[k]]
    return any((k == "res" and pressed) or (k == "set_m" and not pressed)
               for k, pressed in events)


# --- 2. opendbc's rule, driven through the real MazdaCarState ------------------
def opendbc_button_enable(prev, now):
    from opendbc.car import structs
    from opendbc.car.mazda.carstate import CarState as MazdaCarState

    ButtonType = structs.CarState.ButtonEvent.Type
    CP = structs.CarParams()
    CP.pcmCruise = False                      # alpha long
    cs = MazdaCarState.__new__(MazdaCarState)
    cs.CP = CP

    events = []
    for k, t in (("res", ButtonType.accelCruise), ("set_m", ButtonType.decelCruise)):
        if now[k] != prev[k]:
            be = structs.CarState.ButtonEvent()
            be.type = t
            be.pressed = now[k]
            events.append(be)
    return bool(MazdaCarState.update_button_enable(cs, events))


# --- 3. the firmware, through the real safety code ----------------------------
safety = libsafety_py.libsafety
packer = CANPackerPanda("mazda_2017")


def _btn_msg(res=False, set_m=False, cancel=False):
    return packer.make_can_msg_panda("CRZ_BTNS", 0, {
        "CAN_OFF": cancel, "CAN_OFF_INV": (cancel + 1) % 2,
        "RES": res, "RES_INV": (res + 1) % 2,
        "SET_M": set_m, "SET_M_INV": (set_m + 1) % 2,
    })


def _main_on():
    safety.safety_rx_hook(packer.make_can_msg_panda("PEDALS", 0, {"ACC_OFF": 1, "BRAKE_ON": 0}))


def panda_button_enable(prev, now):
    safety.set_safety_hooks(CarParams.SafetyModel.mazda, 1)   # param 1 = alpha long
    safety.init_tests()
    _main_on()
    # Establish `prev` as the BASELINE first, then clear. Order matters: the
    # firmware's button history starts at all-false, so replaying a `prev` that
    # already has RESUME held is itself a rising edge and legitimately arms. If
    # controls_allowed is cleared before that frame instead of after, every case
    # with RESUME held in `prev` reads as a spurious arm -- which is a bug in the
    # measurement, not in mazda.h.
    safety.safety_rx_hook(_btn_msg(res=prev["res"], set_m=prev["set_m"]))
    safety.set_controls_allowed(False)
    safety.safety_rx_hook(_btn_msg(res=now["res"], set_m=now["set_m"]))
    return bool(safety.get_controls_allowed())


def assert_firmware_variant():
    """Refuse to run against a libsafety that is not what the panda is running.

    This test was written to catch a host/firmware disagreement, and then missed
    the real one because libsafety.so had been built WITHOUT -DMAZDA_MADS while
    the firmware is built WITH it (panda_f446/SConscript). It passed 12/12
    against a variant that does not exist on the car.

    Discriminator: with mazda_longitudinal OFF, the CRZ_CTRL branch arms on
    CRZ_AVAILABLE (byte 2 bit 1) in a MADS build and on CRZ_ACTIVE (byte 0
    bit 3) in a stock one. Probe CRZ_AVAILABLE alone -- it arms only under MADS.
    """
    safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)   # param 0 = not long
    safety.init_tests()
    safety.set_controls_allowed(False)
    safety.safety_rx_hook(packer.make_can_msg_panda(
        "CRZ_CTRL", 0, {"CRZ_AVAILABLE": 1, "CRZ_ACTIVE": 0}))
    is_mads = bool(safety.get_controls_allowed())
    if not is_mads:
        print("REFUSING TO RUN: libsafety.so is a STOCK build, but the panda runs\n"
              "a MAZDA_MADS build. Testing this variant proves nothing about the car.\n"
              "Rebuild with the firmware's own defines:\n"
              "  cd ~/opendbc_src/opendbc/safety/tests/libsafety && \\\n"
              "    gcc -shared -fPIC -Wall -Wextra -Werror -nostdlib -fno-builtin \\\n"
              "        -std=gnu11 -Wfatal-errors -Wno-pointer-to-int-cast -DCANFD \\\n"
              "        -DPANDA_NUCLEO -DMAZDA_FILTER -DMAZDA_MADS -DMAZDA_MADS_BRAKE \\\n"
              "        -I ~/opendbc_src -I ~/opendbc_src/opendbc/safety/board \\\n"
              "        -o libsafety.so safety.c")
        return False
    print("libsafety variant: MAZDA_MADS build (matches panda_f446/SConscript)\n")
    return True


def main():
    print("BUTTON ENGAGEMENT — host shim vs opendbc vs panda (no transmit)\n")
    if not assert_firmware_variant():
        return 2

    # every transition of the two engagement buttons, plus the idle case
    states = [{"res": r, "set_m": s} for r in (False, True) for s in (False, True)]
    cases = []
    for prev in states:
        for now in states:
            cases.append((prev, now))

    def label(prev, now):
        def f(d):
            return "".join(k[0].upper() if v else "-" for k, v in sorted(d.items()))
        return "%s -> %s" % (f(prev), f(now))

    print("=== 1. the three implementations agree on every transition ===")
    disagreements = 0
    for prev, now in cases:
        h = host_button_enable(prev, now)
        o = opendbc_button_enable(prev, now)
        p = panda_button_enable(prev, now)
        if not (h == o == p):
            disagreements += 1
            print("  [FAIL] %-18s host=%-5s opendbc=%-5s panda=%s"
                  % (label(prev, now), h, o, p))
    check("all %d transitions agree across host/opendbc/panda" % len(cases),
          disagreements, 0)

    print("\n=== 2. the rule is the one this car actually needs ===")
    off = {"res": False, "set_m": False}
    res_held = {"res": True, "set_m": False}
    set_held = {"res": False, "set_m": True}

    check("RESUME press engages", host_button_enable(off, res_held), True)
    check("RESUME release does NOT re-engage", host_button_enable(res_held, off), False)
    check("SET press alone does NOT engage", host_button_enable(off, set_held), False)
    check("SET release engages", host_button_enable(set_held, off), True)
    check("no change, no engage", host_button_enable(off, off), False)

    print("\n=== 3. and the panda agrees on those same four ===")
    check("RESUME press arms the panda", panda_button_enable(off, res_held), True)
    check("RESUME release does not", panda_button_enable(res_held, off), False)
    check("SET press alone does not", panda_button_enable(off, set_held), False)
    check("SET release arms the panda", panda_button_enable(set_held, off), True)

    print("\n=== 4. MRCC MAIN alone must NOT arm ===")
    # REGRESSION GUARD. mazda.h used to carry, under #ifdef MAZDA_MADS,
    #     controls_allowed = acc_armed && !brake;
    # as a LEVEL assignment on every PEDALS frame. Under alpha long that is the
    # authority flag for throttle and brake, granted by the MAIN switch alone --
    # and because it ran at 50 Hz it also overwrote whatever the buttons had just
    # decided, making the entire CRZ_BTNS branch inert.
    #
    # It reached the car: MEASURED 2026-08-12 on the first flashed build,
    # ctrl_allowed was 1 with btn +0/-0/R0/O0, before any button was pressed.
    # The bench missed it because libsafety was built WITHOUT -DMAZDA_MADS while
    # the firmware is built WITH it (panda_f446/SConscript). Build this test's
    # libsafety with the firmware's defines or it proves nothing about the car.
    safety.set_safety_hooks(CarParams.SafetyModel.mazda, 1)
    safety.init_tests()
    safety.set_controls_allowed(False)
    for _ in range(50):          # a full second of PEDALS at 50 Hz, MAIN on
        _main_on()
    check("MAIN on, no button, 50 frames -> still disarmed",
          bool(safety.get_controls_allowed()), False)

    # and MAIN on must not RE-arm after a cancel either
    safety.init_tests()
    _main_on()
    safety.set_controls_allowed(False)
    safety.safety_rx_hook(_btn_msg(set_m=True))
    safety.safety_rx_hook(_btn_msg())            # SET release -> armed
    armed = bool(safety.get_controls_allowed())
    safety.safety_rx_hook(_btn_msg(cancel=True))  # cancel -> disarmed
    for _ in range(50):
        _main_on()
    check("armed via SET, then cancel, MAIN still on -> stays disarmed",
          (armed, bool(safety.get_controls_allowed())), (True, False))

    print("\n=== 5. CANCEL always wins ===")
    safety.set_safety_hooks(CarParams.SafetyModel.mazda, 1)
    safety.init_tests()
    _main_on()
    safety.set_controls_allowed(True)
    safety.safety_rx_hook(_btn_msg(cancel=True))
    check("cancel clears an armed panda", bool(safety.get_controls_allowed()), False)

    safety.init_tests()
    _main_on()
    safety.set_controls_allowed(False)
    safety.safety_rx_hook(_btn_msg(set_m=True))
    safety.safety_rx_hook(_btn_msg(cancel=True))   # SET released AND cancel, one frame
    check("cancel beats a simultaneous SET release", bool(safety.get_controls_allowed()), False)

    print("\n=== RESULT: %d passed, %d failed ===" % (_p, _f))
    return 1 if _f else 0


if __name__ == "__main__":
    sys.exit(main())
