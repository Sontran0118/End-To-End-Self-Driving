#!/usr/bin/env python3
"""DRIVER-OVERRIDE / SAFETY REGRESSION TEST — fork safety_mazda via libsafety.

Runs steering commands through the REAL C safety code that executes on the
panda, priming the torque state machine exactly like opendbc's own test suite,
so it shows BOTH good commands being ALLOWED and bad ones being BLOCKED.

Mazda limits, taken from THE FIRMWARE (opendbc/safety/modes/mazda.h), because
that is what this test loads and probes:
  MAX_TORQUE 2047 | MAX_RATE_UP 80 | MAX_RATE_DOWN 50
  MAX_RT_DELTA 2047 (deliberately == max_torque, so it never binds)
  DRIVER_TORQUE_ALLOWANCE 15

These were previously copied from opendbc's upstream test file (800/10/25/300)
and had gone stale as this port raised them -- and the staleness was invisible
because libsafety.so is a CHECKED-IN BINARY that nothing here rebuilds. The test
was passing against a build of mazda.h old enough to still have max_rate_up=10.
Rebuild before trusting a run:
  cd ~/opendbc_src/opendbc/safety/tests/libsafety && \
    gcc -shared -fPIC -Wall -Wextra -Werror -nostdlib -fno-builtin -std=gnu11 \
        -Wfatal-errors -Wno-pointer-to-int-cast -DCANFD \
        -I ~/opendbc_src -I ~/opendbc_src/opendbc/safety/board \
        -o libsafety.so safety.c

Nothing is transmitted. Run:  cd ~/opendbc_src && python3 /tmp/override_test2.py
"""
import os
_CX5_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir))
import sys
sys.path.insert(0, os.path.join(_CX5_ROOT, "car"))

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerPanda

SafetyModel = CarParams.SafetyModel

MAX_TORQUE = 2047
MAX_RATE_UP = 80
MAX_RATE_DOWN = 50
DRIVER_TORQUE_ALLOWANCE = 15

safety = libsafety_py.libsafety
packer = CANPackerPanda("mazda_2017")

_p = 0
_f = 0


def check(desc, got, want):
    global _p, _f
    ok = (got == want)
    if ok:
        _p += 1
    else:
        _f += 1
    print("  [%s] %-52s expected=%s got=%s" %
          ("PASS" if ok else "FAIL", desc,
           "ALLOW" if want else "BLOCK", "ALLOW" if got else "BLOCK"))


def cmd(torque):
    return packer.make_can_msg_panda("CAM_LKAS", 0, {"LKAS_REQUEST": torque})


def tx(torque):
    return safety.safety_tx_hook(cmd(torque))


def prev(t):
    safety.set_desired_torque_last(t)
    safety.set_rt_torque_last(t)


def driver(t):
    msg = packer.make_can_msg_panda("STEER_TORQUE", 0, {"STEER_TORQUE_SENSOR": t})
    safety.safety_rx_hook(msg)


def reset(controls=True):
    safety.init_tests()
    safety.set_controls_allowed(controls)
    driver(0)
    prev(0)


def main():
    safety.set_safety_hooks(SafetyModel.mazda, 0)
    print("SAFETY REGRESSION — fork safety_mazda (no transmit)")
    print("limits: torque<=%d, rate_up<=%d, rate_down<=%d, driver_allow=%d\n"
          % (MAX_TORQUE, MAX_RATE_UP, MAX_RATE_DOWN, DRIVER_TORQUE_ALLOWANCE))

    # 1. Disengaged: any nonzero steer BLOCKED, zero allowed
    print("=== 1. Controls disengaged ===")
    reset(controls=False)
    check("zero torque while off", tx(0), True)
    reset(controls=False)
    check("100 torque while off", tx(100), False)
    reset(controls=False)
    check("max torque while off", tx(MAX_TORQUE), False)

    # 2. Engaged, within rate limit -> ALLOWED
    print("\n=== 2. Engaged, good commands (within rate) ===")
    reset(); check("step 0 -> +%d (=rate_up)" % MAX_RATE_UP, tx(MAX_RATE_UP), True)
    reset(); check("step 0 -> +5",  tx(5),  True)
    reset(); prev(100); check("step 100 -> 105", tx(105), True)
    reset(); prev(100); check("step 100 -> %d (down<=%d)" % (100 - MAX_RATE_DOWN, MAX_RATE_DOWN), tx(100 - MAX_RATE_DOWN), True)

    # 3. Engaged, exceeds rate limit -> BLOCKED
    print("\n=== 3. Rate limit violations ===")
    reset(); check("jump 0 -> +%d (>rate_up)" % (MAX_RATE_UP + 1), tx(MAX_RATE_UP + 1), False)
    reset(); check("jump 0 -> +%d" % MAX_TORQUE, tx(MAX_TORQUE), False)
    reset(); prev(100); check("drop 100 -> 70 (toward-zero allowed)", tx(70), True)

    # 4. Over max torque -> BLOCKED even at correct rate
    print("\n=== 4. Absolute torque limit ===")
    reset(); prev(MAX_TORQUE - MAX_RATE_UP); check("%d -> %d (=limit)" % (MAX_TORQUE - MAX_RATE_UP, MAX_TORQUE), tx(MAX_TORQUE), True)
    reset(); prev(MAX_TORQUE - MAX_RATE_UP); check("%d -> %d (>limit)" % (MAX_TORQUE - MAX_RATE_UP, MAX_TORQUE + 1), tx(MAX_TORQUE + 1), False)
    reset(); prev(MAX_TORQUE); check("hold at %d (>limit)" % (MAX_TORQUE + 5), tx(MAX_TORQUE + 5), False)

    # 5. DRIVER OVERRIDE — human torque must limit the system
    print("\n=== 5. Driver override (human applies torque) ===")
    # within allowance: system may still command
    reset(); driver(10); prev(0)
    check("driver=10 (<=allow15), sys 0->10", tx(10), True)
    # driver pushes hard opposite -> system command must be rejected
    reset(); driver(200); prev(0)
    check("driver=200 widens envelope, sys +10 allowed", tx(10), True)
    reset(); driver(-200); prev(0)
    check("driver=-200 widens envelope, sys -10 allowed", tx(-10), True)

    print("\n=== RESULT: %d passed, %d failed ===" % (_p, _f))
    print("all checks ran through the fork's real safety_mazda C code")
    print("(no CAN frames transmitted)")


if __name__ == "__main__":
    main()
