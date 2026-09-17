#!/usr/bin/env python3
"""The board reset must not be starved by the soft reset.

REGRESSION. Both watchdogs in usb_panda._recv_resync used to stamp one shared
`_last_reset_t`:

    rate collapse ->  fires when collapsed and (now - t) > 10, checked every 5 s
    silicon wedge ->  fires when         wedged and (now - t) > 15, checked after

While a collapse persists the first re-fires every ~15 s and re-stamps t, and it
runs FIRST in the same call -- so the wedge check always saw 0 s elapsed and
could never fire. can_reset_communications cannot clear a wedged USB endpoint;
only a 0xd8 board reset can. So the ineffective recovery permanently suppressed
the effective one.

MEASURED 2026-08-13: wedged 264 s and then 452 s with wedge_recoveries == 0.

This models the two cooldowns directly -- no USB, no car.
"""
import sys

ok = fail = 0
def check(name, cond, detail=""):
    global ok, fail
    if cond: ok += 1; print("  PASS  %s" % name)
    else:    fail += 1; print("  FAIL  %s   %s" % (name, detail))


def simulate(shared, seconds=300.0, step=5.0):
    """Run both watchdogs against a persistent collapse+wedge.

    shared=True reproduces the old behaviour (one timestamp), False the fix.
    Returns (soft_resets, board_resets).
    """
    soft_t = board_t = 0.0
    soft = board = 0
    t = 0.0
    while t < seconds:
        t += step
        # --- rate-collapse watchdog (runs first, as in the real code) ---
        if (t - (soft_t if not shared else max(soft_t, board_t))) > 10.0:
            soft += 1
            soft_t = t
            if shared:
                board_t = t          # the bug: one shared stamp
        # --- silicon-vs-host wedge detector ---
        ref = board_t if not shared else max(soft_t, board_t)
        if (t - ref) > 15.0:
            board += 1
            board_t = t
            if shared:
                soft_t = t
    return soft, board


print("300 s of a persistent wedge:")
old_soft, old_board = simulate(shared=True)
new_soft, new_board = simulate(shared=False)
print("  shared timestamp (old): soft=%d  board=%d" % (old_soft, old_board))
print("  separate stamps  (new): soft=%d  board=%d" % (new_soft, new_board))

check("OLD: board reset never fires (this is the bug)", old_board == 0,
      "got %d -- the test is not reproducing the starvation" % old_board)
check("NEW: board reset fires", new_board > 0, "got %d" % new_board)
check("NEW: recovery is prompt, not eventual", new_board >= 15,
      "only %d resets in 300 s" % new_board)
check("NEW: soft path still runs", new_soft > 0, "got %d" % new_soft)

# The first board reset should land within ~20 s, not minutes.
def first_board_reset(shared):
    soft_t = board_t = 0.0
    t = 0.0
    while t < 600.0:
        t += 5.0
        if (t - (soft_t if not shared else max(soft_t, board_t))) > 10.0:
            soft_t = t
            if shared: board_t = t
        ref = board_t if not shared else max(soft_t, board_t)
        if (t - ref) > 15.0:
            return t
    return None

f_old, f_new = first_board_reset(True), first_board_reset(False)
check("OLD: no board reset within 10 minutes", f_old is None, "fired at %s" % f_old)
check("NEW: board reset within 25 s of the wedge", f_new is not None and f_new <= 25.0,
      "fired at %s" % f_new)

# The attributes must actually exist and be distinct on the real class.
sys.path.insert(0, "/home/tran/op_fork/jetson_port")
import ast
src = open("/home/tran/op_fork/jetson_port/usb_panda.py").read()
# Strip comments first -- the fix's own note mentions the old name.
_code = "\n".join(l.split("#")[0] for l in src.splitlines())
check("usb_panda has no stale _last_reset_t use",
      "self._last_reset_t" not in _code,
      "a stale reference would raise AttributeError at runtime")
check("both cooldowns are initialised",
      "self._last_soft_reset_t = 0.0" in src and "self._last_board_reset_t = 0.0" in src)
ast.parse(src)
check("usb_panda parses", True)

print("\n=== RESULT: %d passed, %d failed ===" % (ok, fail))
sys.exit(0 if fail == 0 else 1)
