#!/usr/bin/env python3
"""The radar mute must FAIL OPEN when the host loses the bus.

REGRESSION FOR drivelogs/cb3_panda.log, 2026-08-13:

  t=588.5s  handover 2, radar suppressed
  t=676.7s  the USB feed dies -- 21f=0 165=0 09d=0, the whole bus
  t=940.3s  STILL suppressing, tp=11059 tester-present frames sent blind

264 s of suppression no brake, cancel or MAIN-off could clear, because the
release waits on cs_can.acc_active -- a decoded value with no expiry, which
held True once the frames stopped. The car logged fault codes throughout.

The invariant these checks defend: a decoded VALUE never establishes liveness,
only an arrival TIME does.
"""
import os, sys, importlib
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("OP_NO_CAMERA", "1")

ok = fail = 0
def check(name, cond, detail=""):
    global ok, fail
    if cond: ok += 1; print("  PASS  %s" % name)
    else:    fail += 1; print("  FAIL  %s   %s" % (name, detail))

dw = importlib.import_module("dashcam_web")
RxLiveness = dw.RxLiveness

# ---------------------------------------------------------------- the primitive
r = RxLiveness(dead_s=2.0)
check("fails CLOSED before any frame arrives", r.alive(now=0.0) is False,
      "a fresh process must not assert anything about the car")
check("age is infinite before any frame", r.age(now=0.0) == float("inf"))

r.stamp(now=100.0)
check("alive right after a frame", r.alive(now=100.0) is True)
check("alive at 1.9 s", r.alive(now=101.9) is True)
check("DEAD at 2.1 s", r.alive(now=102.1) is False)
check("age reports the real gap", abs(r.age(now=102.1) - 2.1) < 1e-9)

r.stamp(now=103.0)
check("a new frame revives it", r.alive(now=103.0) is True)

check("0 disables the check", RxLiveness(dead_s=0.0).alive(now=1e9) is True)

# 100 Hz PEDALS means a live bus never has a 2 s hole. Walk a realistic feed.
r2 = RxLiveness(dead_s=2.0)
t = 0.0
live_ok = True
for _ in range(2000):          # 20 s at 100 Hz
    t += 0.01
    r2.stamp(now=t)
    if not r2.alive(now=t):
        live_ok = False
check("never fires on a healthy 100 Hz bus", live_ok)

# ------------------------------------------------- the failure, replayed
# The exact shape of the incident: acc_active latched True, frames stopped.
class FrozenCarState:
    """carstate after the feed died: the last decoded values, forever."""
    acc_active = True          # dashcam.py:293 -- a plain assignment, no expiry
    v_ego = 18.99              # 68.4 kph, the value it froze at

cs = FrozenCarState()
r3 = RxLiveness(dead_s=2.0)
r3.stamp(now=676.7)            # last frame ever received

# OLD release condition: `while acc_active:` -- spins forever.
old_would_release = not cs.acc_active
check("OLD condition never releases (this is the bug)", old_would_release is False,
      "if this passes, the test is not reproducing the incident")

# NEW: rx_alive() AND acc_active. 264 s later, at t=940.3.
new_would_release = not (r3.alive(now=940.3) and cs.acc_active)
check("NEW condition releases despite the stale acc_active", new_would_release is True,
      "a stale True is not evidence ACC is on")

# And it releases PROMPTLY, not eventually: within a second of the deadline.
first_release_t = None
t = 676.7
while t < 700.0:
    t += 0.02
    if not (r3.alive(now=t) and cs.acc_active):
        first_release_t = t
        break
check("releases within ~2 s of the feed dying",
      first_release_t is not None and (first_release_t - 676.7) < 2.1,
      "released at +%.2fs" % ((first_release_t - 676.7) if first_release_t else -1))

# Entry must be gated too: a stale acc_active must not START a suppression.
would_handover = r3.alive(now=940.3) and cs.acc_active
check("stale acc_active cannot trigger a new handover", would_handover is False)

print("\n=== RESULT: %d passed, %d failed ===" % (ok, fail))
sys.exit(0 if fail == 0 else 1)
