"""Exercise dashcam_web's UDS callables against the real IsoTpParallelQuery.

The radar handshake cannot be tested without the car, but the part that is easy
to get wrong -- the can_recv/can_send contract and the sniffer tap feeding them
-- can be. This replays dashcam_web's exact plumbing against a fake radar:

  can_send  -> captures the frame the query transmits
  fake radar -> pushes the matching reply into the sniffer deque
  can_recv  -> drains the deque, same code as dashcam_web

Then asserts enter_radar_programming_session() actually reports success, and
that it reports FAILURE when the reply address is filtered out -- which is what
the panda does when it is still running the 17-ID mazda_filter.h build.
"""
import collections
import sys
import threading
import time

sys.path.insert(0, "/home/tran/opendbc_src")

from opendbc.car.can_definitions import CanData
from opendbc.car.mazda.longitudinal import RADAR_ADDR, enter_radar_programming_session

RESP_ADDR = RADAR_ADDR + 0x8          # 0x76C

# --- the tap, copied from dashcam_web -------------------------------------
uds_sniff = {"on": False, "addrs": frozenset(), "q": collections.deque(maxlen=512)}
panda_lock = threading.Lock()


def _uds_can_recv(wait_for_one: bool = False):
    deadline = time.time() + 0.05
    while True:
        msgs = []
        while uds_sniff["q"]:
            _a, _d, _b = uds_sniff["q"].popleft()
            msgs.append(CanData(_a, _d, _b))
        if msgs:
            return [msgs]
        if not wait_for_one or time.time() > deadline:
            return []
        time.sleep(0.002)


def run(host_visible_addrs, label):
    """host_visible_addrs models the panda's MAZDA_HOST_IDS gate."""
    uds_sniff["addrs"] = frozenset({RESP_ADDR})
    uds_sniff["q"].clear()
    uds_sniff["on"] = True
    sent = []

    def _uds_can_send(msgs):
        with panda_lock:
            for _m in msgs:
                sent.append((_m.address, bytes(_m.dat), _m.src))
                # Fake radar: a single-frame ISO-TP positive response to
                # DIAGNOSTIC_SESSION_CONTROL / PROGRAMMING (0x10 0x02 -> 0x50 0x02).
                if _m.address == RADAR_ADDR and _m.dat[1] == 0x10:
                    reply = bytes([0x02, 0x50, 0x02, 0, 0, 0, 0, 0])
                    # can_thread's tap only sees what the panda lets through.
                    if RESP_ADDR in host_visible_addrs:
                        uds_sniff["q"].append((RESP_ADDR, reply, _m.src))

    t0 = time.time()
    ok = enter_radar_programming_session(_uds_can_recv, _uds_can_send, bus=0)
    dt = time.time() - t0
    uds_sniff["on"] = False
    print(f"  {label:<34} -> {'SUCCESS' if ok else 'FAILURE'}  "
          f"({len(sent)} tx, {dt:.2f}s)")
    return ok, dt


print("radar UDS handshake, dashcam_web plumbing:")
ok_19, _ = run({RESP_ADDR}, "19-ID filter (0x76C visible)")
ok_17, dt_17 = run(set(), "17-ID filter (0x76C blocked)")

fails = []
if not ok_19:
    fails.append("handshake failed even with the response visible -- plumbing is wrong")
if ok_17:
    fails.append("handshake reported success with no reply -- false positive")
if dt_17 > 3.0:
    fails.append(f"failure path took {dt_17:.1f}s -- would stall arming")

print()
if fails:
    for f in fails:
        print("FAIL:", f)
    raise SystemExit(1)
print("PASS -- succeeds when the radar answers, fails fast when it cannot")
raise SystemExit(0)
