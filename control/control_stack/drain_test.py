"""Does _recv_resync empty the device queue, and does it stay bounded?

Fakes the libusb handle with the firmware's actual behaviour: a bulk IN of
`length` bytes returns min(length, pending) and the device sends a ZLP as soon
as the queue is empty, so a short return means drained.
"""
import os
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # control/
import usb_panda

CHUNK = 16384

class FakeHandle:
    def __init__(self, pending): self.pending = pending; self.reads = 0
    def bulkRead(self, ep, length, *a, **k):
        self.reads += 1
        n = min(length, len(self.pending))
        out, self.pending = self.pending[:n], self.pending[n:]
        return out

class FakeP:
    def __init__(self, h): self._handle = h

def mkpanda(pending):
    o = usb_panda.UsbPanda.__new__(usb_panda.UsbPanda)
    import time as _t
    o.echo_count = o.rejected_count = o.desync_count = 0
    o._LOCK_WINDOW_S = 1.0; o._LOCK_MAX_IDS = 80
    o._lock_addrs = set(); o._lock_frames = 0; o._lock_bad = 0
    o._lock_t0 = _t.monotonic(); o.resync_bytes = 0
    o._SYNC = 0xAA; o._marked = False
    o._MAX_DRAIN_READS = 8; o.drain_reads = 0; o.drain_capped = 0
    o._buf = b''; o._last_frame_t = 0.0; o.stall_recoveries = 0
    o._win_t0 = 0.0; o._win_n = 0; o._peak_rate = 0.0
    o._last_reset_t = 0.0; o.rate_recoveries = 0
    o._hw_check_t = 0.0; o._hw_last = None; o._hw_n = 0
    o.wedge_recoveries = 0; o.last_wedge = ""; o._frozen_windows = 0
    o._bus_err = {}
    o.max_gap_ms = o.max_gap_ever = 0.0; o.over_budget = 0; o._last_call_t = 0.0
    o.p = FakeP(FakeHandle(pending))
    return o

def frame(addr, bus=0, marked=True):
    # CANPacket_t: 12-bit reserved/bus/rej/ret in byte0, addr<<3 in the word,
    # then a checksum byte. Build via the module's own packer if present, else
    # a plausible 13-byte packet -- content does not matter for the drain test,
    # only length and the short-read boundary.
    import struct
    word = (addr << 3)
    b = bytearray(struct.pack('<I', (8 << 4) | 0) )  # placeholder header
    return b''

ok = fail = 0
def check(name, cond, detail=""):
    global ok, fail
    if cond: ok += 1; print("  PASS  %s" % name)
    else:    fail += 1; print("  FAIL  %s  %s" % (name, detail))

print("drain loop")

# 1. exactly one chunk pending -> one read, short, done
o = mkpanda(b'\x00' * 5000)
o._recv_resync()
check("short first read stops after 1 transfer", o.p._handle.reads == 1, "reads=%d" % o.p._handle.reads)
check("device queue empty", len(o.p._handle.pending) == 0)

# 2. 3.5 chunks pending -> 4 reads, queue emptied in ONE call
o = mkpanda(b'\x00' * (CHUNK * 3 + 500))
o._recv_resync()
check("multi-chunk drained in one call", len(o.p._handle.pending) == 0,
      "left=%d" % len(o.p._handle.pending))
check("issued 4 transfers", o.p._handle.reads == 4, "reads=%d" % o.p._handle.reads)
check("drain_reads counted", o.drain_reads == 4, "%d" % o.drain_reads)
check("not flagged capped", o.drain_capped == 0)

# 3. more than the cap -> bounded, and it says so
o = mkpanda(b'\x00' * (CHUNK * 20))
o._recv_resync()
check("bounded at the cap", o.p._handle.reads == 8, "reads=%d" % o.p._handle.reads)
check("cap reported", o.drain_capped == 1, "%d" % o.drain_capped)
check("remainder left for next call", len(o.p._handle.pending) == CHUNK * 12)

# 4. empty queue (ZLP) -> one read, no stall, no crash
o = mkpanda(b'')
o._recv_resync()
check("empty queue costs one transfer", o.p._handle.reads == 1, "reads=%d" % o.p._handle.reads)

# 5. exact multiple of CHUNK: the boundary case that could have spun
o = mkpanda(b'\x00' * CHUNK)
o._recv_resync()
check("exact-chunk queue takes 2 reads then stops", o.p._handle.reads == 2,
      "reads=%d" % o.p._handle.reads)

# 6. THE REGRESSION: sustained worst-case gaps must not accumulate a backlog.
BYTES_PER_GAP = int(2870 * 14 * 0.43)     # 2870 fps, 14 B/frame, 430 ms gap
o = mkpanda(b'')
for _ in range(40):
    o.p._handle.pending += b'\x00' * BYTES_PER_GAP
    o._recv_resync()
left_frames = len(o.p._handle.pending) // 14
check("no backlog after 40 worst-case gaps", left_frames == 0,
      "%d frames still queued" % left_frames)
check("stayed under the 2048-frame queue depth", left_frames < 2048)

print("\n%d/%d" % (ok, ok + fail))
sys.exit(0 if fail == 0 else 1)
