"""USB transport for the panda, replacing the serial one.

WHY THIS EXISTS. This port began on a Nucleo-F446RE whose only link to the Jetson
was a UART over the ST-Link VCP, so `SerialPanda` in bench_can_loopback.py
reimplemented panda's four wire operations over a framed serial protocol. The
board is now an STM32F407 (FK407M1) talking native USB, and there is no serial
device at all:

    /dev/serial/by-id/  does not exist
    /dev/ttyACM*        does not exist

find_panda_port() falls through all three by-id globs, returns /dev/ttyACM0, and
serial.Serial() raises on a node that isn't there -- so the whole stack died at
startup regardless of any flag.

This exposes the SAME four methods SerialPanda does, so dashcam_web only has to
change which object it constructs.
"""
import sys
import importlib.util

import os as _os
# Firmware package lives in this monorepo at <root>/firmware. Derived from
# __file__ so the tree can be cloned anywhere; OP_PANDA_PKG overrides it.
_PANDA_PKG = _os.environ.get(
    "OP_PANDA_PKG",
    _os.path.join(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))), "firmware"))


def _load_panda():
    """Import the local panda firmware repo UNDER THE NAME `panda`.

    Two traps here. First, <repo>/firmware/__init__.py line 10 does
    `from panda import Panda` -- an ABSOLUTE import -- so the package only loads
    under that name; registering it in sys.modules BEFORE exec means the
    partially-initialised module is already bound by the time that line runs
    (line 5 has bound Panda onto it).

    Second, op_fork/panda is ALSO a package named `panda` and is on PYTHONPATH.
    It is the upstream H7-only copy: its constants.py has no F4 config, and it
    raises FileNotFoundError on opendbc/safety/can.h at import. Loading by
    explicit path shadows it rather than depending on sys.path order.
    """
    if "panda" in sys.modules and hasattr(sys.modules["panda"], "Panda"):
        return sys.modules["panda"].Panda
    spec = importlib.util.spec_from_file_location(
        "panda", _PANDA_PKG + "/__init__.py",
        submodule_search_locations=[_PANDA_PKG])
    mod = importlib.util.module_from_spec(spec)
    sys.modules["panda"] = mod
    spec.loader.exec_module(mod)
    return mod.Panda


class UsbPanda:
    """SerialPanda's interface, backed by libusb.

    NB: needs /etc/udev/rules.d/99-comma-panda.rules for unprivileged access, or
    every call raises LIBUSB_ERROR_ACCESS. The rule must cover all three product
    IDs -- ddcc (app), ddee (bootstub), 0483:df11 (DFU) -- because the panda
    changes identity across a flash.
    """

    def __init__(self, serial=None):
        Panda = _load_panda()
        self._Panda = Panda
        self.p = Panda(serial)
        # TX echoes and safety-rejected frames, counted rather than discarded
        # silently. rejected > 0 means the panda's safety hook refused one of OUR
        # frames -- the exact "the car ignores us" failure mode that looks like a
        # wiring problem. Nothing surfaced this before.
        self.echo_count = 0
        self.rejected_count = 0
        # Stream realignments. Non-zero is not fatal, but a climbing count means
        # the host is not draining fast enough to keep up with the bus.
        self.desync_count = 0
        # Wrong-lock detector state -- see the long note in _recv_resync.
        # A real bus shows a small stable ID set (23 on bus 0 of this car); a
        # wrong byte alignment shows essentially random addresses. 80 is far
        # above any plausible real bus here and far below the hundreds a wrong
        # lock produces, so it discriminates without tuning.
        import time as _t_init
        self._LOCK_WINDOW_S = 1.0
        self._LOCK_MAX_IDS = int(__import__("os").environ.get("OP_LOCK_MAX_IDS", 80))
        self._lock_addrs = set()
        self._lock_frames = 0
        self._lock_bad = 0
        self._lock_t0 = _t_init.monotonic()
        self.resync_bytes = 0     # bytes dropped re-locking onto a frame boundary
        # Per-packet sync marker emitted by the marked firmware. _marked latches
        # True the first time a marked packet parses cleanly, after which the
        # parser REQUIRES the marker -- see the note in _recv_resync.
        self._SYNC = 0xAA
        self._marked = False
        # Drain loop bounds -- see the note at the bulkRead in _recv_resync.
        # 8 x 16 KiB = 128 KiB per call, ~10000 packets, far more than the
        # firmware's 2048-frame queue can ever hold, so the cap is a runaway
        # guard rather than a limit reached in normal operation.
        self._MAX_DRAIN_READS = int(__import__("os").environ.get("OP_MAX_DRAIN_READS", 8))
        self.drain_reads = 0      # total bulk transfers issued
        self.drain_capped = 0     # calls that hit the cap with data still waiting
        self._buf = b''           # our own tail, not the lib's overflow buffer
        self._last_frame_t = 0.0  # when a genuine frame last arrived
        self.stall_recoveries = 0
        # Rate-collapse watchdog state. See _recv_resync.
        self._win_t0 = 0.0
        self._win_n = 0
        self._peak_rate = 0.0
        # Two INDEPENDENT cooldowns. can_reset_communications (soft) cannot
        # clear a wedged USB endpoint; only a 0xd8 board reset can. Sharing one
        # timestamp let the soft path, which retries every ~15 s during any
        # collapse, permanently suppress the board reset.
        self._last_soft_reset_t = 0.0
        self._last_board_reset_t = 0.0
        self.rate_recoveries = 0
        # Silicon-vs-host wedge detection. See the long note in _recv_resync: the
        # hardware counter is the only unambiguous witness that frames exist but are
        # not crossing the USB link.
        self._hw_check_t = 0.0
        self._hw_last = None
        self._hw_n = 0
        self.wedge_recoveries = 0
        self.last_wedge = ""
        # Consecutive 5 s windows with zero silicon rx while the ignition is on.
        # See the frozen-core branch in _recv_resync.
        self._frozen_windows = 0
        self._bus_err = {}
        # DRAIN-GAP WATCHDOG.
        #
        # can_rx_q holds 2048 frames and bus 0 carries ~2870/s, so the host has
        # 0.71 s between drains before the firmware starts dropping. Every failure
        # this stack has had -- the 450 s freeze, the 725,938-overflow freeze, and
        # the one at t=412 s -- is downstream of missing that deadline. Yet the gap
        # itself was never recorded, so each time the CAUSE had to be inferred from
        # the damage (rx_ovf, resync, a frozen carState) and twice the inference was
        # wrong: first "the radar IDs exceed the link budget" (they do not -- 39 KB/s
        # of ~1000 KB/s), then "the model role is stealing the panda's cores" (it was
        # already pinned off them).
        #
        # These three make the deadline directly observable. If max_gap_ms stays
        # under ~700 the host is keeping up and any overflow came from somewhere
        # else; if it spikes past it, this names the stall instead of guessing at it.
        self.max_gap_ms = 0.0     # worst drain gap since the last status read
        self.max_gap_ever = 0.0   # worst for the whole session
        self.over_budget = 0      # drains that missed the 0.71 s queue deadline
        self._last_call_t = 0.0

    def ep1_guard_counts(self):
        """EP1 IN guard counters from the panda (control request 0xda).

        Returns (busy_skips, nospace_skips), or None on firmware without the
        guard. busy_skips > 0 is direct evidence of the race that used to
        corrupt the bulk endpoint: an IN token arriving while a transfer was
        still open, which the old code answered by rewriting DIEPTSIZ on top of
        the transfer in flight.
        """
        import struct as _s
        try:
            b = bytes(self.control_read(0xda, 0, 0, 8))
        except Exception:
            return None
        if len(b) < 8:
            return None
        return _s.unpack('<II', b[:8])

    # struct health_t, board/health.h. Only the two ignition bytes are needed here;
    # the full layout lives in dashcam_web.HEALTH_FMT.
    _HEALTH_FMT = '<IIIIIIIIBBBBBHBBBHfBBHBHHB'

    def _ignition_on(self) -> bool:
        """True if the car is awake, per the panda's own ignition detect.

        Used to tell a WEDGED core from a PARKED car: both look like zero traffic,
        and resetting the board every 10 s in a car that is simply switched off
        would be a reset loop rather than a recovery. Fails CLOSED -- if the health
        read does not work we report False, so an unreadable board is never a
        reason to reset it.
        """
        import struct as _s
        try:
            hh = self.control_read(0xd2, 0, 0, 64)
            v = _s.unpack(self._HEALTH_FMT, bytes(hh)[:_s.calcsize(self._HEALTH_FMT)])
        except Exception:
            return False
        # [8] ignition_line, [9] ignition_can. Either is enough.
        return bool(v[8] or v[9])

    def bus_error_state(self, bus: int = 0) -> dict:
        """bus_off / error counters / rx delta, for the CAR line.

        A frozen core reports all of these as healthy -- that is the whole point of
        the note in _recv_resync -- so this is not a wedge detector. It is here so
        the OTHER failure modes (a genuinely error-passive or bus-off core, which
        DO show up here) stop being invisible, and so total_rx_cnt is on the line
        where a frozen core can be seen by eye.
        """
        try:
            h = self.p.can_health(bus)
        except Exception:
            return {}
        return {"bus_off": int(h.get("bus_off", 0)),
                "tx_err": int(h.get("transmit_error_cnt", 0)),
                "rx_err": int(h.get("receive_error_cnt", 0)),
                "total_rx": int(h.get("total_rx_cnt", 0)),
                "total_fwd": int(h.get("total_fwd_cnt", 0))}

    def _board_reset(self):
        """0xd8 and reconnect. The only thing measured to clear a wedged link.

        MEASURED 2026-08-11: with the link wedged at 4 frames/s against 2325/s of
        silicon traffic, can_reset_communications() left it wedged; a board reset
        restored 1425 frames/s immediately (the remainder is the host ID filter,
        not loss).

        The safety mode is deliberately NOT restored here. A reset drops the board
        to SAFETY_SILENT, and re-arming Mazda safety is the caller's decision, not
        a side effect of a recovery path -- the alternative is torque authority
        quietly reappearing after a fault the caller never saw.
        """
        import time as _t
        self._buf = b''
        try:
            self.control_write(0xd8, 0, 0)
        except Exception:
            pass                      # the reset tears the link down mid-transfer
        try:
            self.p.close()
        except Exception:
            pass
        _t.sleep(4.0)
        try:
            self.p = self._Panda()
        except Exception:
            # Leave self.p as it was; the next call raises and the caller decides.
            # Better a loud failure than a half-open handle that silently reads 0.
            pass
        self._hw_last = None
        self._hw_n = 0

    def control_write(self, req, p1, p2):
        return self.p._handle.controlWrite(self._Panda.REQUEST_OUT, req, p1, p2, b'')

    def control_read(self, req, p1, p2, length):
        return self.p._handle.controlRead(self._Panda.REQUEST_IN, req, p1, p2, length)

    def can_send(self, addr, dat, bus):
        self.p.can_send(addr, dat, bus)

    # Wire format, from pack/unpack_can_buffer: 6-byte header then data.
    #   header[0] = dlc<<4 | bus<<1 | fd
    #   header[1..4] = little-endian word: addr<<3 | ext<<2 | returned<<1 | rejected
    #   header[5] = checksum; header+data XOR to 0
    _HEAD = 6
    _DLC_TO_LEN = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)

    def _recv_resync(self):
        """can_recv with byte-level resync instead of an assert.

        The stock unpack_can_buffer() asserts on the first bad checksum and
        stops, leaving the offending bytes in the overflow buffer so every later
        call throws too -- one glitch wedges reception for good.

        Recovering by discarding the whole buffer works but is expensive: on this
        bus it fired ~1.6x/s, and each one threw away a full 16 KiB batch of
        perfectly good frames. So do what SerialPanda._recv already did on the
        serial link -- validate each candidate header and slide ONE byte on
        failure. That re-locks onto the next real frame boundary and loses only
        the corrupted packet.

        Validation is checksum-first-and-last: the range checks are cheap but a
        random byte pattern passes them often, and only the packet's own XOR
        checksum reliably separates a real frame from one the slide invented.
        """
        # HARD CAP on the carry-over tail. The slide-one-byte resync consumes at
        # most a byte per iteration, but every call APPENDS another 16 KiB. If a
        # stretch of bytes never yields a valid checksum -- a header whose dlc
        # implies a length longer than what has arrived looks "plausible" and
        # breaks out to wait for more -- the tail can grow faster than it is
        # consumed and reception stalls SILENTLY: no exception, no BAD RECV, just
        # an empty list forever while cs_can freezes on stale values.
        #
        # OBSERVED: v, rpm and steering angle stuck at identical values for 175 s
        # with cam_age climbing and zero errors logged, so openpilot could not
        # engage and nothing said why. Bound it: past the cap, throw the tail away
        # and resynchronise from the next transfer. Losing a few frames is
        # recoverable; losing reception is not.
        if len(self._buf) > 65536:
            self._buf = b''
            self.desync_count += 1
        # DRAIN SIZE. The firmware's can_rx_q holds 2048 frames; once it is full
        # it drops on push (rx_buffer_overflow) and the USB stream degrades into
        # partial packets that cost resync bytes, which slows the drain further --
        # a spiral that ends in a silent freeze.
        #
        # MEASURED 2026-08-09: rx_buffer_overflow 446,577 and resync_bytes 7,217
        # over one 990 s drive, ending with carState frozen (v pinned at 41.0 kph,
        # steering angle decoding as -1369.6 deg from a torn frame) while CAN1's
        # hardware total_rx_cnt kept climbing past 2.8M. The silicon never missed a
        # frame; only the host feed starved.
        #
        # DO NOT raise this to "drain the whole queue in one call". Tried 49152 on
        # the reasoning that 16 KiB (~1170 frames) sits under the 2048-frame queue
        # depth: throughput COLLAPSED to 334 frames/s on a bus carrying ~1400, and
        # rx_buffer_overflow climbed 2,377 -> 29,723 in minutes. A bulk IN transfer
        # ends on a short packet or not at all, so asking for 48 KiB makes libusb
        # sit waiting for packets the firmware has no reason to send yet, and the
        # request length becomes latency. 16 KiB returns promptly and keeps up.
        #
        # The drain loop below is NOT that mistake repeated. It keeps every
        # request at 16 KiB and instead issues MORE of them, and each one still
        # ends the moment the firmware has nothing left (the EP1 handler sends a
        # ZLP -- see the note in board/drivers/usb.h), so a follow-up read on an
        # empty queue costs one round trip, not the 15 s libusb timeout.
        # Timed BEFORE the transfer: the question is how long the firmware's queue
        # was left unattended, which is the interval between successive drains, not
        # the duration of one.
        import time as _t0
        _call_t = _t0.monotonic()
        if self._last_call_t:
            _gap_ms = (_call_t - self._last_call_t) * 1000.0
            if _gap_ms > self.max_gap_ms:
                self.max_gap_ms = _gap_ms
            if _gap_ms > self.max_gap_ever:
                self.max_gap_ever = _gap_ms
            # 2048 frames / ~2870 per second. This figure only became MEANINGFUL
            # with the drain loop below: it assumes a call empties the queue, and
            # while the drain was one fixed 16 KiB read a call could only ever
            # retrieve ~1260 packets (~439 ms of traffic), so the real deadline
            # was shorter than this and the check passed while frames were lost.
            if _gap_ms > 710.0:
                self.over_budget += 1
        self._last_call_t = _call_t

        # DRAIN UNTIL THE QUEUE IS EMPTY, not one fixed read per call.
        #
        # This used to be a single 16 KiB bulkRead. 16384 bytes is only ~1260
        # packets at ~13 bytes each, which is ~439 ms of traffic on a bus running
        # 2870 frames/s -- and the measured worst drain gap is 428 ms. So ONE read
        # barely covered ONE gap, and any time the firmware queue held more than
        # 16 KiB the excess was left behind to accumulate until can_rx_q (2048
        # frames) overflowed and the firmware dropped it.
        #
        # MEASURED 2026-08-13: over_budget stayed at 0 -- the host was meeting the
        # 710 ms deadline the code checks -- while rx_ovf climbed 96,870 ->
        # 269,432 in a single run. The deadline was computed from queue depth and
        # never accounted for how much a single transfer can actually retrieve,
        # so it reported success while the queue overflowed continuously.
        #
        # A short read (fewer bytes than asked for) means the device had nothing
        # more to give, which is the only reliable "queue empty" signal here.
        #
        # BOUNDED: at most _MAX_DRAIN_READS transfers per call, so a saturated bus
        # cannot turn one can_recv() into an unbounded stall. Hitting the cap is
        # itself a signal the host is behind, so it is counted rather than hidden.
        _chunks = []
        for _k in range(self._MAX_DRAIN_READS):
            _b = bytes(self.p._handle.bulkRead(1, 16384))
            if not _b:
                break
            _chunks.append(_b)
            if len(_b) < 16384:
                break                      # short read -> device queue drained
        else:
            # Ran the cap without a short read: more was still waiting.
            self.drain_capped += 1
        self.drain_reads += len(_chunks)
        raw = self._buf + b''.join(_chunks)
        out = []
        i, n = 0, len(raw)
        while i + self._HEAD <= n:
            # SYNC MARKER, when the firmware emits one (CAN_SYNC_MARKER in
            # <repo>/firmware/board/can_comms.h).
            #
            # Accepting BOTH formats is deliberate: firmware and host can be
            # updated in either order without a flag day, and a mismatch degrades
            # to the old behaviour instead of killing reception outright.
            #
            # Once the marked firmware is running, `self._marked` latches and the
            # parser REQUIRES the marker. That is what makes the re-lock
            # deterministic: a false lock then needs marker + valid header +
            # valid XOR at exactly the right stride, and again for the packet
            # after it, because the next marker must also land.
            start = i
            if raw[i] == self._SYNC and (i + 1 + self._HEAD) <= n:
                hdr = i + 1
            elif self._marked:
                i += 1
                self.resync_bytes += 1
                continue
            else:
                hdr = i

            if hdr + self._HEAD > n:
                break

            dlc = raw[hdr] >> 4
            bus = (raw[hdr] >> 1) & 0x7
            dl = self._DLC_TO_LEN[dlc]
            if hdr + self._HEAD + dl > n:
                # might just be a packet split across two USB transfers -- only
                # keep it if the header is plausible, else it is garbage.
                if bus <= 2:
                    i = start
                    break
                i = start + 1
                self.resync_bytes += 1
                continue
            w = (raw[hdr + 1] | (raw[hdr + 2] << 8)
                 | (raw[hdr + 3] << 16) | (raw[hdr + 4] << 24))
            addr = w >> 3
            ok = bus <= 2 and (addr <= 0x1FFFFFFF if (w >> 2) & 1 else addr <= 0x7FF)
            if ok:
                c = 0
                for b in raw[hdr:hdr + self._HEAD + dl]:
                    c ^= b
                ok = (c == 0)
            if not ok:
                # Slide from the CANDIDATE START, not past the marker: the marker
                # byte itself may be payload that merely looked like one.
                i = start + 1
                self.resync_bytes += 1
                continue
            if hdr != start:
                # A marked packet parsed cleanly -- latch into strict mode.
                self._marked = True
            rejected = w & 1
            returned = (w >> 1) & 1
            tag = bus + (192 if rejected else (128 if returned else 0))
            out.append((addr, raw[hdr + self._HEAD:hdr + self._HEAD + dl], tag))
            i = hdr + self._HEAD + dl
        self._buf = raw[i:]

        # ---- WRONG-LOCK DETECTOR -------------------------------------------
        #
        # THE FAILURE THIS EXISTS FOR. The stream has no sync marker: packets are
        # concatenated, and a lost byte is recovered by sliding one byte until a
        # candidate passes bus<=2, the address range and an 8-bit XOR. Those are
        # weak: a wrong alignment passes often enough that, on a bus this
        # repetitive, the parser can LOCK ONTO THE WRONG OFFSET and stay there --
        # it keeps finding "valid" packets forever, advancing by their length.
        #
        # That is not a stall and not a wedge, so neither existing watchdog can
        # see it: throughput is completely normal. MEASURED 2026-08-13 on a real
        # drive: can rx 2316/s before the failure and 2320/s after, while every
        # decoded value was garbage (rpm 8196, brake and gas both 1) for FOUR
        # MINUTES. Suppression stopped, lateral stopped, nothing was logged.
        #
        # THE DISCRIMINATOR. A real CAN bus carries a small, stable set of IDs --
        # 23 on bus 0 of this car. A wrong lock reads the payload bytes of one
        # packet as the header of another, so its addresses are essentially
        # random and the DISTINCT-ADDRESS COUNT explodes. That needs no prior
        # knowledge of which IDs are expected and self-calibrates to any bus.
        #
        # Cheap: a set of at most a few hundred ints per window.
        if out:
            for _a, _d, _tag in out:
                self._lock_addrs.add(_a)
            self._lock_frames += len(out)
        _now_lk = _t0.monotonic()
        if _now_lk - self._lock_t0 >= self._LOCK_WINDOW_S:
            _distinct = len(self._lock_addrs)
            # Only judge a window with enough traffic to be meaningful; a quiet
            # bus legitimately shows few frames and few IDs.
            if self._lock_frames >= 200 and _distinct > self._LOCK_MAX_IDS:
                self.desync_count += 1
                self.last_wedge = "wrong-lock: %d distinct ids in %.1fs" % (
                    _distinct, _now_lk - self._lock_t0)
                # Drop the carry-over and re-lock from the next transfer. This is
                # the cheap fix and usually enough: a fresh transfer boundary is
                # a fresh chance to lock correctly.
                self._buf = b''
                self._lock_bad += 1
                print("!!! CAN WRONG-LOCK: %d distinct ids in %.1f s (%d frames) "
                      "-- dropping buffer to re-lock [%d]"
                      % (_distinct, _now_lk - self._lock_t0, self._lock_frames,
                         self._lock_bad), flush=True)
                # If re-locking does not take, the byte stream itself is broken
                # and only a board reset clears it -- the same escalation the
                # stall watchdog uses.
                if self._lock_bad >= 3:
                    print("!!! CAN WRONG-LOCK persists -- board reset", flush=True)
                    self._lock_bad = 0
                    try:
                        self._board_reset()
                    except Exception as _e:
                        print("    reset failed:", _e, flush=True)
            else:
                self._lock_bad = 0
            self._lock_addrs = set()
            self._lock_frames = 0
            self._lock_t0 = _now_lk

        # STALL WATCHDOG. Reception on this board dies silently under sustained
        # load: no exception, no BAD RECV, just an empty list forever while the
        # stack keeps publishing the LAST carState it decoded. Observed twice --
        # v, rpm and steering angle frozen at identical values for 112 s and
        # 175 s, cam_age climbing, and openpilot unable to engage with nothing in
        # the log to say why. A frozen carState is worse than a reported failure,
        # because every consumer believes it.
        #
        # Measured on a FRESH connection while stalled: 11 frames in 6 s with
        # 7116 bytes discarded by the resync, and the panda's own
        # rx_buffer_overflow at 1.79M -- so the device is producing frames the USB
        # path is not delivering intact. This does NOT fix that; it stops it being
        # permanent. 0xc0 clears the firmware's half-sent packet and we drop our
        # tail, which is the only realignment available from this side.
        import time as _t
        _now = _t.monotonic()

        # RATE-COLLAPSE WATCHDOG.
        #
        # The zero-frames watchdog below only fires when reception stops DEAD. The
        # real-world failure is worse than that and slips straight past it: once
        # can_rx_q is permanently full the link does not go silent, it goes to a
        # TRICKLE. Measured on a fresh connection while degraded: 1 frame/s against
        # the ~1400 frame/s the bus was actually carrying. `out` is non-empty often
        # enough that _last_frame_t keeps getting refreshed, so the 3 s timer never
        # expires -- reception was starved for 368 s with stall_recoveries == 0.
        #
        # A trickle is indistinguishable from a healthy bus if you only ask "did
        # anything arrive?", so ask "did the rate fall off a cliff?" instead. Track
        # the best rate this connection has genuinely sustained, and treat a drop to
        # under a fifth of it as the same failure the zero case represents.
        #
        # Guards: only arm once a real bus has been seen (_peak_rate > 200/s), so a
        # parked car at 0 frames/s is never "collapsed"; and rate-limit the recovery
        # to once per 10 s, because can_reset_communications discards in-flight
        # frames and retrying it in a tight loop would itself starve the feed.
        self._win_n += len(out)
        if self._win_t0 == 0.0:
            self._win_t0 = _now
        elif (_now - self._win_t0) >= 5.0:
            rate = self._win_n / (_now - self._win_t0)
            self._peak_rate = max(self._peak_rate, rate)
            collapsed = (self._peak_rate > 200.0 and rate < (0.2 * self._peak_rate))
            # SEPARATE COOLDOWN FROM THE BOARD RESET BELOW. These two used to
            # share self._last_reset_t, and that made the cheap recovery starve
            # the only one that works.
            #
            # Walk it: this block fires when collapsed and >10 s since the last
            # reset, and it is checked every 5 s -- so while a collapse persists
            # it re-fires every ~15 s and stamps the shared timestamp each time.
            # The wedge detector below requires >15 s since that same stamp, and
            # runs AFTER this block in the same call, so it saw 0 s elapsed every
            # single time. It could never fire while a collapse was in progress
            # -- which is precisely when it is needed.
            #
            # MEASURED: across the 2026-08-13 failures the endpoint was wedged
            # for 264 s and then 452 s with wedge_recoveries stuck at 0, while
            # can_reset_communications (which cannot clear this fault -- only a
            # 0xd8 can) retried harmlessly throughout.
            if collapsed and (_now - self._last_soft_reset_t) > 10.0:
                self.rate_recoveries += 1
                self._last_soft_reset_t = _now
                self._buf = b''
                try:
                    self.p.can_reset_communications()
                except Exception:
                    pass
            self._win_t0, self._win_n = _now, 0

        # SILICON-VS-HOST WEDGE DETECTOR. The authoritative test, and the only one
        # that cannot be fooled by a quiet bus.
        #
        # Both watchdogs above infer a fault from the host's own arrival rate, which
        # is ambiguous: 4 frames/s looks the same whether the car is parked or the
        # USB link has died. CAN1's hardware counter settles it -- it counts what the
        # SILICON received, regardless of what crossed the link.
        #
        # MEASURED 2026-08-11 while carState had been frozen for 11 minutes:
        #     silicon      2327 frames/s
        #     reaching host   4 frames/s      99.8% loss
        #     rx overflow     0 frames/s      the firmware was NOT dropping them
        #     can_recv calls 7436/s           we were polling correctly
        #
        # So the frames were not queued and discarded, they were never delivered.
        # rx_ovf reaching 1,440,340 was the CONSEQUENCE -- the queue backs up because
        # nothing drains it -- not the cause. And drain_over_budget stayed 0
        # throughout, because the host met every deadline against an empty pipe.
        #
        # can_reset_communications() does NOT clear this; a board reset does
        # (measured: 4/s -> 1425/s immediately after 0xd8). So escalate.
        if _now - self._hw_check_t >= 5.0:
            try:
                hw = self.p.can_health(0)["total_rx_cnt"]
            except Exception:
                hw = None
            if hw is not None and self._hw_last is not None:
                hw_rate = (hw - self._hw_last) / (_now - self._hw_check_t)
                host_rate = self._hw_n / (_now - self._hw_check_t)
                # Only meaningful with a genuinely busy bus; a parked car is not a
                # wedge. 10% is far below the ~60% the host filter alone explains.
                if hw_rate > 200.0 and host_rate < 0.10 * hw_rate:
                    # Own cooldown -- see the note in the rate watchdog above.
                    # 15 s is long enough that a reset gets a fair chance to
                    # take effect before another is considered, and short
                    # enough that a wedge costs seconds rather than minutes.
                    if (_now - self._last_board_reset_t) > 15.0:
                        self._last_board_reset_t = _now
                        self.wedge_recoveries += 1
                        self.last_wedge = "silicon %.0f/s host %.0f/s" % (hw_rate, host_rate)
                        self._board_reset()
                # SECOND WEDGE CASE: THE SILICON ITSELF IS FROZEN.
                #
                # The check above catches a live core whose frames are not reaching
                # the host. It cannot catch a core stuck in bxCAN INITIALISATION,
                # because then total_rx_cnt does not advance either and hw_rate is
                # 0 -- the `hw_rate > 200` guard skips it entirely.
                #
                # That is the KEY-CYCLE failure, and it is the one that has been
                # mistaken for the alpha-long radar fault more than once. bxCAN
                # needs 11 consecutive recessive bits to leave init; with the engine
                # off the bus never supplies them, and turning the ignition back on
                # does NOT retrigger init. Re-setting the safety mode cannot help a
                # core that is already wedged -- only a device reset does.
                #
                # It is invisible to every health field: MEASURED 2026-08-09 over a
                # 543 s run, CAN1 total_rx_cnt frozen at 5,263,945 with delta 0/s
                # while the ignition was ON, and TEC=0, REC=0, bus_off=0, last_error
                # "No error". A core in init neither receives nor error-counts, so
                # nothing reports a problem.
                #
                # It is not merely a reception failure. Forwarding runs inside
                # can_rx(), so a wedged core also stops bridging cam <-> bus 0 --
                # and with no harness relay this panda is the ONLY path between
                # them. The cluster then raises a front camera sensor fault that
                # looks exactly like the radar-suppression one.
                #
                # Signature: ignition asserted, zero frames, for two consecutive
                # windows (10 s). Two rather than one so a momentary gap during the
                # crank itself does not trigger a reset.
                elif hw_rate == 0.0 and self._ignition_on():
                    self._frozen_windows += 1
                    # Board-reset cooldown, not the soft one: this branch also
                    # calls _board_reset(), so it must share the timestamp with
                    # the wedge case above and NOT with can_reset_communications.
                    if (self._frozen_windows >= 2
                            and (_now - self._last_board_reset_t) > 15.0):
                        self._last_board_reset_t = _now
                        self._frozen_windows = 0
                        self.wedge_recoveries += 1
                        self.last_wedge = "core frozen in init (ignition on, 0 rx for %.0fs)" % (
                            2 * (_now - self._hw_check_t))
                        self._board_reset()
                else:
                    self._frozen_windows = 0
            self._hw_last = hw
            self._hw_check_t = _now
            self._hw_n = 0
        self._hw_n += len(out)

        if out:
            self._last_frame_t = _now
        elif self._last_frame_t and (_now - self._last_frame_t) > 3.0:
            self.stall_recoveries += 1
            self._last_frame_t = _now
            self._buf = b''
            try:
                self.p.can_reset_communications()
            except Exception:
                pass
        return out

    def can_recv(self):
        """Genuine received frames only, as (addr, data, bus).

        unpack_can_buffer() folds two status bits into the bus number: +128 for a
        TX echo (it went out), +192 for safety-rejected (it never reached the
        wire). SerialPanda's can_recv() returned BOTH as ordinary bus-0 traffic,
        so the stack was decoding its own output as if the car had sent it. That
        is harmless for 0x243 (CarStateFromCAN does not decode it) but NOT under
        --alpha-long, where we send 0x21c CRZ_CTRL -- a frame CarStateFromCAN
        DOES decode for cruise state. Reading our own command back as truth is a
        feedback loop, so drop them here and count them instead.
        """
        try:
            frames = self._recv_resync()
        except AssertionError:
            # "CAN packet checksum incorrect" -- the byte stream has lost frame
            # alignment. unpack_can_buffer() has no resync: it asserts on the
            # first bad header and gives up, and because the bad bytes stay in
            # can_rx_overflow_buffer, EVERY later call throws too. One hiccup
            # therefore wedges reception permanently, which is what the storm of
            # "CAN: BAD RECV, RETRYING" in the log was -- 0.1 s of sleep per
            # retry, so the panda's rx queue then overflowed (53014 and climbing)
            # and the stack went blind while still transmitting at 100 Hz.
            #
            # Realigning needs BOTH ends: 0xc0 clears the firmware's half-sent
            # packet (can_read_buffer), and dropping the host buffer discards the
            # bytes we can no longer place. Losing one batch of frames is fine;
            # they are re-sent by the car at 100 Hz.
            self.desync_count += 1
            try:
                self.p.can_reset_communications()
            except Exception:
                pass
            self.p.can_rx_overflow_buffer = b''
            return []
        out = []
        for addr, dat, bus in frames:
            if bus >= 192:
                self.rejected_count += 1
            elif bus >= 128:
                self.echo_count += 1
            else:
                out.append((addr, dat, bus))
        return out

    def close(self):
        try:
            self.p.close()
        except Exception:
            pass
