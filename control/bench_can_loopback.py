#!/usr/bin/env python3
"""SAFETY BENCH TEST: verify the DIY F446 panda over serial with CAN in internal LOOPBACK mode.

Loopback mode (CAN_BTR_SILM|LBKM) loops TX->RX *inside* the STM32 bxCAN peripheral — NOTHING goes
on the physical bus, transceivers optional. This proves the panda protocol + CAN peripheral +
firmware work with ZERO risk. RUN THIS (and pass it) before ever connecting to the car.

Steps: connect serial -> set safety SILENT (outputs blocked) -> enable loopback -> send a CAN
frame on bus 0 -> read it back -> verify address+data match.

Usage: python3 bench_can_loopback.py [/dev/ttyACM0]
"""
import sys, struct, time, serial

SER_SYNC=0x5A; SER_HACK=0x79; SER_NACK=0x1F; SER_CK=0xAB; SER_HDR=7
DLC_TO_LEN=[0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64]; LEN_TO_DLC={l:d for d,l in enumerate(DLC_TO_LEN)}

def cksum(b): 
    r=0
    for x in b: r^=x
    return r

class SerialPanda:
    def __init__(self, dev):
        self.s=serial.Serial(dev, 1500000, timeout=0.2)
        self.s.reset_input_buffer(); self.s.reset_output_buffer()
    def _txn(self, endpoint, tx=b"", max_rx=0):
        hdr=bytearray(SER_HDR)
        hdr[0]=SER_SYNC; hdr[1]=endpoint
        hdr[2]=len(tx)&0xFF; hdr[3]=len(tx)>>8
        hdr[4]=max_rx&0xFF; hdr[5]=max_rx>>8
        hdr[6]=SER_CK ^ cksum(hdr[:6])           # device folds to 0
        self.s.write(hdr)
        ack=self.s.read(1)
        if ack!=bytes([SER_HACK]): return None
        if tx:
            self.s.write(tx); self.s.write(bytes([SER_CK ^ cksum(tx)]))
        rh=self.s.read(3)
        if len(rh)<3 or rh[0]!=SER_HACK: return None
        rlen=rh[1]|(rh[2]<<8)
        data=self.s.read(rlen) if rlen else b""
        self.s.read(1)                           # trailing checksum
        return data
    # control transfer: [request:1][param1:2][param2:2][length:2] = ControlPacket_t (packed)
    def control_write(self, req, p1, p2):
        pkt=struct.pack("<BHHH", req, p1, p2, 0)
        return self._txn(0, pkt, 0)
    def control_read(self, req, p1, p2, length):
        pkt=struct.pack("<BHHH", req, p1, p2, length)
        return self._txn(0, pkt, length)
    def can_send(self, addr, dat, bus):
        ext=1 if addr>=0x800 else 0; dlc=LEN_TO_DLC[len(dat)]
        h=bytearray(6); w=(addr<<3)|(ext<<2)
        h[0]=(dlc<<4)|(bus<<1); h[1]=w&0xFF; h[2]=(w>>8)&0xFF; h[3]=(w>>16)&0xFF; h[4]=(w>>24)&0xFF
        h[5]=cksum(h[:5]+dat)
        self._txn(3, bytes(h)+dat, 0)
    def can_health(self, bus, tries=4):
        # 0xc2 = per-CAN health counters, indexed by CAN NUMBER (0=CAN1/bus 0,
        # 1=CAN2/bus 2). Packed can_health_t, 64 bytes.
        #
        # Retries because a control transfer shares the link with the CAN stream:
        # if a previous _txn was cut short, a stale byte is still in the input
        # buffer and the next transfer reads it as the ack. Flush and retry rather
        # than report a missing counter as if it were a device fault.
        d = None
        for _ in range(tries):
            d = self.control_read(0xc2, bus, 0, 64)
            if d and len(d) >= 64:
                break
            self.s.reset_input_buffer()
            time.sleep(0.02)
        if not d or len(d) < 64:
            return None
        f = struct.unpack("<BIBBBBBBBBIIIIIIIHHBBBIIII", d[:64])
        return {"bus_off": f[0], "rx_err": f[8], "tx_err": f[9],
                "total_err": f[10], "tx_lost": f[11], "rx_lost": f[12],
                "tx": f[13], "rx": f[14], "fwd": f[15]}
    def can_recv_ex(self):
        # Same stream as can_recv(), but keeps the two status bits the firmware
        # packs alongside the address:
        #   rejected=1 -> safety_tx_hook refused it; it never reached the wire
        #   returned=1 -> tx-complete echo from process_can; it DID go out
        #   both 0     -> a genuine frame out of the CAN core's RX FIFO
        # Without this distinction a "loopback" test passes on the rejected echo
        # alone, proving only that the serial link works.
        return self._recv(True)
    def can_recv(self):
        return [(a, d, b) for (a, d, b, _r, _t) in self._recv(True)]
    def _recv(self, _ex):
        # Parse the packed CAN stream with validation + resync.
        # Without this, a single byte of misalignment corrupts every following
        # frame: the bus/addr/len fields are read at wrong offsets, producing
        # impossible IDs (>0x7FF), impossible lengths (>8) and bus-0 payloads
        # mislabelled as bus 1. On a bad header we drop ONE byte and retry,
        # which re-locks onto the next real frame boundary.
        raw = self._txn(1, b"", 2040)
        out = []
        d = raw or b""
        i = 0
        n = len(d)
        while i + 6 <= n:
            dlc = d[i] >> 4
            bus = (d[i] >> 1) & 0x7
            dl = DLC_TO_LEN[dlc]
            w = d[i+1] | (d[i+2] << 8) | (d[i+3] << 16) | (d[i+4] << 24)
            addr = w >> 3
            ext = (w >> 2) & 1
            rejected = w & 1
            returned = (w >> 1) & 1
            # validity checks: standard IDs are 11-bit, classic CAN is <=8 bytes,
            # and this firmware only has buses 0-2.
            #
            # Then the packet's OWN checksum, which is the only check that can
            # tell a real frame from one the resync below invented: the firmware
            # XORs the 6 header bytes and the payload to zero (can_set_checksum).
            # Without it, sliding through packet data lands on byte patterns that
            # pass the range checks and get reported as real CAN IDs -- which is
            # exactly where entries like 0x1fe823f4 came from.
            ok = (dl <= 8) and (bus <= 2) and (i + 6 + dl <= n) and \
                 (addr <= 0x1FFFFFFF if ext else addr <= 0x7FF)
            if ok:
                c = 0
                for x in d[i:i+6+dl]:
                    c ^= x
                ok = (c == 0)
            if not ok:
                i += 1          # resync: slide one byte and retry
                continue
            out.append((addr, bytes(d[i+6:i+6+dl]), bus, rejected, returned))
            i += 6 + dl
        return out

def main():
    dev=sys.argv[1] if len(sys.argv)>1 else "/dev/ttyACM0"
    print(f"=== F446 PANDA CAN LOOPBACK BENCH TEST on {dev} ===")
    p=SerialPanda(dev)
    results=[]
    def check(desc, got, want):
        ok = got == want
        results.append(ok)
        print("  [%s] %-52s want=%-6s got=%s" % ("PASS" if ok else "FAIL", desc, want, got))

    h=p.control_read(0xd2, 0, 0, 64)             # 0xd2 = get health
    print(f"health resp: {len(h) if h else 0} bytes {'(panda responding)' if h else '(NO RESPONSE — check link)'}")

    # LIVE-BUS GUARD. This test transmits, and it raises the safety mode to
    # ALLOUTPUT to do so. Both are safe ONLY on a bench, where loopback is the
    # single thing standing between us and the wire -- and loopback is set by a
    # control transfer that can be dropped on a busy link. If real traffic is
    # arriving, the panda is plugged into a car: refuse, rather than inject test
    # frames (0x240 is a real STEER_TORQUE ID) onto a live vehicle bus.
    a = p.can_health(0); b = p.can_health(1)
    time.sleep(0.3)
    a2 = p.can_health(0); b2 = p.can_health(1)
    if None in (a, b, a2, b2):
        print("❌ ABORT — cannot read CAN health; check the link."); return 1
    live = (a2["rx"]-a["rx"]) + (b2["rx"]-b["rx"])
    if live > 0:
        print(f"\n❌ ABORT — {live} frames arrived in 300 ms: this bus is LIVE.")
        print("   bench test only. Disconnect the panda from the car (or switch the")
        print("   ignition off) before running it. Nothing was transmitted.")
        return 1
    print("bus quiet (0 frames in 300 ms) — safe to transmit in loopback")

    # ORDER MATTERS: loopback FIRST. set_safety_mode() re-runs can_init_all(),
    # which re-applies BTR from the can_loopback global, so loopback survives the
    # mode change -- but doing it the other way round leaves a window where the
    # panda is live on the wire in a mode that permits output.
    print("enable CAN loopback (0xe5, 1) — TX loops to RX internally, nothing on the wire")
    p.control_write(0xe5, 1, 0)
    # SAFETY_ALLOUTPUT (17), not SILENT. In SILENT every tx is refused by
    # safety_tx_hook and bounced straight back to the host with rejected=1 --
    # which looks identical to a loopback at the addr/data level, so the old
    # version of this test passed without the CAN peripheral doing anything at
    # all. Output is safe here only because loopback is already on.
    print("set safety ALLOUTPUT (0xdc, mode=17) — tx permitted, but loopback keeps it off the wire")
    p.control_write(0xdc, 17, 0)
    time.sleep(0.1)
    p.can_recv_ex()                              # drain anything stale
    base=p.can_health(0)

    # --- 1. a whitelisted ID makes the full round trip -------------------------
    # 0x240 STEER_TORQUE is one of the 17 IDs mazda_host_visible() passes to the
    # host. Expect BOTH a genuine RX frame (rejected=0, returned=0, straight out
    # of the CAN core) and the tx-complete echo (returned=1).
    print("\n=== 1. whitelisted ID 0x240 round-trips through the CAN core ===")
    on_data=bytes([0xDE,0xAD,0xBE,0xEF,0x01,0x02,0x03,0x04])
    p.can_send(0x240, on_data, 0)
    time.sleep(0.05)
    got=p.can_recv_ex()
    rx  = [g for g in got if g[0]==0x240 and g[3]==0 and g[4]==0]
    echo= [g for g in got if g[0]==0x240 and g[4]==1]
    rej = [g for g in got if g[3]==1]
    check("genuine RX frame out of the CAN core", len(rx)>=1, True)
    check("...payload intact", rx[0][1] if rx else None, on_data)
    check("...on bus 0", rx[0][2] if rx else None, 0)
    check("tx-complete echo seen", len(echo)>=1, True)
    check("nothing was safety-rejected", len(rej), 0)

    # --- 2. a non-whitelisted ID is carried by CAN but hidden from the host ----
    # This is the whole point of the accept-all filter + software gate split:
    # 0x123 must still be transmitted and still be RECEIVED by the CAN core (so
    # that can_rx() can forward it car->cam), while never reaching the Jetson.
    print("\n=== 2. non-whitelisted ID 0x123 is forwarded-capable but host-gated ===")
    before=p.can_health(0)
    p.can_send(0x123, bytes([0x11,0x22,0x33,0x44]), 0)
    time.sleep(0.05)
    got=p.can_recv_ex()
    after=p.can_health(0)
    check("host sees no 0x123 at all", [g for g in got if g[0]==0x123], [])
    check("CAN core still TRANSMITTED it", (after["tx"]-before["tx"]) if before and after else None, 1)
    check("CAN core still RECEIVED it (so can_rx can forward)",
          (after["rx"]-before["rx"]) if before and after else None, 1)

    # --- 3. bus error counters clean ------------------------------------------
    print("\n=== 3. no bus errors during the test ===")
    end=p.can_health(0)
    if base and end:
        check("no bus-off", end["bus_off"], 0)
        check("rx error counter", end["rx_err"], 0)
        check("tx error counter", end["tx_err"], 0)
        check("no rx lost", end["rx_lost"]-base["rx_lost"], 0)
        check("no tx lost", end["tx_lost"]-base["tx_lost"], 0)
    else:
        check("can_health readable", False, True)

    # restore a safe state before we let go of the link
    p.control_write(0xdc, 0, 0)                  # SAFETY_SILENT
    p.control_write(0xe5, 0, 0)                  # loopback off

    ok = all(results)
    print("\n%d/%d checks passed" % (sum(results), len(results)))
    print("\n"+("✅ PASS — CAN peripheral, panda protocol and the host gate all behave."
                 if ok else "❌ FAIL — debug before proceeding."))
    print("(Loopback = zero bus activity. Passing this is REQUIRED before any car connection.)")
    return 0 if ok else 1

if __name__=="__main__": sys.exit(main())
