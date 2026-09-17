# The host link: what went wrong on USB, and what SPI changes

Written 2026-09-15 after a week of chasing a link failure that presented, in
turn, as a radar fault, a CPU scheduling problem, and a drain-rate problem
before it turned out to be none of those.

Read this before rewriting the transport. Every number here was measured on
the car, not inferred.


## 1. The failure

Symptom on the road: carState freezes. Speed, RPM and steering angle hold
their last value indefinitely. openpilot disengages itself. The dash lights
up. Nothing in the logs says why.

What the counters actually showed, with nothing running but a diagnostic:

    bulkRead(1, 16384)  ->  4 bytes, every time
    CAN silicon             2327 frames/s   -- receiving normally
    reaching the host          4 frames/s   -- 99.8% loss
    rx_buffer_overflow     1,163,362        -- and climbing
    0xd8 board reset        clears it instantly

The last line is the important one. A USB-side reset fixes it completely, so
the fault is endpoint state -- not CAN, not the queue, not comms_can_read.


## 2. Root cause: an EP1 transfer started on top of one in flight

`USB_WritePacket` arms the endpoint BEFORE filling its FIFO:

    USBx_INEP(ep)->DIEPTSIZ = ((numpacket << 19) & PKTCNT) | (len & XFRSIZ);
    USBx_INEP(ep)->DIEPCTL |= (CNAK | EPENA);   // armed here...
    for (i = 0; i < count32b; i++)               // ...still empty until this ends
        USBx_DFIFO(ep) = *src_copy++;

That leaves a window -- roughly 16 store instructions -- where the endpoint is
enabled and its FIFO is empty.

EP1 uses `ITTXFE` as its fill trigger. ITTXFE means literally "IN token
received when TxFIFO is empty". A host token landing inside that window raises
it WITH EPENA ALREADY SET. The handler then called USB_WritePacket again,
rewriting DIEPTSIZ while the first transfer was still outstanding and pushing
16 more words into a FIFO that already held 16.

    1. DIEPTSIZ rewritten to "1 packet / 64 bytes" mid-transfer
    2. FIFO now holds 128 bytes
    3. core sends 64, XFRC fires, EPENA clears -- 64 bytes STRANDED
    4. every later fill stacks on the stranded bytes
    5. the 64-word FIFO saturates, pushes are dropped silently by the core
    6. packets tear; the descriptor never realigns -- runt packets forever

ITTXFE is an UNDERRUN NOTIFICATION, not a fill trigger. ST's programming model
fills IN endpoints from TXFE/DIEPEMPMSK. EP0 in this very file does exactly
that, and checks DTXFSTS for room before writing. EP1 never did. The two
faults are complementary: either alone would be harmless.

### Why it is a race and not a capacity limit

The same build ran 11 minutes on one drive and 61 on the next. A queue that is
merely too small fails at a repeatable load. A race fails whenever the timing
happens to line up. That variance is what finally moved the investigation off
drain rate.

### comms_can_read is not the source

Ruled out arithmetically rather than by inspection. With 15-byte packets into
a 64-byte buffer, `can_read_buffer.ptr` cycles

    0, 11, 7, 3, 14, 10, 6, 2, 13, 9, 5, 1, 12, 8, 4

and EVERY one of those states returns a full 64 while the queue is non-empty.
It cannot return 4. The queue was overflowing at 932 frames/s, so can_pop
always succeeded.

### The fix that is flashed

Guard the write; count what the guard catches so it proves itself.

    if (DIEPCTL & EPENA)                 ep1_busy_skips++;      // still open
    else if (DTXFSTS_avail < 64/4)       ep1_nospace_skips++;   // no room
    else                                 USB_WritePacket(...);

Skipping is safe and self-correcting: with the FIFO non-empty the core answers
the token from what is there; if it is empty the core NAKs and the host
retries. No frames are lost -- they stay in can_rx_q.

Counters read back via control request 0xda, deliberately NOT in health_t: a
struct change means bumping HEALTH_PACKET_VERSION and updating every host that
unpacks it, and this is a diagnostic, not part of the contract.

STATUS: flashed 2026-08-13, image dd815b01, 58356 B. NOT YET PROVEN. busy_skips
read 0 after a few minutes, which means nothing when the failure took 11 and 61
minutes previously. The measurement that matters is a drive of comparable
length. If busy_skips comes back non-zero, the diagnosis is confirmed.


## 3. Why the protocol is fragile, beyond that one bug

USB bulk is reliable ON THE WIRE -- CRC, retries, guaranteed delivery. The
corruption above happened BEFORE the wire, in the device's own FIFO. The bytes
that crossed arrived perfectly; they were simply the wrong bytes. Every
protection layer USB offers was downstream of the fault.

Four properties compound it:

**No framing.** The stream is packets concatenated with no delimiter. Lose one
byte and the host must guess the boundary, and its only tests are bus <= 2, an
address range, and an 8-bit XOR. Those are weak enough that a WRONG alignment
passes often -- and on a repetitive bus the parser locks onto it and keeps
finding "valid" packets forever, advancing by the wrong stride. Measured: four
minutes at full throughput (2316 frames/s before, 2320 after) with every
decoded value garbage, invisible to every watchdog because nothing was slow or
missing.

**A 1-in-256 checksum.** An 8-bit XOR admits roughly one corrupt frame in 256
by chance. At 2900 frames/s a stream with holes delivers ~11 plausible-looking
bad frames per second. That is how v=379.7 kph and rpm=14282 reached carState.

**No acknowledgment.** The host asks for 16 KB and takes whatever arrives.
There is no sequence number and no ack, so loss is not merely unhandled, it is
UNOBSERVABLE. The host cannot distinguish a quiet bus from a dead link.

**Every failure is silent.** rx_ovf passed a million with nothing reading it.
The firmware's own can-rx counter kept climbing because the silicon was fine.
Decoded values froze rather than erroring. Nothing in the chain ever said "I
am broken", which is why three separate investigations blamed the wrong layer.


## 4. Mitigations already in the tree (keep these on any transport)

**Per-packet sync marker.** CAN_SYNC_MARKER 0xAA prefixes every packet. 0xAA
decodes as bus 5, so it can never legally start a packet -- which means a host
parsing an UNMARKED stream loses nothing by treating it as a candidate, and
both formats coexist so firmware and host can be updated in either order. The
host requires marker + valid header + valid XOR and chains that across
packets, making re-lock deterministic instead of a guess.

**Bounded drain loop (host).** One 16 KiB read holds ~1260 packets, about
439 ms of traffic at 2870 frames/s, against a measured worst drain gap of
428 ms. A single read barely covered a single gap; the excess accumulated
until can_rx_q (2048 frames) dropped it. Read until short, capped at 8.

**Two independent reset cooldowns (host).** The rate-collapse watchdog and the
silicon-vs-host wedge detector shared one timestamp. The first re-fires every
~15 s during a collapse and runs first in the same call, so the second always
saw 0 s elapsed and could never fire -- and can_reset_communications CANNOT
clear a wedged endpoint, only a 0xd8 can. The ineffective recovery was
permanently suppressing the effective one. Measured: wedged 264 s and then
452 s with wedge_recoveries stuck at 0.

**Liveness from arrival time, never from a decoded value.** This shape bit the
project twice. set_speed_raw latched and made a false conclusion unfalsifiable
for weeks. acc_active latched and held the radar suppressed for 264 s with no
brake, cancel or MAIN-off able to clear it, because the release that frees it
is itself driven by the receive path that had died. A decoded VALUE can never
establish liveness. Only an arrival TIME can.


## 5. Did comma ship USB? Yes, for years.

From board/boards/ in this tree:

    dos      has_spi=false  -> USB    comma two's panda (STM32F4)
    red      has_spi=false  -> USB    red panda (STM32H7)
    nucleo   has_spi=false  -> USB    this port
    tres     has_spi=true   -> SPI    comma three
    cuatro   has_spi=true   -> SPI    comma 3X

USB was the production transport across comma's fleet for most of the
company's history. SPI arrived with the comma three, when the panda moved onto
the same board as the SOM and a cable stopped being necessary.

So USB demonstrably CAN work in production. Two things follow:

- The EP1 defect is present in commaai/panda today, unchanged. Their current
  hardware does not load it. Whether it ever hurt dos or red is unknown.
- The FIFO allocation here is byte-identical to upstream (GRXFSIZ 0x40,
  DIEPTXF0 (0x40<<16)|0x40, DIEPTXF[0] (0x40<<16)|0x80), so the race window
  was not widened by mis-sizing anything locally.

Plausible reasons it did not visibly bite comma, none verified: red is H7 with
a different USB core revision, only dos shares the F4 core; their host read at
a different cadence, and the race needs a token inside a sub-microsecond
window; this board runs the CAN RX ISR at ~2900/s on the same core, and
anything delaying the ISR between EPENA and the last FIFO write widens the
window. It may also simply have looked like "USB flakiness" and got a retry
wrapper rather than an investigation.


## 6. If you stay on USB

Non-negotiable, in order:

1. Keep the EP1 guard, or better, move EP1 to the EP0 pattern: fill from
   TXFE/DIEPEMPMSK with a DTXFSTS space check. That eliminates the window
   rather than tolerating it.
2. Keep the sync marker.
3. Replace the 8-bit XOR with something that actually discriminates -- CRC-8
   at minimum, and a per-packet sequence number so loss is visible.
4. Keep the bounded drain loop and both reset cooldowns.
5. Surface rx_ovf, drain_reads, drain_capped and the guard counters somewhere
   a human sees them. Most of this week was spent because nothing did.


## 7. Moving to SPI

The protocol is host-polled with a software handshake, so there is NO
DATA_READY or IRQ line to wire. Four signals:

    SCK  MOSI  MISO  CS      (+ common ground)

Handshake, from python/spi.py and board/drivers/spi.h:

    SYNC 0x5A   host header, with checksum
    HACK 0x79   device: header accepted
    DACK 0x85   device: data accepted
    NACK 0x1F   device: not ready, retry
    CHECKSUM_START 0xAB

Every transaction is acknowledged or refused, header and data are checksummed
separately, the host drives, and the device holds no streaming state between
transactions. The failure modes in section 3 become structurally impossible
rather than merely guarded against.

### Hardware

3.3 V both sides -- Orin Nano 40-pin header and STM32F407 I/O. No level
shifting.

Jetson already exposes /dev/spidev0.0, 0.1, 1.0, 1.1. The host code defaults
to /dev/spidev0.0.

F407 pins are clear. Currently in use:

    PA2/PA3     USART2 (ST-Link VCP)
    PB8/PB9     CAN1  -> car main bus
    PB5/PB6     CAN2  -> camera/LKAS bus

SPI2 on PB12(NSS)/PB13(SCK)/PB14(MISO)/PB15(MOSI) collides with none of them.
SPI1 on PA4-PA7 is also free. SPI2 keeps port A clear.

Bandwidth is a non-issue: ~40 KB/s needed, spidev on Orin does tens of MHz.

### Software

    board/drivers/spi.h        COMPLETE, MCU-independent, calls comms_can_read
    python/spi.py              COMPLETE host transport
    board/boards/nucleo.h      flip .has_spi = false -> true
    board/stm32f407/llspi.h    A 12-LINE STUB -- this is the whole job

The stub:

    void llspi_miso_dma(uint8_t *addr, int len) { UNUSED(addr); UNUSED(len); }
    void llspi_mosi_dma(uint8_t *addr, int len) { UNUSED(addr); UNUSED(len); }
    void llspi_init(void) { }

The H7 version is 107 lines. You cannot copy it -- different DMA controller,
different register layout -- so budget ~110 lines written against the F4
reference manual. That is the only genuinely new engineering.

### Honest tradeoff

SPI has no hot-plug, no enumeration, and a short reliable cable length. All
three are irrelevant for two boards bolted together in a car, and all three
are why USB exists for the desktop. For a fixed board-to-board link inside a
vehicle, SPI is the conventional answer and USB is the unusual one.
