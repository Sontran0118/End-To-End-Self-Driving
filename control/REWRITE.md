# Writing this system from scratch

A build order for replacing the Python stack with C++ and the panda firmware
with your own, without re-earning a summer of measurements.

Companion document: firmware/HOST_LINK.md covers the transport layer
in detail -- read it before touching USB or SPI.


## 0. The scope is smaller than the line count suggests

    board/ total                136,054 lines   <- almost all vendor CMSIS
    board/stm32f407/ (yours)        927 lines   <- write from scratch
    board/drivers/  (shared)      3,880 lines   <- write from scratch
    opendbc safety/modes/mazda.h    636 lines   <- ALREADY C, reuse verbatim
    host stack (Python)           9,897 lines   <- port to C++

So roughly 5k lines of firmware and 10k of host code. Not 136k.


## 1. What NOT to rewrite

**The safety model.** `car/opendbc/safety/modes/mazda.h` is already C, already
compiles into the firmware, and is already tested through the real compiled
code by override_test2 (16/16) and button_engage_test (14/14). It is also the
audited part. Rewriting the board layer is reasonable; rewriting the safety
model is a much larger risk than it looks.

**The empirical constants.** The code is replaceable. These are not -- each
cost real driving to establish, and a clean-room rewrite silently discards
them:

    STEER_MAX / max_rate_down / max_torque     measured steering envelope
    23 distinct IDs on bus 0                   the ID census for this car
    0x21c MSG_1 redundancy quad                upstream templates invert it,
                                               so every synthesised frame was
                                               discarded until this was found
    ACCEL_MAX_BP / ACCEL_MAX_V * 0.55          the accel curve you settled on
    T_FOLLOW_MIN = 0                           lead distance belongs to the model
    UDS 0x764 session behaviour                only 10 02 is answered; 19 02 FF,
                                               19 0A, 10 03, OBD 03/07 all get
                                               no reply at all
    OP_SENSOR_ID=0, OP_CAMERA_FLIP=2           both Jetson defaults are wrong
    GPS 38400 on ttyTHS1                       not the default

**The regression suites.** Six of them, and the good ones already test through
real compiled C rather than a Python reimplementation. Point them at the new
code instead of writing new tests:

    override_test2        16/16   steering limits through the real safety C
    button_engage_test    14/14   host / opendbc / panda arming agree 3 ways
    coast_band_test       26/26   target bands, coast region, model passthrough
    rx_deadman_test       13/13   suppression fails open when the link dies
    drain_test            13/13   USB queue empties and stays bounded
    wedge_cooldown_test    9/9    board reset is not starved by the soft reset

Every one of these reproduces its bug FIRST. If the old behaviour passes the
test, the test is rejected as non-discriminating. Keep that discipline -- a
check that cannot fail is not evidence.


## 2. Firmware, bottom-up

Each layer testable on the bench before the next. Do not move up until the
current layer is proven.

**1. Clock, flash, linker.** 168 MHz PLL (the F407 ceiling -- 180 is the F446). Sector map for the F407. Keep
`.isr_vector` at 0x8004000 if you keep the bootstub, and keep the bootstub --
it is what lets you recover a bad flash without SWD.

**2. bxCAN.** TX, RX, acceptance filters wide open, forwarding in the RX ISR.
Prove it with an internal-loopback harness before it ever sees a car.

  Trap already paid for: bxCAN needs 11 consecutive recessive bits to leave
  initialisation. With the engine off the bus never supplies them, and turning
  the ignition on later does NOT retrigger init. The core then sits wedged,
  receiving nothing and error-counting nothing -- TEC=0, REC=0, bus_off=0,
  last_error "No error" -- so no health field reports a problem. Forwarding
  runs inside can_rx(), so a wedged core also stops bridging cam <-> car and
  the cluster raises a front camera fault that looks exactly like the
  radar-suppression one. Force the idle state recessive with a pull-up, and
  never start the stack on a dead bus.

**3. Host link.** See HOST_LINK.md. If USB: fill the FIFO before EPENA or
guard EPENA/DTXFSTS, and drive EP1 from TXFE/DIEPEMPMSK rather than ITTXFE.
If SPI: ~110 lines of F4 DMA in llspi.h and the rest is already written.

**4. Ring buffers and host protocol.** Keep the per-packet sync marker.
Upgrade the 8-bit XOR to CRC-8 and add a sequence number while you are here --
both are cheap and both turn silent corruption into a visible counter.

**5. Safety hooks.** Drop in mazda.h unchanged.

**6. Health and counters.** Whatever you add, make sure a human can see it.
Most of this project's lost time was spent because failures were silent.


## 3. Control stack in C++

Less work than it sounds, because the substrate is already C++ -- you are
removing a layer, not adding one.

    cereal / msgq          C++ natively; the Python is the binding
    controlsd internals    latcontrol_torque, drive_helpers have C++ upstream
    your own logic         govern_accel, SpeedTarget, RxLiveness, drain loop
                           -- a few hundred lines each

Order: transport (libusb C API, which is what the Python wraps) -> carstate
decode -> govern_accel -> the CAN TX thread.

Keep the two-role split. One process owns the USB/SPI handle, CAN receive, TX
and carState; the other runs the camera, the network and the planner and sends
carControl back over msgq. It exists so GPU work cannot stall CAN work, and
that reason does not go away in C++.


## 4. Model inference in C++

The easiest piece. You are already on TensorRT, and TRT's NATIVE api is C++ --
the Python `tensorrt` module is a binding over it. The .trt engine file is
unchanged.

    IRuntime -> deserializeCudaEngine -> IExecutionContext -> enqueueV3
    + CUDA buffers for input/output

The non-obvious work is around the inference, not in it:

    YUV420 -> 12-channel pack          currently NumPy
    calibration perspective warp       currently NumPy (get_warp_matrix)
    recurrent feature buffer           feat queue depth 96, subsampled /4
                                       into a 24x512 buffer

That state management is where bugs hide, not the enqueue call. Port it
against the Python running side by side and diff the tensors.

FP32, not FP16 -- a TensorRT 10.16 gelu-fusion bug breaks the half-precision
build. FP32 runs at 42 Hz, which is fast enough that it costs nothing.


## 5. How to do it without losing a summer

Port layer by layer with the Python running as reference. Not big-bang.

1. Firmware first, on the bench, with the EXISTING Python host talking to it.
   If Python <-> new-firmware works, the firmware is right. This isolates one
   variable at a time, which is the thing that was missing every time this
   project chased the wrong layer.
2. Then swap the host side up one layer at a time, old path still available.
3. Diff behaviour against logged drives before trusting anything on the road.
4. Only then take it to the car, parked first, engine running, wheels chocked.


## 6. Rules this project learned the hard way

**A decoded value can never establish liveness -- only an arrival time can.**
Cost: set_speed_raw made a wrong conclusion unfalsifiable for weeks;
acc_active held the radar suppressed for 264 s with nothing able to clear it.

**Failure timing is diagnostic.** A queue that is too small fails at a
repeatable load. A race fails whenever the timing lines up. 11 minutes one
drive and 61 the next is a race, and that single observation is what redirected
the USB investigation after three wrong answers.

**Instrument before concluding.** Every wrong answer here came from inferring a
cause from damage instead of measuring it. rx_ovf, drain gap, the ID census and
the guard counters all exist because a conclusion was drawn without them.

**Make the test fail first.** If the old code passes your new test, the test
proves nothing.

**Never start the stack on a sleeping bus**, and never chain build -> flash.
Show the disassembly and hash, then ask.
