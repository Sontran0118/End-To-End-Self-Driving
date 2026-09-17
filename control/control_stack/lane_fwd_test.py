#!/usr/bin/env python3
"""REGRESSION TEST for the two Jetson-port CAM_LKAS/CAM_LANEINFO changes.

Runs against the REAL safety C code (opendbc libsafety), built twice: once stock
and once with -DPANDA_NUCLEO, so the port's deviation from upstream is explicit
and provable rather than asserted.

What it proves:
  1. FORWARDING  camera 0x440 CAM_LANEINFO now forwards cam->car on the port
     build, and is still blocked on the stock build.
  2. NO WEAKENING  camera 0x243 CAM_LKAS is still blocked cam->car (openpilot's
     steering frame is the only one the car ever sees), and relay-malfunction
     detection -- the check that catches a camera never cut off the main bus --
     still latches on BOTH 0x243 and 0x440 seen on bus 0.
  3. MODE GATING  forwarding only happens in a car safety mode. SAFETY_SILENT
     and SAFETY_NOOUTPUT set disable_forwarding, so in dashcam mode the lane
     frame does NOT reach the car, with or without this change.
  4. ANGLE  create_steering_control's new steering_angle argument is byte-for-byte
     a no-op at its default of 0, round-trips through the DBC bit layout, and
     clamps instead of wrapping.

Nothing is transmitted. No hardware needed.

  python3 lane_fwd_test.py
"""
import os
import subprocess
import sys
import tempfile
import types

OPENDBC = os.environ.get("OPENDBC_SRC", "/home/tran/opendbc_src")
sys.path.insert(0, OPENDBC)

from cffi import FFI  # noqa: E402

SAFETY_SILENT, SAFETY_NOOUTPUT, SAFETY_MAZDA = 0, 19, 13
LKAS, HUD = 0x243, 0x440

CDEF_PKT = """
typedef struct {
  unsigned char fd : 1;
  unsigned char bus : 3;
  unsigned char data_len_code : 4;
  unsigned char rejected : 1;
  unsigned char returned : 1;
  unsigned char extended : 1;
  unsigned int addr : 29;
  unsigned char checksum;
  unsigned char data[64];
} CANPacket_t;
"""
CDEF_API = """
bool safety_rx_hook(CANPacket_t *msg);
bool safety_tx_hook(CANPacket_t *msg);
int safety_fwd_hook(int bus_num, int addr);
int set_safety_hooks(uint16_t mode, uint16_t param);
void can_set_checksum(CANPacket_t *packet);
void init_tests(void);
void set_relay_malfunction(bool c);
bool get_relay_malfunction(void);
"""

results = []


def check(desc, got, want):
    ok = got == want
    results.append(ok)
    print("  [%s] %-58s want=%-6s got=%s" % ("PASS" if ok else "FAIL", desc, want, got))


def build(tmpdir, nucleo):
    """Compile libsafety from opendbc source, optionally as the Nucleo port."""
    out = os.path.join(tmpdir, "libsafety_%s.so" % ("nucleo" if nucleo else "stock"))
    cmd = ["gcc", "-shared", "-fPIC", "-Wall", "-Wextra", "-Werror", "-nostdlib",
           "-fno-builtin", "-std=gnu11", "-Wno-pointer-to-int-cast", "-DCANFD"]
    if nucleo:
        cmd.append("-DPANDA_NUCLEO")
    cmd += ["-I" + OPENDBC, "-I" + os.path.join(OPENDBC, "opendbc/safety/board"),
            "-o", out, os.path.join(OPENDBC, "opendbc/safety/tests/libsafety/safety.c")]
    subprocess.check_call(cmd)
    return out


def load(path):
    ffi = FFI()
    ffi.cdef(CDEF_PKT, packed=True)
    ffi.cdef(CDEF_API)
    return ffi, ffi.dlopen(path)


def pkt(ffi, lib, addr, bus, dat=b"\x00" * 8):
    p = ffi.new("CANPacket_t *")
    p[0].extended = 0
    p[0].addr = addr
    p[0].data_len_code = len(dat)
    p[0].bus = bus
    p[0].data = bytes(dat)
    lib.can_set_checksum(p)
    return p


def test_forwarding(tmpdir):
    print("safety_fwd_hook returns the destination bus, or -1 when blocked.")

    print("\n=== 1. STOCK opendbc: both camera frames blocked cam->car ===")
    _, lib = load(build(tmpdir, nucleo=False))
    assert lib.set_safety_hooks(SAFETY_MAZDA, 0) == 0
    check("stock: camera 0x243 cam->car BLOCKED", lib.safety_fwd_hook(2, LKAS), -1)
    check("stock: camera 0x440 cam->car BLOCKED", lib.safety_fwd_hook(2, HUD), -1)
    check("stock: car 0x202 car->cam forwarded", lib.safety_fwd_hook(0, 0x202), 2)

    print("\n=== 2. PANDA_NUCLEO: the lane frame now reaches the car ===")
    ffi, lib = load(build(tmpdir, nucleo=True))
    assert lib.set_safety_hooks(SAFETY_MAZDA, 0) == 0
    check("nucleo: camera 0x243 cam->car STILL BLOCKED", lib.safety_fwd_hook(2, LKAS), -1)
    check("nucleo: camera 0x440 cam->car FORWARDED to bus 0", lib.safety_fwd_hook(2, HUD), 0)
    check("nucleo: car 0x202 car->cam forwarded", lib.safety_fwd_hook(0, 0x202), 2)

    print("\n=== 3. relay-malfunction protection is NOT weakened ===")
    lib.init_tests()
    lib.set_relay_malfunction(False)
    check("relay_malfunction clear at start", bool(lib.get_relay_malfunction()), False)
    lib.safety_rx_hook(pkt(ffi, lib, HUD, 0))
    check("0x440 on bus 0 (uncut camera) latches it", bool(lib.get_relay_malfunction()), True)
    check("...forwarding then dies", lib.safety_fwd_hook(2, HUD), -1)
    check("...and tx dies", bool(lib.safety_tx_hook(pkt(ffi, lib, LKAS, 0))), False)
    lib.set_relay_malfunction(False)
    lib.safety_rx_hook(pkt(ffi, lib, LKAS, 0))
    check("0x243 on bus 0 latches it independently", bool(lib.get_relay_malfunction()), True)
    lib.set_relay_malfunction(False)
    lib.safety_rx_hook(pkt(ffi, lib, HUD, 2))
    check("0x440 on bus 2 (normal traffic) does NOT latch", bool(lib.get_relay_malfunction()), False)
    check("...and still forwards to the car", lib.safety_fwd_hook(2, HUD), 0)

    print("\n=== 4. forwarding only runs in a CAR safety mode ===")
    print("      (nooutput_init sets disable_forwarding -- dashcam mode forwards NOTHING)")
    for name, mode in (("SAFETY_SILENT", SAFETY_SILENT), ("SAFETY_NOOUTPUT", SAFETY_NOOUTPUT)):
        assert lib.set_safety_hooks(mode, 0) == 0
        lib.init_tests()
        check("%s: camera 0x440 cam->car blocked" % name, lib.safety_fwd_hook(2, HUD), -1)
        check("%s: car 0x202 car->cam blocked" % name, lib.safety_fwd_hook(0, 0x202), -1)


def test_angle():
    from opendbc.car.mazda.values import CAR
    from opendbc.car.mazda.interface import CarInterface
    from opendbc.can import CANPacker
    from opendbc.car.mazda import mazdacan

    # the ORIGINAL mazdacan.py straight out of git, as a separate module
    orig_src = subprocess.check_output(
        ["git", "-C", OPENDBC, "show", "HEAD:opendbc/car/mazda/mazdacan.py"]).decode()
    orig = types.ModuleType("mazdacan_orig")
    exec(compile(orig_src, "<git HEAD:mazdacan.py>", "exec"), orig.__dict__)

    CP = CarInterface.get_non_essential_params(CAR.MAZDA_CX5_2022)
    packer = CANPacker("mazda_2017")

    def decode_angle(d):
        # STEERING_ANGLE: DBC 33|12@0+ -> byte4 bits 1..0, byte5, byte6 bits 7..6
        return (((d[4] & 0x03) << 10) | (d[5] << 2) | ((d[6] >> 6) & 0x03)) - 2048

    print("\n=== 5. steering_angle default is byte-identical to upstream ===")
    n = diff = 0
    for frame in range(16):
        for torque in (-800, -437, -1, 0, 1, 250, 800):
            for b1 in (0, 1):
                for e1 in (0, 1):
                    for e2 in (0, 1):
                        bits = {"BIT_1": b1, "ERR_BIT_1": e1, "ERR_BIT_2": e2}
                        a = mazdacan.create_steering_control(packer, CP, frame, torque, bits)
                        b = orig.create_steering_control(packer, CP, frame, torque, bits)
                        n += 1
                        diff += bytes(a[1]) != bytes(b[1])
    check("%d frames swept, byte-level differences" % n, diff, 0)

    print("\n=== 6. a non-zero angle packs correctly and clamps ===")
    bits = {"BIT_1": 1, "ERR_BIT_1": 0, "ERR_BIT_2": 0}
    bad = [(a, decode_angle(bytes(mazdacan.create_steering_control(packer, CP, 3, 100, bits, a)[1])))
           for a in (-2048, -1024, -1023, -1, 0, 1, 1023, 1024, 2047)]
    bad = [(w, g) for w, g in bad if w != g]
    check("9 angles round-trip exactly %s" % (bad or ""), bad, [])
    for ang, want in ((5000, 2047), (-5000, -2048), (2048, 2047), (-2049, -2048)):
        got = decode_angle(bytes(mazdacan.create_steering_control(packer, CP, 0, 0, bits, ang)[1]))
        check("angle %6d clamps (not wraps)" % ang, got, want)

    z = bytes(mazdacan.create_steering_control(packer, CP, 0, 100, bits, 0)[1])
    nz = bytes(mazdacan.create_steering_control(packer, CP, 0, 100, bits, 400)[1])
    check("angle 0 vs 400 give different payloads", z != nz, True)
    check("...and different checksums", z[7] != nz[7], True)
    dt = lambda d: (((d[0] & 0x0F) << 8) | d[1]) - 2048  # noqa: E731
    check("torque field untouched by the angle", (dt(z), dt(nz)), (100, 100))

    print("\n=== 7. model curvature -> steering angle inverts controlsd exactly ===")
    import math
    from opendbc.car.vehicle_model import VehicleModel
    VM = VehicleModel(CP)
    worst = 0.0
    for a_deg in (-180.0, -90.0, -10.0, 0.0, 10.0, 90.0, 180.0):
        for v in (0.0, 8.33, 22.2, 33.3):
            # controlsd: self.curvature = -VM.calc_curvature(sa, v, roll)
            curv = -VM.calc_curvature(math.radians(a_deg), v, 0.0)
            back = math.degrees(VM.get_steer_from_curvature(-curv, v, 0.0))
            worst = max(worst, abs(back - a_deg))
    check("worst round-trip error over 28 cases < 1e-6 deg (%.2e)" % worst, worst < 1e-6, True)


def main():
    with tempfile.TemporaryDirectory() as tmpdir:
        test_forwarding(tmpdir)
    test_angle()
    print("\n%d/%d checks passed" % (sum(results), len(results)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
