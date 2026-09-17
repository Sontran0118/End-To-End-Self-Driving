"""BNO085 (GY-BNO08x) driver over I2C for the Jetson Orin Nano.

Talks SHTP directly over /dev/i2c-N rather than going through a vendor library.
The Arduino-side Adafruit library was found to silently deliver zero events on
this module, and the CircuitPython stack needs Blinka, so a ~200 line native
driver is both smaller and easier to debug than either.

WIRING (Orin Nano 40-pin -> GY-BNO08x):
    pin 1  3V3  -> VIN     (3.3 V ONLY, the part is not 5 V tolerant)
    pin 3  SDA  -> SDA
    pin 5  SCL  -> SCL
    pin 9  GND  -> GND

ADDRESS: this board answers at 0x4B, NOT the 0x4A that nearly every example
hardcodes -- its ADR pin is strapped high. Verified with `i2cdetect -y -r 7`.

THE THING THAT MAKES THIS CHIP LOOK BROKEN: on power-up it emits a ~276 byte
advertisement packet on channel 0 and then goes quiet. It sends NO sensor data
until the host explicitly enables each report with a Set Feature command. A
driver that connects, reads the advertisement, and waits will sit there forever
receiving nothing -- which reads as a dead sensor rather than an unconfigured
one.

I2C QUIRK: each read transaction restarts the packet from its header. You cannot
read 4 header bytes, then continue reading the payload in a second transaction --
the second read returns the header again. So the length is read first, then the
whole packet (header included) is re-read in one transaction.
"""

from __future__ import annotations

import fcntl
import os
import struct
import time
from dataclasses import dataclass

I2C_SLAVE = 0x0703

# SHTP channels
CH_COMMAND = 0
CH_EXECUTABLE = 1
CH_CONTROL = 2
CH_REPORTS = 3      # sensor input reports arrive here
CH_WAKE = 4
CH_GYRO = 5

# control-channel report IDs
SET_FEATURE_COMMAND = 0xFD
PRODUCT_ID_REQUEST = 0xF9
PRODUCT_ID_RESPONSE = 0xF8

# sensor (feature) report IDs
RPT_ACCELEROMETER = 0x01
RPT_GYROSCOPE = 0x02
RPT_MAGNETOMETER = 0x03
RPT_LINEAR_ACCEL = 0x04
RPT_ROTATION_VECTOR = 0x05
RPT_GAME_ROTATION = 0x08

RPT_NAMES = {
    RPT_ACCELEROMETER: "accelerometer",
    RPT_GYROSCOPE: "gyroscope",
    RPT_MAGNETOMETER: "magnetometer",
    RPT_LINEAR_ACCEL: "linear_accel",
    RPT_ROTATION_VECTOR: "rotation_vector",
    RPT_GAME_ROTATION: "game_rotation",
}

# Fixed-point scaling. The BNO085 sends signed 16-bit integers with an implied
# binary point ("Q point") that differs per report; these come from the BNO08x
# datasheet section 6.5. Getting one wrong yields plausible-looking numbers off
# by a power of two, which is easy to miss -- gravity reading 4.9 instead of 9.8.
Q_ACCEL = 8       # m/s^2
Q_GYRO = 9        # rad/s
Q_MAG = 4         # uT
Q_QUAT = 14       # unit quaternion
Q_ACCURACY = 12   # radians

ACCURACY_NAMES = {0: "unreliable", 1: "low", 2: "medium", 3: "high"}


@dataclass
class Reading:
    """One decoded sensor report."""
    kind: str
    values: tuple
    accuracy: int
    t_mono: float

    def __str__(self) -> str:
        v = "  ".join(f"{x:+8.3f}" for x in self.values)
        return f"{self.kind:16s} {v}   [{ACCURACY_NAMES.get(self.accuracy, '?')}]"


class BNO085:
    def __init__(self, bus: int = 7, addr: int = 0x4B):
        self.bus, self.addr = bus, addr
        self._fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        fcntl.ioctl(self._fd, I2C_SLAVE, addr)
        self._seq = {}          # per-channel outgoing sequence numbers
        self.product_id = None

    # ---- raw SHTP transport -------------------------------------------------

    def _write_packet(self, channel: int, payload: bytes) -> None:
        seq = self._seq.get(channel, 0)
        self._seq[channel] = (seq + 1) & 0xFF
        length = len(payload) + 4
        hdr = bytes([length & 0xFF, (length >> 8) & 0xFF, channel, seq])
        os.write(self._fd, hdr + payload)

    def _read_packet(self, max_len: int = 512) -> tuple[int, bytes]:
        """Return (channel, payload). Empty payload means nothing queued."""
        try:
            hdr = os.read(self._fd, 4)
        except OSError:
            return -1, b""
        if len(hdr) < 4:
            return -1, b""
        length = (hdr[0] | (hdr[1] << 8)) & 0x7FFF   # bit 15 = continuation
        if length <= 4:
            return hdr[2], b""
        # Re-read the whole packet: a fresh transaction restarts at the header,
        # so we cannot simply continue from where the 4-byte read stopped.
        n = min(length, max_len)
        try:
            full = os.read(self._fd, n)
        except OSError:
            return -1, b""
        if len(full) < 4:
            return -1, b""
        return full[2], full[4:]

    def _drain(self, seconds: float = 0.5) -> int:
        """Consume the power-on advertisement so it does not confuse parsing."""
        end, n = time.monotonic() + seconds, 0
        while time.monotonic() < end:
            ch, payload = self._read_packet()
            if not payload:
                time.sleep(0.01)
                continue
            n += 1
        return n

    # ---- configuration ------------------------------------------------------

    def enable_report(self, report_id: int, interval_ms: int = 10) -> None:
        """Enable one sensor report. Nothing is streamed until this is called."""
        interval_us = int(interval_ms * 1000)
        payload = struct.pack(
            "<BBBHIII",
            SET_FEATURE_COMMAND,
            report_id,
            0,            # feature flags
            0,            # change sensitivity
            interval_us,  # report interval, microseconds
            0,            # batch interval
            0,            # sensor-specific config
        )
        self._write_packet(CH_CONTROL, payload)
        time.sleep(0.05)

    def disable_report(self, report_id: int) -> None:
        """Stop a report. Interval 0 means 'never', which is how SHTP disables.

        Needed because the BNO085 REMEMBERS enabled reports across close() and
        even across power cycles -- a report switched on by an earlier program
        keeps streaming into the next one, wasting bus bandwidth and, worse,
        showing up in a caller that never asked for it.
        """
        self.enable_report(report_id, interval_ms=0)

    def read_product_id(self) -> bool:
        self._write_packet(CH_CONTROL, bytes([PRODUCT_ID_REQUEST, 0]))
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            ch, payload = self._read_packet()
            if payload and payload[0] == PRODUCT_ID_RESPONSE and len(payload) >= 15:
                sw_major, sw_minor = payload[2], payload[3]
                sw_patch = struct.unpack_from("<H", payload, 12)[0]
                self.product_id = f"SW {sw_major}.{sw_minor}.{sw_patch}"
                return True
            time.sleep(0.005)
        return False

    # ---- reading ------------------------------------------------------------

    def poll(self) -> list[Reading]:
        """Read one SHTP packet and decode any sensor reports inside it."""
        ch, payload = self._read_packet()
        if ch != CH_REPORTS or len(payload) < 6:
            return []
        # Input reports start with a 5-byte timebase header (0xFB + int32 delta).
        if payload[0] == 0xFB:
            payload = payload[5:]
        out, i, now = [], 0, time.monotonic()
        while i + 4 <= len(payload):
            rid = payload[i]
            if rid in (RPT_ACCELEROMETER, RPT_GYROSCOPE, RPT_MAGNETOMETER,
                       RPT_LINEAR_ACCEL):
                if i + 10 > len(payload):
                    break
                status = payload[i + 2] & 0x03
                x, y, z = struct.unpack_from("<hhh", payload, i + 4)
                q = {RPT_ACCELEROMETER: Q_ACCEL, RPT_LINEAR_ACCEL: Q_ACCEL,
                     RPT_GYROSCOPE: Q_GYRO, RPT_MAGNETOMETER: Q_MAG}[rid]
                s = float(1 << q)
                out.append(Reading(RPT_NAMES[rid], (x / s, y / s, z / s), status, now))
                i += 10
            elif rid in (RPT_ROTATION_VECTOR, RPT_GAME_ROTATION):
                need = 14 if rid == RPT_ROTATION_VECTOR else 12
                if i + need > len(payload):
                    break
                status = payload[i + 2] & 0x03
                qi, qj, qk, qr = struct.unpack_from("<hhhh", payload, i + 4)
                s = float(1 << Q_QUAT)
                vals = (qi / s, qj / s, qk / s, qr / s)
                out.append(Reading(RPT_NAMES[rid], vals, status, now))
                i += need
            else:
                break   # unknown report: cannot know its length, so stop
        return out

    def close(self) -> None:
        os.close(self._fd)


def quat_to_euler(qi: float, qj: float, qk: float, qr: float) -> tuple[float, float, float]:
    """Quaternion -> (roll, pitch, yaw) in degrees."""
    import math
    sinr_cosp = 2 * (qr * qi + qj * qk)
    cosr_cosp = 1 - 2 * (qi * qi + qj * qj)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (qr * qj - qk * qi)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2 * (qr * qk + qi * qj)
    cosy_cosp = 1 - 2 * (qj * qj + qk * qk)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


if __name__ == "__main__":
    import sys
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0

    imu = BNO085(bus=7, addr=0x4B)
    print(f"opened /dev/i2c-{imu.bus} @ 0x{imu.addr:02x}")
    print(f"drained {imu._drain(0.6)} startup packet(s)")
    print(f"product id: {imu.product_id if imu.read_product_id() else 'no response'}")

    # GAME rotation vector, not the plain one. The plain RPT_ROTATION_VECTOR
    # fuses the magnetometer to get an absolute (magnetic-north) yaw, and in a
    # car that is worse than useless: the body is a steel box full of switching
    # currents, so the mag never calibrates. Measured here, side by side: game
    # rotation reports accuracy "high" immediately while the mag-fused one sits
    # at "unreliable", and their roll/pitch agree to 0.01 deg -- so the mag buys
    # nothing but an unusable confidence flag.
    #
    # The tradeoff is that game rotation's yaw has an arbitrary origin (whatever
    # heading it booted at) rather than pointing at north. That is fine here:
    # absolute heading comes from GPS and vision, and what this sensor is for is
    # roll/pitch and yaw RATE.
    # Turn off anything a previous run left enabled (the chip remembers), so the
    # rates below reflect what THIS program asked for.
    for rid in (RPT_ROTATION_VECTOR, RPT_MAGNETOMETER, RPT_LINEAR_ACCEL):
        imu.disable_report(rid)

    for rid in (RPT_ACCELEROMETER, RPT_GYROSCOPE, RPT_GAME_ROTATION):
        imu.enable_report(rid, interval_ms=20)   # 50 Hz
        print(f"enabled {RPT_NAMES[rid]}")

    print(f"\nreading for {seconds:.0f}s ...\n")
    counts, last, end = {}, {}, time.monotonic() + seconds
    while time.monotonic() < end:
        for r in imu.poll():
            counts[r.kind] = counts.get(r.kind, 0) + 1
            last[r.kind] = r
        time.sleep(0.002)

    print("rates:")
    for k, n in sorted(counts.items()):
        print(f"  {k:16s} {n:5d} samples  {n/seconds:6.1f} Hz")
    print("\nlast values:")
    for k, r in sorted(last.items()):
        print(f"  {r}")
    if RPT_NAMES[RPT_GAME_ROTATION] in last:
        q = last[RPT_NAMES[RPT_GAME_ROTATION]].values
        roll, pitch, yaw = quat_to_euler(*q)
        print(f"\n  orientation: roll {roll:+7.2f}  pitch {pitch:+7.2f}  yaw {yaw:+7.2f}  (degrees)")
    imu.close()
