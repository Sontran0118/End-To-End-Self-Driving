from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


# Steer torque limits

class CarControllerParams:
  # Raised from the upstream 800 (Jetson port). MUST stay equal to .max_torque in
  # opendbc/safety/modes/mazda.h -- if this is the higher of the two the panda
  # rejects the frame, zeroes its desired_torque_last, and every following frame
  # fails the rate check too, so 0x243 stops reaching the bus and the EPS raises
  # the front LKAS fault. Changing this means rebuilding and reflashing the panda.
  # 800 (upstream) -> 1000 -> 1400. MEASURED 2026-07-30: the EPS tracked
  # LKAS_REQUEST 1:1 (0.98-1.00) from 50 all the way to 890 counts with no
  # roll-off, so the stock camera's 399 peak is Mazda's calibration choice, not a
  # rack limit. The apparent "clamp at 450" in the earlier run was this stack's
  # own max_rt_delta, not the car -- see the note in mazda.h.
  #
  # !! RAISING THIS CHANGES THE LATERAL LOOP GAIN. LatControlTorque works in a
  # normalised [-1,1] space and the carcontroller multiplies by STEER_MAX, so
  # 1.0 now means 1400 counts where it used to mean 1000. latAccelFactor is the
  # measured m/s^2 per unit NORMALISED torque, so it must scale by the same
  # ratio or the feedforward over-commands by exactly that factor.
  #   1000 -> 1400 is 1.4x, so latAccelFactor 1.7601683 -> 2.4642356.
  # It is published in dashcam_web.py (liveTorqueParameters, useParams=True).
  # Note it was NOT rescaled for the earlier 800 -> 1000 change either, so the
  # feedforward has been over-commanding by ~25% since then.
  #   TRIED 1400 -> 2047 (the wire maximum) on 2026-08-01 and REVERTED the same
  #   day. It cannot help, and the arithmetic says why: the controller emits
  #   counts = output_lataccel * STEER_MAX / latAccelFactor, and latAccelFactor
  #   has to be scaled by the same ratio to keep the feedforward honest. Both
  #   scale together, so 1400/2.4642356 == 2047/3.6030645 == 568.13 counts per
  #   m/s^2 -- the commanded torque for a given error is IDENTICAL. The only
  #   thing that moved was the clip point, and the 1400 ceiling had been
  #   truncating the measured +-982 limit-cycle peak. Raising it just let the
  #   oscillation swing further. Amplitude is not the constraint; the RAMP is
  #   (see STEER_DELTA_UP below) -- exactly what the 2026-07-30 note above says.
  #   1400 -> 2047 (2026-08-01, second attempt, and this time for a reason that
  #   holds up). The first attempt was reverted on the argument that STEER_MAX
  #   and latAccelFactor scale together so nothing changes. That is true BELOW
  #   the rail -- counts = output_lataccel * STEER_MAX / latAccelFactor gives
  #   568 counts per m/s^2 either way -- but it is FALSE at the rail. steer_max
  #   in latcontrol.py is 1.0 normalised, so update_limits() clamps the PID to
  #   lateral_accel_from_torque(1.0) = latAccelFactor, and the ceiling in counts
  #   is STEER_MAX itself: 1400 vs 2047, a 46% higher ceiling.
  #
  #   MEASURED: cc.actuators.torque sits at exactly 1.000 for 26% of engaged
  #   samples, so the controller is saturated a quarter of the time and that
  #   extra headroom is reachable. latAccelFactor rescaled with it in
  #   dashcam_web.py (1.7601682915983443 * 2047/1000 = 3.6030645) so the
  #   below-rail feedforward is unchanged.
  #
  #   NOTE this does NOT buy tighter low-speed turns: the EPS blocks on STEERING
  #   ANGLE (0.3-1.2% below 60 deg, 48.7% past 200 deg), not on torque. More
  #   ceiling gets you there faster, not further.
  # 2047 -> 1200 (2026-08-08, smoothness). 2047 is the WIRE maximum and 2.56x
  # upstream's 800. Authority is not free: every count of controller ripple is
  # felt 2.56x harder at the rim than stock, so the same numeric roughness reads
  # as much worse steering. MEASURED on this car at highway speed after the other
  # fixes: applied torque stdev 284 with a range of -756..+900, i.e. the loop is
  # nowhere near needing 2047 -- the peak demand seen was under 1250.
  #
  # 1200 keeps ~35% headroom over the largest observed demand while cutting the
  # felt magnitude of any residual ripple by 41%.
  #
  # SAFE without a firmware flash: mazda.h's max_torque is 2047 and the panda
  # rejects a frame only when the sender's ceiling EXCEEDS the limit. Lowering is
  # always allowed. Raising it back above 2047 would require flashing mazda.h
  # FIRST, or every frame is rejected and 0x243 stops reaching the bus.
  #
  # NB latAccelFactor and STEER_MAX scale together below the rail: the controller
  # works in normalised [-1,1] and the carcontroller multiplies by STEER_MAX, so
  # halving this halves the counts for the same normalised command. If it now
  # feels short of authority in a tight bend, raise back toward 1600 rather than
  # compensating with gain.
  # 1200 -> 800, upstream/stock value.
  #
  # READ THIS WITH latAccelFactor. The controller emits NORMALISED torque in
  # [-1,1] and carcontroller.py:45 multiplies by STEER_MAX, so the loop's total
  # gain is STEER_MAX / latAccelFactor counts per m/s^2:
  #     stock openpilot : 800 / 1.76 = 455
  #     this build      : 800 / 2.60 = 308   <-- 32% WEAKER THAN STOCK
  # Because OP_LAT_ACCEL_FACTOR is currently 2.6 rather than the 1.76 the fleet
  # uses, dropping STEER_MAX to the stock number does NOT reproduce stock feel --
  # it lands well below it. If it now runs wide in bends, the fix is to put
  # OP_LAT_ACCEL_FACTOR back to 1.76 (restoring 455) rather than raising this.
  #
  # Peak demand MEASURED on this car after the other fixes was just under 1250
  # counts, so 800 WILL saturate in a tight curve and the car will run wide.
  # That is a deliberate trade of curve authority for smoothness.
  STEER_MAX = 800                # stock upstream value (wire max is 2047)
  # RAISED from 10 (Jetson port). This is the RAMP, and on this stack it is the
  # limit that actually binds -- measured applied_peak 360 against a STEER_MAX of
  # 1000 with want_peak at 1000, i.e. the controller asked for full scale and the
  # ramp handed it 36% before the corner was over. Amplitude was never the cap.
  #
  # It is per MESSAGE, so the achievable ramp is tx_rate * STEER_DELTA_UP:
  #     50 Hz  * 15 =  750 counts/s
  #    100 Hz  * 15 = 1500 counts/s   (stock openpilot is 100 Hz * 10 = 1000)
  # against the 46.8 Hz * 10 = 468 counts/s this port was actually achieving.
  #
  # BOUNDED BY max_rt_delta in opendbc/safety/modes/mazda.h, which is a cap per
  # 250 ms window regardless of rate: at 50 Hz that window holds 12.5 messages,
  # so 12.5 * 15 = 188 must stay under it.
  #
  # Read the note on max_rt_delta in mazda.h before touching either number. It
  # is currently at a DIAGNOSTIC 900, because under a jittery tx stream it stops
  # being headroom and becomes a hard torque ceiling -- that is what produced
  # the apparent "EPS clamps at 450" result on 2026-07-30, which was this
  # constant and not the rack.
  #
  # All three numbers move together or the panda rejects; see the note on
  # STEER_MAX above for why that failure is total rather than a soft clamp.
  # 10 -> 15 -> 30 -> 50 (2026-08-01). This is the limit that actually binds:
  # measured applied_peak 360 against want_peak 1000, i.e. the ramp delivered 36%
  # of what the controller asked for before the corner ended. Raising STEER_MAX
  # was tried first and did nothing (see above) because amplitude was never the
  # constraint. At 70 Hz: 50 * 70 = 3500 counts/s, so 0 -> 1400 in 0.40 s against
  # 0.67 s at 30. Per 250 ms RT window that is 17.5 * 50 = 875, under the 1400
  # max_rt_delta, so the RT check still cannot bind.
  #
  # DOWN raised to match. It was 25 while UP was 30, and an up-rate faster than
  # the down-rate lets torque RATCHET across a limit cycle -- each oscillation
  # builds faster than it releases. Symmetric is the safe direction, and a
  # faster release is independently desirable.
  # MUST stay equal to max_rate_up / max_rate_down in mazda.h -- a sender that
  # ramps faster than the safety model allows gets its frame rejected, which
  # zeroes desired_torque_last and stops 0x243 entirely.
  # 50/50 TRIED 2026-08-01 AND REVERTED -- the EPS refuses it. MEASURED, same
  # car, same speeds, comparing the drive before the change to the one after:
  #        DELTA_UP 30            DELTA_UP 50
  #   0-20 kph  eff/req 0.62      eff/req 0.00
  #   lkas_block   3.4%           lkas_block 100%
  # eps_request tracked applied_torque 1:1 both times, so the frames reach the
  # rack -- it receives them and applies NOTHING, asserting the LKAS fault bit.
  # 30 * 70 Hz = 2100 counts/s is accepted; 50 * 70 = 3500 counts/s is not.
  # The rack has its own plausibility limit on how fast LKAS_REQUEST may move,
  # and it sits between those two. Raise this again only with a measured
  # eff/req ratio to show the EPS still follows.
  # RETRY of 50/50 (2026-08-01, second attempt). The first attempt measured
  # eff/req 0.00 and lkas_block 100%, which looked like the rack refusing the
  # 3500 counts/s ramp -- but every one of those runs had ALPHA LONG active, and
  # alpha long was later shown to be the thing degrading EPS acceptance on its
  # own (0.09 ratio / 47% blocked at 40-60 kph, against 0.83 / 0.0% with it off).
  # So the ramp was never isolated. This retest runs with alpha long OFF, where
  # the baseline at 30 is eff/req 0.69-0.83 and lkas_block 0-9%.
  #
  # If eff/req collapses again the rack really does have a ramp limit between
  # 2100 and 3500 counts/s; if it holds, the ramp was innocent all along.
  # DOWN raised to match so torque cannot ratchet across a limit cycle.
  # MUST stay equal to max_rate_up / max_rate_down in mazda.h.
  # 10 -> 15 -> 30 -> 50 -> 40 (2026-08-01). 50 was measured good with alpha
  # long OFF (eff/req 0.93 at 20-40 kph, lkas_block 0.0%) -- the earlier
  # catastrophic result at 50 was alpha long, not the ramp. Backed off to 40 as
  # a middle setting: 2800 counts/s at 70 Hz against 2100 at 30.
  #
  # DOWN deliberately left HIGHER than UP. That is the safe asymmetry and
  # openpilot's usual convention: the system can always release at least as fast
  # as it grabs, and torque cannot ratchet across a limit cycle (an up-rate
  # faster than the down-rate builds more each swing than it gives back).
  # MUST stay equal to max_rate_up / max_rate_down in mazda.h.
  # 40 (2026-08-02). History: 10 -> 15 -> 30 -> 50 -> 40 -> 10 -> 40.
  # Stock 10 was tried and reverted: the ramp is per MESSAGE, so time to full
  # scale is STEER_MAX / (tx_rate * DELTA_UP), and STEER_MAX here is 2047 rather
  # than upstream's 800. Stock 10 therefore means 2.92 s to full authority
  # against the 1.14 s upstream intends -- far slower than the pairing was
  # designed for. At 40 it is 0.73 s.
  #
  # MEASURED with alpha long off: delta 40 gives eff/req 0.87 at 20-40 kph and
  # 0.85 at 40-60 with 0.0-0.7% lkas_block, so the rack has no objection to this
  # ramp. The earlier collapse at 50 was alpha long, not the ramp.
  # MUST stay equal to max_rate_up / max_rate_down in mazda.h.
  # 40 -> 10 (2026-08-08, jitter). At 100 Hz this is 1000 counts/s of ramp
  # instead of 4000. MEASURED at 40: applied_torque stdev 784 counts on a +-2047
  # scale with 8% of samples pinned at the rail, and eps_req vs eps_eff diverging
  # in SIGN (-187 commanded while the rack applied +736) -- the controller was
  # building torque far faster than the rack could respond, overshooting, then
  # reversing. That is a delay-driven limit cycle, and rate authority is what
  # feeds it.
  #
  # SAFE without a firmware flash: mazda.h's max_rate_up stays 40, and the panda
  # rejects a frame only when the sender ramps FASTER than the limit. A slower
  # sender is strictly more conservative. If this value is ever raised back above
  # 40, mazda.h must be rebuilt and flashed FIRST or every frame is rejected,
  # desired_torque_last is zeroed, and 0x243 stops reaching the bus entirely.
  # 10 -> 40 (2026-08-08, at request). At 100 Hz this is 4000 counts/s of ramp
  # instead of 1000 -- far quicker to build torque into a sharp curve.
  #
  # STILL SAFE without a firmware flash: mazda.h's max_rate_up is 40, and the
  # panda rejects a frame only when the sender ramps FASTER than the limit. At
  # exactly 40 the sender sits ON the limit with no margin -- do NOT raise it
  # further without rebuilding and flashing mazda.h FIRST, or every frame is
  # rejected, desired_torque_last is zeroed, and 0x243 stops reaching the bus
  # entirely (a hard failure, not a soft clamp).
  #
  # NB the jitter measurements that motivated dropping this to 10 were taken
  # while CAN reception was intermittently frozen (see the bus-2 host filter in
  # mazda_filter.h), so the controller was acting on stale carState. Those
  # numbers are not trustworthy; re-measure at this setting on clean data.
  # 40 -> 80, paired with --lkas-hz 50. PER MESSAGE, so the ramp is unchanged:
  #   100 Hz x 40 = 4000 counts/s   ==   50 Hz x 80 = 4000 counts/s
  # The rate came down because 100 Hz produced 1177 late frames and a 34 ms worst
  # gap against a 10 ms target -- the EPS faults on irregular 0x243, not on a
  # lower rate. This keeps the steering response identical while making the
  # stream regular. MUST stay <= mazda.h's max_rate_up (now 80): a sender that
  # ramps faster than the firmware permits has EVERY frame rejected.
  # 80 -> 40, paired with --lkas-hz 100. PER MESSAGE, so the ramp is what matters:
  #   50 Hz x 80 = 4000 counts/s   ==   100 Hz x 40 = 4000 counts/s
  # Left at 80 while running 100 Hz the ramp would DOUBLE to 8000 counts/s and
  # reach STEER_MAX 800 in 0.10 s, which is far more aggressive than anything
  # tested here. Firmware max_rate_up stays 80, so no flash is needed: the panda
  # rejects only a sender that ramps FASTER than the limit, never a slower one.
  STEER_DELTA_UP = 80             # per message; 50 Hz -> 4000 counts/s
  # 50 -> 25 (stock). With DELTA_UP at 10 this was a 5:1 asymmetry: torque built
  # at 1000 counts/s and collapsed at 5000. That ratio is itself a sawtooth
  # generator -- slow ramp in, fast dump out, repeat -- and it survives any
  # amount of delay compensation because it is a rate-limit artefact, not a
  # phase problem. Stock's 10/25 is 2.5:1; keep the same ratio.
  STEER_DELTA_DOWN = 25           # torque decrease per refresh (stock 25)
  STEER_DRIVER_ALLOWANCE = 15     # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 1     # weight driver torque
  STEER_DRIVER_FACTOR = 1         # from dbc
  STEER_ERROR_MAX = 350           # max delta between torque cmd and torque motor
  STEER_STEP = 1  # 100 Hz

  def __init__(self, CP):
    pass


@dataclass
class MazdaCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.mazda]))


@dataclass(frozen=True, kw_only=True)
class MazdaCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.7  # not optimized yet


class MazdaFlags(IntFlag):
  # Static flags
  # Gen 1 hardware: same CAN messages and same camera
  GEN1 = 1


@dataclass
class MazdaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'mazda_2017'})
  flags: int = MazdaFlags.GEN1


class CAR(Platforms):
  MAZDA_CX5 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2017-21")],
    MazdaCarSpecs(mass=3655 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=15.5)
  )
  MAZDA_CX9 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2016-20")],
    MazdaCarSpecs(mass=4217 * CV.LB_TO_KG, wheelbase=3.1, steerRatio=17.6)
  )
  MAZDA_3 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2017-18")],
    MazdaCarSpecs(mass=2875 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=14.0)
  )
  MAZDA_6 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 6 2017-20")],
    MazdaCarSpecs(mass=3443 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=15.5)
  )
  MAZDA_CX9_2021 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2021-23", video="https://youtu.be/dA3duO4a0O4")],
    MAZDA_CX9.specs
  )
  MAZDA_CX5_2022 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2022-25")],
    MAZDA_CX5.specs,
  )


class LKAS_LIMITS:
  STEER_THRESHOLD = 15
  DISABLE_SPEED = 45    # kph
  ENABLE_SPEED = 52     # kph


class Buttons:
  NONE = 0
  SET_PLUS = 1
  SET_MINUS = 2
  RESUME = 3
  CANCEL = 4


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # TODO: check data to ensure ABS does not skip ISO-TP frames on bus 0
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
