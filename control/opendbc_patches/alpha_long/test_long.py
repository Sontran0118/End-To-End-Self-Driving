"""Load the vendored longitudinal.py by path against opendbc 8ddffb37.

Checks, in order of what fails first:
  1. every import it needs exists at the pinned version
  2. every DBC signal it patches exists in mazda_2017.dbc
  3. the neutral command really is neutral, and the safety hook's unpacking of
     ACCEL_CMD agrees with what the packer produced
"""
import importlib.util
import sys

sys.path.insert(0, "/home/tran/opendbc_src")

OVERLAY = ("/home/tran/op_fork/jetson_port/opendbc_patches/alpha_long/overlay"
           "/opendbc/car/mazda/longitudinal.py")

spec = importlib.util.spec_from_file_location("mazda_longitudinal", OVERLAY)
mod = importlib.util.module_from_spec(spec)
try:
    spec.loader.exec_module(mod)
    print("1. imports OK -- every dependency exists at 8ddffb37")
except Exception as e:
    print(f"1. IMPORT FAILED: {type(e).__name__}: {e}")
    raise SystemExit(1)

# --- 2. which DBC signals are missing? -------------------------------------
need = {
    "CRZ_INFO": ["ACCEL_CMD", "ACC_ACTIVE", "ACC_SET_ALLOWED", "CRZ_ENDED",
                 "STOPPING_MAYBE", "STOPPING_MAYBE2", "RESUME_UNLATCHING_MAYBE", "CTR1"],
    "CRZ_CTRL": ["CRZ_ACTIVE", "ACC_ACTIVE_2", "DISABLE_TIMER_1", "DISABLE_TIMER_2",
                 "RADAR_HAS_LEAD", "RADAR_LEAD_RELATIVE_DISTANCE", "ACC_GAS_MAYBE2"],
}
missing = []
for msg, sigs in need.items():
    have = mod.MAZDA_LONG_DBC.name_to_msg[msg].sigs
    for s in sigs:
        if s not in have:
            missing.append(f"{msg}.{s}")
print(f"2. DBC signals missing: {len(missing)}")
for m in missing:
    print(f"     {m}")
if missing:
    print("   -> the mazda_2017.dbc patch is REQUIRED; these are what it adds.")
    raise SystemExit(2)

# --- 3. neutral command + round-trip against the safety hook ---------------
raw = mod.build_crz_info(0.0, 0, False, False, 0.0)
print(f"3. neutral CRZ_INFO: {raw.hex()}")


def safety_unpack(d):
    """Exactly what mazda.h mazda_tx_hook does."""
    accel_raw = (((d[2] & 0x3) << 11) | (d[3] << 3) | (d[4] >> 5))
    return accel_raw - 4096


print(f"   safety hook reads accel = {safety_unpack(raw)} (want 0)")
ok = safety_unpack(raw) == 0

for accel, v in ((1.0, 10.0), (-1.0, 10.0), (2.5, 20.0), (-3.0, 5.0)):
    r = mod.build_crz_info(accel, 3, True, False, v)
    want = mod.accel_to_accel_cmd(accel, v)
    got = safety_unpack(r)
    flag = "OK " if got == want else "BAD"
    print(f"   {flag} accel={accel:+.1f} v={v:4.1f} -> packed {want:+5d}, "
          f"safety reads {got:+5d}")
    ok &= got == want

# clip must stay inside the panda's +-2000 window
for accel in (10.0, -10.0):
    r = mod.build_crz_info(accel, 0, True, False, 0.0)
    got = safety_unpack(r)
    inside = -2000 <= got <= 2000
    print(f"   {'OK ' if inside else 'BAD'} saturated accel={accel:+.1f} -> "
          f"{got:+5d} (panda window +-2000)")
    ok &= inside

print("\nALL CHECKS PASS" if ok else "\nFAILURES ABOVE")
raise SystemExit(0 if ok else 3)
