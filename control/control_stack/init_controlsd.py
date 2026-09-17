#!/usr/bin/env python3
"""Bring controlsd to life: generate CX-5 CarParams, write to the params store,
instantiate Controls. This proves the full control stack initializes for the
Mazda with a real car interface. No car, no transmit — pure init test."""
import os
os.environ.setdefault("PARAMS_ROOT", "/tmp/op_params")

from openpilot.common.params import Params
import openpilot.cereal.messaging as messaging
from opendbc.car.structs import car as car_struct
from opendbc.car.mazda.values import CAR
from opendbc.car.mazda.interface import CarInterface

# 1. build CarParams for the CX-5 2022+ (our car)
fingerprint = CAR.MAZDA_CX5_2022
CP = CarInterface.get_non_essential_params(fingerprint)
print("CarParams built for:", CP.carFingerprint)
print("  steerRatio=%.2f wheelbase=%.2f mass=%.0f" %
      (CP.steerRatio, CP.wheelbase, CP.mass))
print("  safetyModel:", CP.safetyConfigs[0].safetyModel if CP.safetyConfigs else "none")

# 2. write CarParams into the params store (what `card` normally does)
params = Params()
params.put("CarParams", CP.to_bytes())
print("CarParams written to params store")

# 3. instantiate Controls (reads CarParams back, builds the Mazda interface)
from openpilot.selfdrive.controls.controlsd import Controls
controls = Controls()
print("\nControls() INITIALIZED")
print("  car interface:", type(controls.CI).__module__)
print("  lateral controller:", type(controls.LaC).__name__)
print("  longitudinal controller:", type(controls.LoC).__name__)
print("  vehicle model steerRatio:", controls.VM.sR)
print("\n=== FULL CONTROL STACK LIVE FOR THE CX-5 ===")
