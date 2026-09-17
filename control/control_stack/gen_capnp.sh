#!/bin/bash
# Replicate openpilot cereal's capnp C++ codegen (from cereal/SConscript).
set -e
cd ~/op_fork/openpilot/cereal

CAR_CAPNP_DIR=/home/tran/opendbc_src/opendbc/car
mkdir -p gen/cpp

echo "=== generating C++ from capnp schemas ==="
# The SConscript uses --src-prefix twice (cereal dir + opendbc car dir) and
# import-path to the opendbc car dir so log.capnp's `import "/car.capnp"` resolves.
capnpc \
  --src-prefix=. \
  --src-prefix="$CAR_CAPNP_DIR" \
  --import-path="$CAR_CAPNP_DIR" \
  log.capnp deprecated.capnp custom.capnp "$CAR_CAPNP_DIR/car.capnp" \
  -o c++:gen/cpp/ 2>&1 | tail -10

echo "=== generated files ==="
ls -la gen/cpp/ 2>&1
