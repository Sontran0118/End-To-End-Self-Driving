#!/bin/bash
set -e
export PATH=$HOME/.local/bin:$PATH
cd ~/op_fork/openpilot/common

echo "=== deps params.cc needs ==="
grep -E '#include' params.cc | head -20

echo "=== cythonize params_pyx.pyx ==="
cythonize params_pyx.pyx 2>&1 | tail -3

echo "=== compile ==="
# openpilot include root is the repo dir so "common/params.h" resolves
INC="-I$HOME/op_fork/openpilot -I$HOME/op_fork -I/usr/include/python3.12"
g++ -std=c++1z -fPIC -O2 -shared $INC \
    params_pyx.cpp params.cc util.cc \
    -o params_pyx.so 2>&1 | tail -15 && echo "BUILD OK"
ls -la params_pyx.so 2>&1
