#!/usr/bin/env bash
# Host unit tests for Power group cores (no F´ required).
# CXX defaults to c++; override e.g. CXX="python -m ziglang c++" or zig c++.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
CXX=${CXX:-c++}
PM="$ROOT/Components/Power/PowerMonitor"
NV="$ROOT/Components/Power/NvFlags"
UV="$PM/UvState"

echo "== PowerRail (A) =="
$CXX -std=c++14 -I"$PM" \
  "$PM/test/PowerRailHostTest.cpp" "$PM/PowerRailCore.cpp" "$PM/PowerRailSim.cpp" \
  -o /tmp/power_rail_host_test
/tmp/power_rail_host_test

echo "== UvState (B) =="
$CXX -std=c++14 -I"$UV/Core" \
  "$UV/test/UvStateHostTest.cpp" "$UV/Core/UvStateCore.cpp" \
  -o /tmp/uvstate_host_test
/tmp/uvstate_host_test

echo "== NvFlags (C) =="
$CXX -std=c++14 -I"$NV" \
  "$NV/test/NvFlagsHostTest.cpp" "$NV/NvFlagsCore.cpp" \
  -o /tmp/nvflags_host_test
/tmp/nvflags_host_test

echo "ALL POWER HOST TESTS PASSED"
