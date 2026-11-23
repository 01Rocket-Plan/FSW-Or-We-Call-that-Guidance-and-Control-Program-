#!/usr/bin/env bash
# BleDriver 主機快速自測（不需 F´）：RfDriverCore（原檔）＋BleLinkCore＋BleHalSim
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
W="$ROOT/Components/Wireless"
if [[ -z "${CXX:-}" ]]; then
  if command -v c++ >/dev/null 2>&1; then CXX=c++
  elif command -v g++ >/dev/null 2>&1; then CXX=g++
  else CXX="python3 -m ziglang c++"; fi
fi
echo "== BleDriver host =="
# shellcheck disable=SC2086
$CXX -std=c++14 -DHOST_TEST_MAIN -I"$ROOT" -Wall -Wextra -Wpedantic \
  "$W/BleDriver/test/BleDriverHostTest.cpp" \
  "$W/BleDriver/BleLinkCore.cpp" \
  "$W/BleDriverSim/BleHalSim.cpp" \
  "$W/RfDriver/RfDriverCore.cpp" \
  "$W/Framer/FramerCore.cpp" \
  -o /tmp/bledriver_host_test
/tmp/bledriver_host_test
echo "ALL BLEDRIVER HOST TESTS PASSED"
