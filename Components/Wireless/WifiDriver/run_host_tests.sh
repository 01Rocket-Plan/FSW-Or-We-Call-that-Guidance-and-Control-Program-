#!/usr/bin/env bash
# WifiDriver 主機快速自測（不需 F´）：RfDriverCore（原檔）＋WifiLinkCore＋WifiHalSim
# 加 sanitizer：CXXFLAGS="-fsanitize=address,undefined -g" bash Components/Wireless/WifiDriver/run_host_tests.sh
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
W="$ROOT/Components/Wireless"
if [[ -z "${CXX:-}" ]]; then
  if command -v c++ >/dev/null 2>&1; then CXX=c++
  elif command -v g++ >/dev/null 2>&1; then CXX=g++
  else CXX="python3 -m ziglang c++"; fi
fi
echo "== WifiDriver host =="
# shellcheck disable=SC2086
$CXX -std=c++14 -DHOST_TEST_MAIN -I"$ROOT" -Wall -Wextra -Wpedantic ${CXXFLAGS:-} \
  "$W/WifiDriver/test/WifiDriverHostTest.cpp" \
  "$W/WifiDriver/WifiLinkCore.cpp" \
  "$W/WifiDriverSim/WifiHalSim.cpp" \
  "$W/RfDriver/RfDriverCore.cpp" \
  "$W/RfDriver/LoraTimeOnAir.cpp" \
  "$W/Framer/FramerCore.cpp" \
  -o /tmp/wifidriver_host_test
/tmp/wifidriver_host_test
echo "ALL WIFIDRIVER HOST TESTS PASSED"
