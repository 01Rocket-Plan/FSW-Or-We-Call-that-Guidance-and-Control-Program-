#!/usr/bin/env bash
# 主機端單元測試（不需 F´）。CXX 預設 c++，可設 CXX="python3 -m ziglang c++"
set -euo pipefail
R=$(cd "$(dirname "$0")/../../.." && pwd)
DS=$(cd "$(dirname "$0")" && pwd)
if [ -z "${CXX:-}" ]; then
  if command -v c++ >/dev/null 2>&1; then
    CXX=c++
  else
    CXX="python3 -m ziglang c++"
  fi
fi
CFLAGS=(-std=c++14 -Wall -Wextra -Werror -I"$R")

echo "== Esp32CamSimTest =="
$CXX "${CFLAGS[@]}" \
  "$DS/test/Esp32CamSimTest.cpp" \
  "$DS/Esp32CamCore.cpp" \
  "$DS/Esp32CamSimDevice.cpp" \
  -o /tmp/esp32cam_host_test
/tmp/esp32cam_host_test

echo "== Esp32CamWrapperLogicTest =="
$CXX "${CFLAGS[@]}" \
  "$DS/test/Esp32CamWrapperLogicTest.cpp" \
  "$DS/Esp32CamCore.cpp" \
  "$DS/Esp32CamSimDevice.cpp" \
  -o /tmp/esp32cam_wrapper_logic_test
/tmp/esp32cam_wrapper_logic_test

echo "== Ov3640SimTest (shared CamTypes camState) =="
$CXX "${CFLAGS[@]}" \
  "$R/Components/Camera/Ov3640Driver/test/Ov3640SimTest.cpp" \
  "$R/Components/Camera/Ov3640Driver/Ov3640Core.cpp" \
  "$R/Components/Camera/Ov3640Driver/Ov3640SimDevice.cpp" \
  -o /tmp/ov3640_host_test_from_esp32cam
/tmp/ov3640_host_test_from_esp32cam

echo "== Ov3640Driver full host suite (Sim + Adapter) =="
"$R/Components/Camera/Ov3640Driver/run_host_tests.sh"

