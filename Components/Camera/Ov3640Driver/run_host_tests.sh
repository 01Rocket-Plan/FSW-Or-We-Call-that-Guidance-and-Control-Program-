#!/usr/bin/env bash
# OV3640 Core／Sim／Adapter host 測試 + fpp-check（不需完整 F´ 建置）
set -euo pipefail
H=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$H/../../.." && pwd)
if [[ -f /workspace/fppenv/bin/activate ]]; then
  # shellcheck source=/dev/null
  source /workspace/fppenv/bin/activate
elif [[ -f /workspace/fpv/bin/activate ]]; then
  # shellcheck source=/dev/null
  source /workspace/fpv/bin/activate
fi

if command -v zig >/dev/null 2>&1; then
  CXX=(zig c++)
elif [[ -x /home/box/.local/lib/python3.13/site-packages/ziglang/zig ]]; then
  CXX=(/home/box/.local/lib/python3.13/site-packages/ziglang/zig c++)
elif command -v g++ >/dev/null 2>&1; then
  CXX=(g++)
elif command -v c++ >/dev/null 2>&1; then
  CXX=(c++)
else
  echo "No C++ compiler found" >&2
  exit 1
fi

echo "== fpp-check =="
fpp-check \
  "$H/fpp_stubs/Fw_Svc_Drv_stubs.fpp" \
  "$H/CamTypes.fpp" \
  "$H/Ov3640Driver.fpp"
echo "fpp-check OK"

echo "== Ov3640SimTest (${CXX[*]}) =="
# host_ac first: CamTypes.hpp includes autocoded *Ac.hpp (stand-ins live here)
"${CXX[@]}" -std=c++17 -Wall -Wextra -Werror -I"$H/test/host_ac" -I"$ROOT" -I"$H" \
  "$H/test/Ov3640SimTest.cpp" \
  "$H/Ov3640Core.cpp" \
  "$H/Ov3640SimDevice.cpp" \
  -o /tmp/ov3640_sim_test
/tmp/ov3640_sim_test

echo "== Ov3640AdapterTest (${CXX[*]}) =="
# host_ac first so ComponentAc stub wins over missing generated headers
"${CXX[@]}" -std=c++17 -Wall -Wextra -Werror \
  -I"$H/test/host_ac" -I"$ROOT" -I"$H" \
  "$H/test/Ov3640AdapterTest.cpp" \
  "$H/Ov3640Core.cpp" \
  "$H/Ov3640SimDevice.cpp" \
  "$H/Ov3640Driver.cpp" \
  "$H/Ov3640DriverSim.cpp" \
  -o /tmp/ov3640_adapter_test
/tmp/ov3640_adapter_test
