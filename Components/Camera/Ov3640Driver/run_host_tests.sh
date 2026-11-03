#!/usr/bin/env bash
# OV3640 Core／Sim host 測試（不需 F´）
set -euo pipefail
H=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$H/../../.." && pwd)
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
OUT=/tmp/ov3640_sim_test
echo "== Ov3640SimTest (${CXX[*]}) =="
"${CXX[@]}" -std=c++17 -Wall -Wextra -Werror -I"$ROOT" \
  "$H/test/Ov3640SimTest.cpp" \
  "$H/Ov3640Core.cpp" \
  "$H/Ov3640SimDevice.cpp" \
  -o "$OUT"
"$OUT"
