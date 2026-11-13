#!/usr/bin/env bash
# FlightMode 殼主機測試（不需 F´）
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
# 編譯器：先用 $CXX；再找 c++／g++／clang++；最後退回 python 的 ziglang（同 tools/GDS）
if [ -n "${CXX:-}" ]; then :
elif command -v c++ >/dev/null 2>&1; then CXX=c++
elif command -v g++ >/dev/null 2>&1; then CXX=g++
elif command -v clang++ >/dev/null 2>&1; then CXX=clang++
elif command -v python3 >/dev/null 2>&1 && python3 -c "import ziglang" 2>/dev/null; then CXX="python3 -m ziglang c++"
elif command -v python >/dev/null 2>&1 && python -c "import ziglang" 2>/dev/null; then CXX="python -m ziglang c++"
else echo "找不到 C++ 編譯器：請安裝 g++／clang++，或 pip install ziglang，或設定 CXX" >&2; exit 1; fi
OUT="${TMPDIR:-/tmp}/mcu_flightmode_test"
echo "== FlightMode shell host（CXX=$CXX）=="
# shellcheck disable=SC2086
$CXX -std=c++14 -Wall -Wextra -Werror -O0 -I"$ROOT" \
  "$HERE/FlightModeCore.cpp" "$HERE/test/FlightModeShellTest.cpp" -o "$OUT"
"$OUT"
