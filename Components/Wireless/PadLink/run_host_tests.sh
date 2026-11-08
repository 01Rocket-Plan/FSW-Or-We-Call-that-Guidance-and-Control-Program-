#!/usr/bin/env bash
# Issue #24 host 自測：PadLink codec／PadCmdGate／PadControllerCore／UmbilicalTxCore（不需要 F´）
# 沒有系統 c++ 時：CXX="python -m ziglang c++"
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
PL="$ROOT/Components/Wireless/PadLink"
PC="$ROOT/Components/Wireless/PadController"
UT="$ROOT/Components/Wireless/UmbilicalTx"
if command -v c++ >/dev/null 2>&1; then CXX=${CXX:-c++}
elif command -v g++ >/dev/null 2>&1; then CXX=${CXX:-g++}
else CXX=${CXX:-"python3 -m ziglang c++"}; fi
OUT=${TMPDIR:-/tmp}/padlink_host_test
echo "== PadLink host (umbilical + pad<->ground + pad controller) =="
# shellcheck disable=SC2086
$CXX -std=c++14 -Wall -Wextra -Werror -I"$PL" -I"$PC" -I"$UT" \
  "$PL/test/PadLinkHostTest.cpp" "$PL/PadLinkCodec.cpp" "$PL/PadCmdGate.cpp" \
  "$PC/PadControllerCore.cpp" "$UT/UmbilicalTxCore.cpp" \
  -o "$OUT"
"$OUT"
echo "ALL PADLINK HOST TESTS PASSED"
