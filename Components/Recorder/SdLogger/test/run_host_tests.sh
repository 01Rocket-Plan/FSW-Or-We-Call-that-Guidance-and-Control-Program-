#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CXX="${CXX:-c++}"
OUT="${TMPDIR:-/tmp}/sdlogger_host_test"
$CXX -std=c++17 -O0 -g -Wall -Wextra -I"$ROOT/../../.." \
  "$ROOT/SdCardCore.cpp" "$ROOT/SdCardSimDevice.cpp" "$ROOT/SdLoggerCore.cpp" \
  "$ROOT/test/SdLoggerHostTest.cpp" -o "$OUT"
"$OUT"
