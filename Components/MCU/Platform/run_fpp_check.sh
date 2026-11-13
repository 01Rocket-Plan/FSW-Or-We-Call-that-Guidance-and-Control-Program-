#!/usr/bin/env bash
# fpp-check Platform.fpp＋SilImuFeed.fpp（用 EnvSensing 的 stub；lib/fprime 鎖定後可改用真框架）
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
fpp-check \
  "$ROOT/Components/EnvSensing/fpp_stubs/Fw_Svc_Drv_stubs.fpp" \
  "$ROOT/RocketTypes/RocketTypes.fpp" \
  "$HERE/Platform.fpp" \
  "$HERE/SilImuFeed/SilImuFeed.fpp"
echo "fpp-check OK"
