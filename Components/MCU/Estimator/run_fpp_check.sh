#!/usr/bin/env bash
# fpp-check Estimator.fpp（用 EnvSensing 的 F´ stub；lib/fprime 鎖定後改用真框架）
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
fpp-check \
  "$ROOT/Components/EnvSensing/fpp_stubs/Fw_Svc_Drv_stubs.fpp" \
  "$ROOT/RocketTypes/RocketTypes.fpp" \
  "$HERE/Estimator.fpp"
echo "fpp-check OK"
