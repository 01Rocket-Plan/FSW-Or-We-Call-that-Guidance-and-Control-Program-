#!/usr/bin/env bash
# fpp-check FlightMode.fpp（EnvSensing F´ stub＋Power NvFlags 埠）
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
fpp-check \
  "$ROOT/Components/Power/fpp_stubs/Fw_Svc_stubs.fpp" \
  "$ROOT/RocketTypes/RocketTypes.fpp" \
  "$ROOT/Components/Power/NvFlags/NvFlagsPorts.fpp" \
  "$HERE/FlightMode.fpp"
echo "fpp-check OK"
