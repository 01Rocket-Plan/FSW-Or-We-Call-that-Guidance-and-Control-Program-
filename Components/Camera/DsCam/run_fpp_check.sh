#!/usr/bin/env bash
# fpp-check Esp32Cam.fpp against shared CamTypes (+ EnvSensing-style stubs)
set -euo pipefail
DS="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DS/../../.." && pwd)"
CAM="$(cd "$DS/.." && pwd)"
if [[ -f /workspace/fpv/bin/activate ]]; then
  # shellcheck source=/dev/null
  source /workspace/fpv/bin/activate
elif [[ -f /workspace/fppenv/bin/activate ]]; then
  # shellcheck source=/dev/null
  source /workspace/fppenv/bin/activate
fi
STUBS="$ROOT/Components/EnvSensing/fpp_stubs/Fw_Svc_Drv_stubs.fpp"
echo "== fpp-check (CamTypes + Esp32Cam) =="
fpp-check \
  "$STUBS" \
  "$CAM/Ov3640Driver/CamTypes.fpp" \
  "$DS/Esp32Cam.fpp"
echo "fpp-check OK"
