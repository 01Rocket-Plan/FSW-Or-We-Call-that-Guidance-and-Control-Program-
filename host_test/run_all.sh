#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /workspace/fppenv/bin/activate
echo "== fpp-check =="
fpp-check \
  "$ROOT/Components/EnvSensing/fpp_stubs/Fw_Svc_Drv_stubs.fpp" \
  "$ROOT/Components/EnvSensing/fpp_stubs/EnvSensingBus.fpp" \
  "$ROOT/RocketTypes/RocketTypes.fpp" \
  "$ROOT/Components/EnvSensing/ImuDriver/ImuDriver.fpp" \
  "$ROOT/Components/EnvSensing/UsMpu6050/UsMpu6050.fpp"
echo "fpp-check OK"
echo "== host unit tests =="
cd "$ROOT/host_test"
gcc -std=c11 -O2 -Wall -Wextra -o test_imu_core \
  test_imu_core.c \
  ../Components/EnvSensing/ImuDriver/imu_core.c \
  ../Components/EnvSensing/UsMpu6050/mpu6050_core.c -lm
./test_imu_core
