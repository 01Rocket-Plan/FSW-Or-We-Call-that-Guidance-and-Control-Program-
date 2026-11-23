#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
ES="$(cd "$(dirname "$0")/.." && pwd)"
if [[ -f /workspace/fppenv/bin/activate ]]; then
  # shellcheck source=/dev/null
  source /workspace/fppenv/bin/activate
elif [[ -f /workspace/fpv/bin/activate ]]; then
  # shellcheck source=/dev/null
  source /workspace/fpv/bin/activate
fi
echo "== fpp-check =="
fpp-check \
  "$ES/fpp_stubs/Fw_Svc_Drv_stubs.fpp" \
  "$ES/Bus/EnvSensingBus.fpp" \
  "$ROOT/RocketTypes/RocketTypes.fpp" \
  "$ES/ImuDriver/ImuDriver.fpp" \
  "$ES/Lsm6dsoDriver/Lsm6dsoDriver.fpp" \
  "$ES/UsMpu6050/UsMpu6050.fpp"
echo "fpp-check OK"
echo "== host unit tests =="
cd "$ES/host_test"
gcc -std=c11 -O2 -Wall -Wextra -o test_imu_core \
  test_imu_core.c \
  ../ImuDriver/imu_core.c \
  ../UsMpu6050/mpu6050_core.c -lm
./test_imu_core
echo "== LSM6DSO 備援核心（lsm6dso-alt 分支）=="
gcc -std=c11 -O2 -Wall -Wextra -c -o lsm6dso_core.o ../Lsm6dsoDriver/lsm6dso_core.c
g++ -std=c++14 -O2 -Wall -Wextra -o test_lsm6dso_core test_lsm6dso_core.cpp lsm6dso_core.o -lm
rm -f lsm6dso_core.o
./test_lsm6dso_core
