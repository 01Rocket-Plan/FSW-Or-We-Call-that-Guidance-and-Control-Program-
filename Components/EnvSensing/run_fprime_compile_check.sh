#!/usr/bin/env bash
# fpp-to-cpp + compile ImuDriver/Sim + UsMpu6050/Sim against F´ v4.4.1 headers with zig clang (-Wall).
set -euo pipefail
ES="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$ES/../.." && pwd)"
FPRIME="${FPRIME:-/workspace/fprime441}"
OUT="${OUT:-/tmp/imu_fgen}"
OBJ="${OBJ:-/tmp/imu_obj}"
if [[ -f /workspace/fpv/bin/activate ]]; then
  # shellcheck source=/dev/null
  source /workspace/fpv/bin/activate
elif [[ -f /workspace/fppenv/bin/activate ]]; then
  # shellcheck source=/dev/null
  source /workspace/fppenv/bin/activate
fi
CXX="${CXX:-python -m ziglang c++}"

rm -rf "$OUT" "$OBJ"
mkdir -p "$OUT" "$OBJ"

CFG="$FPRIME/default/config"
PLAT="$FPRIME/cmake/platform/unix/Platform"
IMP=$(printf '%s,' \
  "$PLAT/PlatformTypes.fpp" \
  "$CFG/FpConfig.fpp" \
  "$CFG/ComCfg.fpp" \
  "$CFG/FpConstants.fpp" \
  "$CFG/AcConstants.fpp" \
  "$FPRIME/Fw/Types/Types.fpp" \
  "$FPRIME/Fw/Buffer/Buffer.fpp" \
  "$FPRIME/Fw/Cmd/Cmd.fpp" \
  "$FPRIME/Fw/Log/Log.fpp" \
  "$FPRIME/Fw/Prm/Prm.fpp" \
  "$FPRIME/Fw/Time/Time.fpp" \
  "$FPRIME/Fw/Tlm/Tlm.fpp" \
  "$FPRIME/Fw/Com/Com.fpp" \
  "$FPRIME/Svc/Sched/Sched.fpp" \
  "$FPRIME/Drv/Ports/I2cDriverPorts.fpp" \
  "$FPRIME/Drv/Ports/SpiDriverPorts.fpp" | sed 's/,$//')

echo "== fpp-to-cpp =="
fpp-to-cpp -d "$OUT" -p "$ROOT,$FPRIME" -i "$IMP" \
  "$ROOT/RocketTypes/RocketTypes.fpp" \
  "$ES/EnvSensingBus.fpp" \
  "$ES/ImuDriver/ImuDriver.fpp" \
  "$ES/UsMpu6050/UsMpu6050.fpp"

# Framework Ac scaffold from Bot2 compile tree if present
if [[ -d /tmp/fgen/Fw ]]; then
  for d in Fw Drv Svc cmake default; do
    rm -rf "$OUT/$d"
    cp -a "/tmp/fgen/$d" "$OUT/$d"
  done
fi
# Ensure Spi ports exist (Bot2 fgen may lack SpiWriteRead)
if [[ ! -f "$OUT/Drv/Ports/SpiWriteReadPortAc.hpp" ]]; then
  SPI_OUT=/tmp/spi_port_gen
  rm -rf "$SPI_OUT" && mkdir -p "$SPI_OUT"
  SPI_IMP=$(printf '%s,'     "$PLAT/PlatformTypes.fpp"     "$CFG/FpConfig.fpp"     "$CFG/ComCfg.fpp"     "$CFG/FpConstants.fpp"     "$CFG/AcConstants.fpp"     "$FPRIME/Fw/Types/Types.fpp"     "$FPRIME/Fw/Buffer/Buffer.fpp" | sed 's/,$//')
  fpp-to-cpp -d "$SPI_OUT" -p "$FPRIME" -i "$SPI_IMP"     "$FPRIME/Drv/Ports/SpiDriverPorts.fpp"
  mkdir -p "$OUT/Drv/Ports"
  cp -a "$SPI_OUT"/Spi*.hpp "$SPI_OUT"/Spi*.cpp "$OUT/Drv/Ports/"
fi

mkdir -p "$OUT/Components/EnvSensing/ImuDriver" "$OUT/Components/EnvSensing/UsMpu6050" "$OUT/RocketTypes"
mv "$OUT"/ImuDriver*ComponentAc.* "$OUT/Components/EnvSensing/ImuDriver/"
mv "$OUT"/UsMpu6050*ComponentAc.* "$OUT/Components/EnvSensing/UsMpu6050/"
for f in RegReadPortAc RegWritePortAc SpiRegReadPortAc SpiRegWritePortAc FaultInjectPortAc BusStatusEnumAc; do
  mv "$OUT/${f}".hpp "$OUT/Components/EnvSensing/" 2>/dev/null || true
  mv "$OUT/${f}".cpp "$OUT/Components/EnvSensing/" 2>/dev/null || true
done
shopt -s nullglob
for f in "$OUT"/*SerializableAc.* "$OUT"/*PortAc.* "$OUT"/*EnumAc.*; do
  mv "$f" "$OUT/RocketTypes/"
done
shopt -u nullglob

INC=(
  -I"$ROOT"
  -I"$OUT"
  -I"$FPRIME"
  -I"$FPRIME/cmake/platform/unix"
  -I"$OUT/cmake/platform/unix"
  -I"$OUT/default"
  -I/tmp/cfg
)

echo "== compile (-c -Wall) =="
TUS=(
  "$OUT/Components/EnvSensing/ImuDriver/ImuDriverComponentAc.cpp"
  "$OUT/Components/EnvSensing/ImuDriver/ImuDriverSimComponentAc.cpp"
  "$OUT/Components/EnvSensing/UsMpu6050/UsMpu6050ComponentAc.cpp"
  "$OUT/Components/EnvSensing/UsMpu6050/UsMpu6050SimComponentAc.cpp"
  "$OUT/Components/EnvSensing/BusStatusEnumAc.cpp"
  "$OUT/Components/EnvSensing/RegReadPortAc.cpp"
  "$OUT/Components/EnvSensing/RegWritePortAc.cpp"
  "$OUT/Components/EnvSensing/SpiRegReadPortAc.cpp"
  "$OUT/Components/EnvSensing/SpiRegWritePortAc.cpp"
  "$OUT/Components/EnvSensing/FaultInjectPortAc.cpp"
  "$ES/ImuDriver/ImuDriver.cpp"
  "$ES/ImuDriver/ImuDriverSim.cpp"
  "$ES/ImuDriver/imu_core.c"
  "$ES/UsMpu6050/UsMpu6050.cpp"
  "$ES/UsMpu6050/UsMpu6050Sim.cpp"
  "$ES/UsMpu6050/mpu6050_core.c"
)
n=0
for src in "${TUS[@]}"; do
  base=$(basename "$src")
  obj="$OBJ/${base%.*}.o"
  echo "  $base"
  if [[ "$src" == *.c ]]; then
    $CXX -std=c11 -c -Wall -o "$obj" "${INC[@]}" "$src"
  else
    $CXX -std=c++14 -c -Wall -o "$obj" "${INC[@]}" "$src"
  fi
  n=$((n+1))
done
echo "F´ compile: PASS ($n TU, 0 errors/0 warnings expected)"
