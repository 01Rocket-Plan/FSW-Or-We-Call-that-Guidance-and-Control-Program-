#!/usr/bin/env bash
# fpp-to-cpp + compile GnssDriver/Sim against F´ v4.4.1 headers with zig clang (-Wall).
set -euo pipefail
GNSS="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$GNSS/../.." && pwd)"
FPRIME="${FPRIME:-/workspace/fprime441}"
OUT="${OUT:-/tmp/gnss_fgen}"
OBJ="${OBJ:-/tmp/gnss_obj}"
if [[ -f /workspace/fpv/bin/activate ]]; then
  # shellcheck source=/dev/null
  source /workspace/fpv/bin/activate
elif [[ -f /workspace/fppenv/bin/activate ]]; then
  # shellcheck source=/dev/null
  source /workspace/fppenv/bin/activate
fi
CXX="${CXX:-python3 -m ziglang c++}"

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
  "$FPRIME/Drv/ByteStreamDriverModel/ByteStreamDriverModel.fpp" | sed 's/,$//')

echo "== fpp-check (stubs) =="
fpp-check \
  "$GNSS/fpp_stubs/Fw_Svc_Drv_stubs.fpp" \
  "$ROOT/RocketTypes/RocketTypes.fpp" \
  "$GNSS/GnssDriver/GnssDriver.fpp" \
  "$GNSS/GnssDriverSim/GnssDriverSim.fpp"
echo "fpp-check OK"

echo "== fpp-to-cpp =="
fpp-to-cpp -d "$OUT" -p "$ROOT,$FPRIME" -i "$IMP" \
  "$ROOT/RocketTypes/RocketTypes.fpp" \
  "$GNSS/GnssDriver/GnssDriver.fpp" \
  "$GNSS/GnssDriverSim/GnssDriverSim.fpp"

# Framework Ac scaffold from prior env_sensing compile tree if present
if [[ -d /tmp/fgen/Fw ]]; then
  for d in Fw Drv Svc cmake default; do
    rm -rf "$OUT/$d"
    cp -a "/tmp/fgen/$d" "$OUT/$d"
  done
fi

mkdir -p "$OUT/Components/Gnss/GnssDriver" "$OUT/Components/Gnss/GnssDriverSim" "$OUT/RocketTypes" "$OUT/Drv/ByteStreamDriverModel"
# Move generated component Ac files (Sim BEFORE Driver: GnssDriver* would also match Sim)
shopt -s nullglob
for f in "$OUT"/GnssDriverSim*ComponentAc.*; do
  mv "$f" "$OUT/Components/Gnss/GnssDriverSim/"
done
for f in "$OUT"/GnssDriver*ComponentAc.*; do
  mv "$f" "$OUT/Components/Gnss/GnssDriver/"
done
for f in "$OUT"/GnssSimFaultEnumAc.*; do
  mv "$f" "$OUT/Components/Gnss/GnssDriverSim/"
done
for f in "$OUT"/GnssLockStageEnumAc.*; do
  mv "$f" "$OUT/Components/Gnss/GnssDriver/"
done
for f in "$OUT"/*SerializableAc.* "$OUT"/*PortAc.* "$OUT"/*EnumAc.*; do
  mv "$f" "$OUT/RocketTypes/" 2>/dev/null || true
done
shopt -u nullglob

# ByteStream Ac under Drv/ByteStreamDriverModel/ (matches generated includes)
need_bs=0
[[ -f "$OUT/Drv/ByteStreamDriverModel/ByteStreamDataPortAc.hpp" ]] || need_bs=1
if [[ "$need_bs" -eq 1 ]]; then
  BS_OUT=/tmp/bs_port_gen
  rm -rf "$BS_OUT" && mkdir -p "$BS_OUT"
  BS_IMP=$(printf '%s,' \
    "$PLAT/PlatformTypes.fpp" \
    "$CFG/FpConfig.fpp" \
    "$CFG/ComCfg.fpp" \
    "$CFG/FpConstants.fpp" \
    "$CFG/AcConstants.fpp" \
    "$FPRIME/Fw/Types/Types.fpp" \
    "$FPRIME/Fw/Buffer/Buffer.fpp" | sed 's/,$//')
  fpp-to-cpp -d "$BS_OUT" -p "$FPRIME" -i "$BS_IMP" \
    "$FPRIME/Drv/ByteStreamDriverModel/ByteStreamDriverModel.fpp"
  mkdir -p "$OUT/Drv/ByteStreamDriverModel"
  # Generated files may be flat or already in subdir
  shopt -s nullglob
  for f in "$BS_OUT"/ByteStream*.hpp "$BS_OUT"/ByteStream*.cpp \
           "$BS_OUT"/Drv/ByteStreamDriverModel/ByteStream*.hpp \
           "$BS_OUT"/Drv/ByteStreamDriverModel/ByteStream*.cpp; do
    cp -a "$f" "$OUT/Drv/ByteStreamDriverModel/"
  done
  shopt -u nullglob
fi

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
  "$OUT/Components/Gnss/GnssDriver/GnssDriverComponentAc.cpp"
  "$OUT/Components/Gnss/GnssDriverSim/GnssDriverSimComponentAc.cpp"
  "$GNSS/GnssDriver/GnssDriver.cpp"
  "$GNSS/GnssDriver/GnssCore.cpp"
  "$GNSS/GnssDriverSim/GnssDriverSim.cpp"
  "$GNSS/GnssDriverSim/GnssSimDevice.cpp"
)
# Plus any RocketTypes / Drv Ac we need for the component objects
shopt -s nullglob
for f in "$OUT/RocketTypes"/*Ac.cpp "$OUT/Drv/ByteStreamDriverModel"/*Ac.cpp "$OUT/Components/Gnss/GnssDriverSim"/*EnumAc.cpp "$OUT/Components/Gnss/GnssDriver"/*EnumAc.cpp; do
  TUS+=("$f")
done
shopt -u nullglob

n=0
warn_err=0
for src in "${TUS[@]}"; do
  [[ -f "$src" ]] || { echo "MISSING $src"; warn_err=1; continue; }
  base=$(basename "$src")
  obj="$OBJ/${base%.*}.o"
  echo "  $base"
  if ! $CXX -std=c++14 -c -Wall -o "$obj" "${INC[@]}" "$src" 2>/tmp/gnss_cc_err.txt; then
    echo "COMPILE FAIL $base"; cat /tmp/gnss_cc_err.txt; warn_err=1
  elif [[ -s /tmp/gnss_cc_err.txt ]]; then
    echo "WARNINGS $base:"; cat /tmp/gnss_cc_err.txt
  fi
  n=$((n+1))
done
if [[ "$warn_err" -ne 0 ]]; then
  echo "F´ compile: FAIL ($n TU attempted)"
  exit 1
fi
echo "F´ compile: PASS ($n TU, 0 errors)"
