#!/usr/bin/env bash
# ======================================================================
# build_mcu_sil.sh — 建（並可跑）DsMcu／UsMcu 的 Linux SIL 部署
#
# 為什麼不直接在 repo 根目錄 cmake：F´ v4 在 configure 階段就對「整個專案」跑 fpp-depend，
# 目前其他組有 fpp 錯誤（見 issue），整個根專案 configure 不起來。這支腳本只把 MCU 部署
# 需要的模組複製到暫存專案，用 lib/fprime（v4.4.1）建。其他組修好後，改用根目錄建置即可。
#
# 需求：Linux／WSL、cmake ≥ 3.16、g++、python venv 內裝 fprime-tools==4.4.0、fprime-fpp==3.4.0
#       （pip install -r lib/fprime/requirements.txt 也可以）
#       git submodule update --init lib/fprime
#
# 用法：tools/sil/build_mcu_sil.sh            # 只建
#       tools/sil/build_mcu_sil.sh --run 3    # 建完各跑 3 秒，印出事件
# ======================================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
WORK="${WORK:-${TMPDIR:-/tmp}/mcu_sil_build}"
RUN_S=""
if [[ "${1:-}" == "--run" ]]; then RUN_S="${2:-3}"; fi

[[ -f "$ROOT/lib/fprime/cmake/FPrime.cmake" ]] || { echo "lib/fprime 不存在：git submodule update --init lib/fprime" >&2; exit 1; }
command -v fpp-check >/dev/null || { echo "找不到 fpp 工具：pip install fprime-tools==4.4.0 fprime-fpp==3.4.0" >&2; exit 1; }

MODS=(
  Components/MCU/Platform
  Components/MCU/Estimator
  Components/MCU/Controller
  Components/MCU/ActuatorMgr
  Components/MCU/FlightMode
  Components/Power/NvFlags
  Components/EnvSensing/BaroDriver
  Components/Wireless/PadLink
  Components/Wireless/UmbilicalLink
  Deployments/DsMcu
  Deployments/UsMcu
)

P="$WORK/proj"
rm -rf "$P" && mkdir -p "$P/lib"
cp -r "$ROOT/RocketTypes" "$P/"
for m in "${MODS[@]}"; do mkdir -p "$P/$(dirname "$m")" && cp -r "$ROOT/$m" "$P/$m"; done
# 框架用硬連結複製（同一檔案系統時不佔空間；F´ 不接受 symlink 的框架路徑）
if ! cp -al "$ROOT/lib/fprime" "$P/lib/fprime" 2>/dev/null; then
  rm -rf "$P/lib/fprime"  # 跨檔案系統時硬連結失敗，清掉半套再整份複製
  cp -r "$ROOT/lib/fprime" "$P/lib/fprime"
fi
printf '[fprime]\nproject_root: .\nframework_path: ./lib/fprime\ndefault_toolchain: native\nlibrary_locations:\n' > "$P/settings.ini"
{
  echo 'cmake_minimum_required(VERSION 3.16)'
  echo 'project(McuSil C CXX)'
  echo 'include("${CMAKE_CURRENT_LIST_DIR}/lib/fprime/cmake/FPrime.cmake")'
  echo 'include("${FPRIME_FRAMEWORK_PATH}/cmake/FPrime-Code.cmake")'
  echo 'add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/RocketTypes")'
  for m in "${MODS[@]}"; do echo "add_fprime_subdirectory(\"\${CMAKE_CURRENT_LIST_DIR}/$m\")"; done
} > "$P/CMakeLists.txt"

mkdir -p "$P/build" && cd "$P/build"
cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug > cfg.log 2>&1 || { tail -40 cfg.log; exit 1; }
make -j"$(nproc)" Deployments_DsMcu Deployments_UsMcu > build.log 2>&1 || { grep -E "error" build.log | head -30; exit 1; }
if grep -E "warning:" build.log | grep -q -E "Components/MCU|Deployments/(Ds|Us)Mcu"; then
  echo "MCU 程式有警告："; grep -E "warning:" build.log | grep -E "Components/MCU|Deployments/(Ds|Us)Mcu"; exit 1
fi
echo "SIL build OK：$P/build/bin/*/Deployments_DsMcu、Deployments_UsMcu"

if [[ -n "$RUN_S" ]]; then
  for d in DsMcu UsMcu; do
    B=$(ls "$P"/build/bin/*/Deployments_$d)
    echo "== $d（${RUN_S} s）"
    (cd "$WORK" && timeout -s TERM "$RUN_S" "$B" 2>&1 || true) | grep "EVENT" | grep -v -E "prmDb|CdhCore.version" \
      | sed -E 's/^EVENT: \([0-9]+\) \([^)]*\) //'
  done
fi
