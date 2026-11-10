#!/usr/bin/env bash
# ======================================================================
# fprime_verify.sh — 用「真 F´」（lib/fprime，v4.4.1）建置檢驗
#
# 規則：docs/REAL_FPRIME_VERIFICATION.md。stub 的 fpp-check／host 測試只算草稿自測，
#       PR 標「可審」之前一定要跑這支並貼結果。
#
# 用法：
#   tools/verify/fprime_verify.sh                       # 整個 repo（用根目錄 CMakeLists.txt）
#   tools/verify/fprime_verify.sh <模組目錄> [...]      # 只建列出的模組（依賴要自己一起列）
#     例：tools/verify/fprime_verify.sh Components/Power/NvFlags Components/Power/PowerMonitor
#   tools/verify/fprime_verify.sh --ut [模組目錄...]    # 另外建＋跑 F´ 單元測試（register_fprime_ut，gtest）
#   tools/verify/fprime_verify.sh --run 3 [模組目錄...] # 建完把每個部署（Deployments/*）各跑 3 秒，印事件（SIL）
#
# 環境變數：
#   FPRIME_FRAMEWORK  F´ 框架路徑（預設 <repo>/lib/fprime；要 v4.4.1）
#   WORK              暫存目錄（預設 /tmp/fprime_verify）
#   JOBS              平行數（預設 nproc）
#   REPO_ROOT         要檢驗的 repo 根目錄（預設＝這支腳本所在的 repo）
#
# 需求：Linux／WSL、cmake ≥ 3.16、g++、python venv 內裝 fprime-tools==4.4.0、fprime-fpp==3.4.0
#
# 結果：全部模組（含部署）編譯＋連結成功才回 0。失敗時印出前 40 行 error。
#       專案程式（不含 lib/fprime）的 warning 會列出來，但不擋。
# ======================================================================
set -euo pipefail
ROOT="${REPO_ROOT:-$(cd "$(dirname "$0")/../.." && pwd)}"
FW="${FPRIME_FRAMEWORK:-$ROOT/lib/fprime}"
WORK="${WORK:-${TMPDIR:-/tmp}/fprime_verify}"
JOBS="${JOBS:-$(nproc)}"

[[ -f "$FW/cmake/FPrime.cmake" ]] || { echo "找不到 F´ 框架：$FW（git submodule update --init lib/fprime，或設 FPRIME_FRAMEWORK）" >&2; exit 2; }
command -v fpp-check >/dev/null || { echo "找不到 fpp 工具：pip install fprime-tools==4.4.0 fprime-fpp==3.4.0" >&2; exit 2; }
FW_VER="$(git -C "$FW" describe --tags 2>/dev/null || echo unknown)"

UT=0
RUN_S=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ut) UT=1; shift ;;
    --run) RUN_S="${2:?--run 要給秒數}"; shift 2 ;;
    *) break ;;
  esac
done

P="$WORK/proj"
rm -rf "$P" && mkdir -p "$P/lib"

if [[ $# -eq 0 ]]; then
  MODE="整個 repo"
  # 工作樹（含未 commit 的修改）整份複製，跳過框架與舊的建置目錄
  (cd "$ROOT" && tar --exclude=./lib/fprime --exclude='./build*' --exclude=./.git -cf - .) | (cd "$P" && tar -xf -)
else
  MODE="模組：$*"
  cp -r "$ROOT/RocketTypes" "$P/"
  for m in "$@"; do
    m="${m%/}"
    [[ -f "$ROOT/$m/CMakeLists.txt" ]] || { echo "沒有 $m/CMakeLists.txt" >&2; exit 2; }
    mkdir -p "$P/$(dirname "$m")" && cp -r "$ROOT/$m" "$P/$m"
  done
  {
    echo 'cmake_minimum_required(VERSION 3.16)'
    echo 'project(FprimeVerify C CXX)'
    echo 'include("${CMAKE_CURRENT_LIST_DIR}/lib/fprime/cmake/FPrime.cmake")'
    echo 'include("${FPRIME_FRAMEWORK_PATH}/cmake/FPrime-Code.cmake")'
    echo 'add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/RocketTypes")'
    for m in "$@"; do echo "add_fprime_subdirectory(\"\${CMAKE_CURRENT_LIST_DIR}/${m%/}\")"; done
  } > "$P/CMakeLists.txt"
  printf '[fprime]\nproject_root: .\nframework_path: ./lib/fprime\ndefault_toolchain: native\nlibrary_locations:\n' > "$P/settings.ini"
fi

# F´ 不接受 symlink 的框架路徑：同一檔案系統用硬連結（不佔空間），否則整份複製
rm -rf "$P/lib/fprime"
cp -al "$FW" "$P/lib/fprime" 2>/dev/null || { rm -rf "$P/lib/fprime"; cp -r "$FW" "$P/lib/fprime"; }

echo "== 真 F´ 檢驗（框架 $FW_VER）— $MODE"
mkdir -p "$P/build" && cd "$P/build"
if ! cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug > cfg.log 2>&1; then
  echo "!! configure 失敗（常見：FPP 符號重複定義／未定義、模組沒列進 SOURCE_FILES）"
  grep -E "error|Error" cfg.log | head -40 || tail -40 cfg.log
  exit 1
fi
if ! make -k -j"$JOBS" all > build.log 2>&1; then
  echo "!! 編譯／連結失敗"
  grep -E "error:|undefined reference" build.log | sort -u | head -40
  exit 1
fi
W="$(grep -E "warning:" build.log | grep -v "/lib/fprime/" | sort -u || true)"
if [[ -n "$W" ]]; then
  echo "-- 專案程式 warning（不擋，但請清掉）："
  echo "$W" | head -40
fi
N="$(grep -c -E "Built target" build.log || true)"
echo "== 通過：configure＋build 成功（$N 個 target）。log：$P/build/build.log"

if [[ "$UT" == 1 ]]; then
  # 另開一個 BUILD_TESTING=ON 的建置目錄，只編「本專案」模組的 UT（不編 F´ 框架自己的上千個測試；
  # Deployments_*_ut_exe 是「部署所有依賴的 UT」總集合，會把框架的也拉進來，所以排除）
  mkdir -p "$P/build-ut" && cd "$P/build-ut"
  if ! cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON > cfg.log 2>&1; then
    echo "!! UT configure 失敗"; grep -E "error|Error" cfg.log | head -20; exit 1
  fi
  mapfile -t UTS < <(make help | sed -n 's/^\.\.\. \(\(Components\|RocketTypes\)[A-Za-z0-9_]*_ut_exe\)$/\1/p')
  if [[ ${#UTS[@]} -eq 0 ]]; then
    echo "-- 沒有找到本專案的單元測試（register_fprime_ut）"
  else
    if ! make -k -j"$JOBS" "${UTS[@]}" > ut_build.log 2>&1; then
      echo "!! 單元測試編譯失敗"
      grep -E "error:|undefined reference" ut_build.log | sort -u | head -40
      exit 1
    fi
    R="$(IFS='|'; echo "${UTS[*]}")"
    if ! ctest -R "^(${R})$" --output-on-failure > ut_run.log 2>&1; then
      echo "!! 單元測試失敗"
      grep -E "Failed|FAILED|Error|Failure" ut_run.log | head -40
      exit 1
    fi
    echo "== 單元測試通過（${#UTS[@]} 支）：$(grep -E "tests passed" ut_run.log)"
  fi
  cd "$P/build"
fi

if [[ -n "$RUN_S" ]]; then
  shopt -s nullglob
  for B in "$P"/build/bin/*/Deployments_*; do
    echo "== SIL：$(basename "$B")（${RUN_S} s）"
    (cd "$WORK" && timeout -s TERM "$RUN_S" "$B" 2>&1 || true) | grep "EVENT" | grep -v -E "prmDb|CdhCore.version" | sed -E 's/^EVENT: \([0-9]+\) \([^)]*\) //' | head -60
  done
fi
