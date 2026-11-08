#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
# 編譯器：先用 $CXX；再找 c++／g++／clang++；最後退回 python 的 ziglang（pip install ziglang）
if [ -n "${CXX:-}" ]; then :
elif command -v c++ >/dev/null 2>&1; then CXX=c++
elif command -v g++ >/dev/null 2>&1; then CXX=g++
elif command -v clang++ >/dev/null 2>&1; then CXX=clang++
elif command -v python3 >/dev/null 2>&1 && python3 -c "import ziglang" 2>/dev/null; then CXX="python3 -m ziglang c++"
elif command -v python >/dev/null 2>&1 && python -c "import ziglang" 2>/dev/null; then CXX="python -m ziglang c++"
else echo "找不到 C++ 編譯器：請安裝 g++／clang++，或 pip install ziglang，或設定 CXX" >&2; exit 1; fi
CXXFLAGS=(-std=c++17 -Wall -Wextra -Werror -Wno-unused-variable -O0 -UNDEBUG -I"$HERE")
cd "$HERE"
echo "== LinkLossView host（CXX=$CXX）=="
# shellcheck disable=SC2086
$CXX "${CXXFLAGS[@]}" LinkLossView.cpp test/LinkLossHostTest.cpp -o build_linkloss_test
./build_linkloss_test
