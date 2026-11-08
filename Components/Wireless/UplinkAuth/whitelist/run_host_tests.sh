#!/usr/bin/env bash
# Host unit tests for the UplinkAuth whitelist gate. No F´ required.
# Override compiler with CXX=...; falls back to c++ / g++ / zig c++.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
if [ -z "${CXX:-}" ]; then
  if command -v c++ >/dev/null 2>&1; then CXX=c++
  elif command -v g++ >/dev/null 2>&1; then CXX=g++
  elif command -v zig >/dev/null 2>&1; then CXX="zig c++"
  else CXX="python3 -m ziglang c++"
  fi
fi
CXXFLAGS=(-std=c++17 -Wall -Wextra -Werror -Wno-unused-variable -O0 -UNDEBUG -I"$HERE")
cd "$HERE"
# shellcheck disable=SC2086
$CXX "${CXXFLAGS[@]}" WhitelistGate.cpp WhitelistSimTable.cpp test/WhitelistHostTest.cpp -o build_whitelist_test
./build_whitelist_test
