#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ZIG="${ZIG:-/home/box/.local/lib/python3.13/site-packages/ziglang/zig}"
CXXFLAGS=(-std=c++17 -Wall -Wextra -Werror -Wno-unused-variable -O0 -UNDEBUG -I"$HERE")
cd "$HERE"
"$ZIG" c++ "${CXXFLAGS[@]}" WhitelistGate.cpp WhitelistSimTable.cpp test/WhitelistHostTest.cpp -o build_whitelist_test
./build_whitelist_test
