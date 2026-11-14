#!/usr/bin/env bash
# Host unit tests for UplinkAuth core (HMAC + anti-replay). No F´ required.
# Prefer zig c++ when system c++ is unavailable: CXX="python3 -m ziglang c++"
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
UA="$ROOT/Components/Wireless/UplinkAuth"
if command -v c++ >/dev/null 2>&1; then
  CXX=${CXX:-c++}
elif command -v g++ >/dev/null 2>&1; then
  CXX=${CXX:-g++}
else
  CXX=${CXX:-"python3 -m ziglang c++"}
fi

echo "== UplinkAuth host (HMAC + anti-replay) =="
# shellcheck disable=SC2086
$CXX -std=c++14 -DHOST_TEST_MAIN -I"$(git -C "$(dirname "$0")" rev-parse --show-toplevel)" -Wall -Wextra -I"$UA" \
  "$UA/test/UplinkAuthHostTest.cpp" \
  "$UA/UplinkAuthCore.cpp" \
  "$UA/HmacSha256.cpp" \
  -o /tmp/uplinkauth_host_test
/tmp/uplinkauth_host_test
echo "ALL UPLINKAUTH HOST TESTS PASSED"
