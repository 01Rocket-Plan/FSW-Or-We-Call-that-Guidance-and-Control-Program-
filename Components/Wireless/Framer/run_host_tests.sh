#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
FR="$ROOT/Components/Wireless/Framer"
if command -v c++ >/dev/null 2>&1; then CXX=${CXX:-c++}
elif command -v g++ >/dev/null 2>&1; then CXX=${CXX:-g++}
else CXX=${CXX:-"/usr/bin//usr/bin/python3 -m ziglang c++"}; fi
echo "== Framer host =="
# shellcheck disable=SC2086
$CXX -std=c++17 -DHOST_TEST_MAIN -I"$(git -C "$(dirname "$0")" rev-parse --show-toplevel)" -Wall -Wextra -I"$FR" \
  "$FR/test/FramerHostTest.cpp" "$FR/FramerCore.cpp" \
  -o /tmp/framer_host_test
/tmp/framer_host_test
echo "ALL FRAMER HOST TESTS PASSED"
