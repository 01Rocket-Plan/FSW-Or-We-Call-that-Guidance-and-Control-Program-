#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
RF="$ROOT/Components/Wireless/RfDriver"
RS="$ROOT/Components/Wireless/RfDriverSim"
if command -v c++ >/dev/null 2>&1; then CXX=${CXX:-c++}
elif command -v g++ >/dev/null 2>&1; then CXX=${CXX:-g++}
else CXX=${CXX:-"/usr/bin//usr/bin/python3 -m ziglang c++"}; fi
echo "== RfDriver host =="
# shellcheck disable=SC2086
$CXX -std=c++17 -Wall -Wextra -I"$RF" -I"$RS" \
  "$RF/test/RfDriverHostTest.cpp" \
  "$RF/RfDriverCore.cpp" \
  "$RS/RfDriverSimDevice.cpp" \
  -o /tmp/rfdriver_host_test
/tmp/rfdriver_host_test
echo "ALL RFDRIVER HOST TESTS PASSED"
