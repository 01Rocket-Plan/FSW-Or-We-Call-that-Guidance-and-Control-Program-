#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
TS="$ROOT/Components/Wireless/TlmSend"
if command -v c++ >/dev/null 2>&1; then CXX=${CXX:-c++}
elif command -v g++ >/dev/null 2>&1; then CXX=${CXX:-g++}
else CXX=${CXX:-"/usr/bin//usr/bin/python3 -m ziglang c++"}; fi
echo "== TlmSend host =="
# shellcheck disable=SC2086
$CXX -std=c++17 -Wall -Wextra -I"$TS" \
  "$TS/test/TlmSendHostTest.cpp" "$TS/TlmSendCore.cpp" \
  -o /tmp/tlmsend_host_test
/tmp/tlmsend_host_test
echo "ALL TLMSEND HOST TESTS PASSED"
