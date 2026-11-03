#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
CQ="$ROOT/Components/Wireless/ComQueue"
if command -v c++ >/dev/null 2>&1; then CXX=${CXX:-c++}
elif command -v g++ >/dev/null 2>&1; then CXX=${CXX:-g++}
else CXX=${CXX:-"/usr/bin/python3 -m ziglang c++"}; fi
echo "== ComQueue host =="
# shellcheck disable=SC2086
$CXX -std=c++17 -Wall -Wextra -pthread -I"$CQ" \
  "$CQ/test/ComQueueHostTest.cpp" \
  "$CQ/ComQueueCore.cpp" \
  -o /tmp/comqueue_host_test
/tmp/comqueue_host_test
echo "ALL COMQUEUE HOST TESTS PASSED"
