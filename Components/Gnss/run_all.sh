#!/usr/bin/env bash
set -euo pipefail
GNSS="$(cd "$(dirname "$0")" && pwd)"
export CXX="${CXX:-python3 -m ziglang c++}"
"$GNSS/run_host_tests.sh"
"$GNSS/run_fprime_compile_check.sh"
