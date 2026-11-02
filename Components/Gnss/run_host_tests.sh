#!/usr/bin/env bash
# GNSS 主機端單元測試（不需 F´）。共用機：CXX="python3 -m ziglang c++" 或 venv zig
set -euo pipefail
GNSS="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$GNSS/../.." && pwd)"
CXX="${CXX:-c++}"
if ! command -v "${CXX%% *}" >/dev/null 2>&1 && [[ "$CXX" == c++ ]]; then
  if python3 -c 'import ziglang' 2>/dev/null; then CXX='python3 -m ziglang c++'
  elif [[ -x /workspace/gnss/.venv/bin/python ]]; then CXX='/workspace/gnss/.venv/bin/python -m ziglang c++'
  fi
fi
F="-std=c++14 -Wall -Wextra -I$ROOT"
SIM="$GNSS/GnssDriverSim"
CORE="$GNSS/GnssDriver"
echo "== host T9 (sim device) =="
$CXX $F "$SIM/test/GnssHostTest.cpp" "$SIM/GnssSimDevice.cpp" -o /tmp/gnss_host_test
/tmp/gnss_host_test
echo "== host T1-T12 (LockStage) (GnssCore + SimDevice) =="
$CXX $F "$SIM/test/GnssCoreTest.cpp" "$SIM/GnssSimDevice.cpp" "$CORE/GnssCore.cpp" -o /tmp/gnss_core_test
/tmp/gnss_core_test
echo "host tests: PASS"
