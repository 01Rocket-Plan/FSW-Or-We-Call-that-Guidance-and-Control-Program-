#!/usr/bin/env bash
# 伺服 Sim 核心 + ModelCores host 測試（不需 F´）。CXX 預設 c++，可設 CXX="python3 -m ziglang c++"
set -e
H=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$H/../.." && pwd)
CXX=${CXX:-c++}

echo "== ServoSimCore host tests =="
$CXX -std=c++14 -Wall -Wextra -Werror \
  "$H/SimCore/test/ServoSimHostTest.cpp" "$H/SimCore/ServoSimCore.cpp" \
  -o /tmp/servo_sim_host_test && /tmp/servo_sim_host_test

echo "== ModelCores host tests =="
$CXX -std=c++14 -Wall -Wextra -Werror -I"$ROOT" \
  "$H/ModelCores/test/ut/ModelCoresTest.cpp" \
  "$H/ModelCores/Sg90Core.cpp" \
  "$H/ModelCores/Mg996rCore.cpp" \
  "$H/ModelCores/Sg90SimDevice.cpp" \
  "$H/ModelCores/Mg996rSimDevice.cpp" \
  -o /tmp/servo_modelcores_host_test && /tmp/servo_modelcores_host_test
