#!/usr/bin/env bash
# 伺服 Sim 核心 host 測試（不需 F´）。CXX 預設 c++，可設 CXX="python3 -m ziglang c++"
set -e; H=$(cd "$(dirname "$0")" && pwd); CXX=${CXX:-c++}
$CXX -std=c++14 -Wall -Wextra -Werror $H/SimCore/test/ServoSimHostTest.cpp $H/SimCore/ServoSimCore.cpp -o /tmp/servo_sim_host_test && /tmp/servo_sim_host_test
