#!/usr/bin/env bash
# 主機端單元測試（不需 F´）。CXX 預設 c++，可設 CXX="python -m ziglang c++"
set -e; R=$(cd "$(dirname "$0")/../.." && pwd); CXX=${CXX:-c++}; H=$(dirname "$0")
$CXX -std=c++14 -I"$R" $H/BaroDriver/test/BaroHostTest.cpp $H/BaroDriver/Bme680Core.cpp $H/BaroDriver/Bme680SimDevice.cpp -o /tmp/baro_host_test && /tmp/baro_host_test
$CXX -std=c++14 -I"$R" $H/UsDht22/test/Dht22HostTest.cpp $H/UsDht22/Dht22Core.cpp $H/UsDht22/Dht22SimIo.cpp -o /tmp/dht_host_test && /tmp/dht_host_test
