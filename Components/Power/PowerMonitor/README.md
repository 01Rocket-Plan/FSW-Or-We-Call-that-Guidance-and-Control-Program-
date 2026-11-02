# PowerMonitor

- 負責：電源組（Zamir）
- 範圍：電壓軌遙測（PowerRail core + Sim）、UvState 子元件、L1/L2 卸載請求
- Base ID：PowerMonitor `0x6000`（Sim `0x6080`）；UvStateComp `0x6200`（Sim `0x6280`）
- 狀態：v0.1 host-tested cores；F´ shell 待拓撲接線與 ESP32 ADC 實測

## Layout
- `PowerMonitor.*` — F´ shell（rg100Hz、PowerRailTlm×3）
- `PowerRailCore.*` / `IAdcReader.hpp` / `PowerRailSim.*` — host-testable core
- `UvState/` — UvStateComp + Core（無 L3；SEP/傘永不卸載）
- `test/PowerRailHostTest.cpp`

## Host tests
See `../run_host_tests.sh`.
