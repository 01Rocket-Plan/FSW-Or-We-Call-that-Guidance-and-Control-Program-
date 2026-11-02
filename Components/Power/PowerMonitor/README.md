# PowerMonitor

- 負責：電源組（Zamir）
- 範圍：電壓軌遙測（PowerRail core + Sim）、UvState 子元件、L1/L2 卸載請求
- Base ID：PowerMonitor `0x6000`（Sim `0x6080`）；UvStateComp `0x6200`（Sim `0x6280`）
- 狀態：v0.1 host-tested cores；F´ shell 待拓撲接線與 ESP32 ADC 實測

## 範圍說明（回覆 Edward 審查）

**是的：這裡只談電壓監控與欠壓應急處理。**

- 本資料夾的軟體職責＝量測三條電壓軌、發出 `PowerRailTlm`、跑 `UvState` 欠壓狀態機、發出 L1／L2 卸載請求。
- 「誰供電給誰」（電源樹、穩壓器、配線）屬於硬體／拓撲接線，不在本元件軟體範圍；到時候線自己接一接即可。
- 尾翼鎖定／L3、SEP、降落傘是否卸載，由 `flightMode`／`actuatorMgr` 依 §7 決定；電力組只發 `UvState` 事件與 L1／L2 請求。

## Layout
- `PowerMonitor.*` — F´ shell（rg100Hz、PowerRailTlm×3）
- `PowerRailCore.*` / `IAdcReader.hpp` / `PowerRailSim.*` — host-testable core
- `UvState/` — UvStateComp + Core（無 L3；SEP/傘永不卸載）
- `test/PowerRailHostTest.cpp`

## Host tests
See `../run_host_tests.sh`.
