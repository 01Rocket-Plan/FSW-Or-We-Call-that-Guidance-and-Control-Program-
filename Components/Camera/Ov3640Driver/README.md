# Ov3640Driver

- 負責：相機驅動組子Bot2（US OV3640）
- 範圍：`Ov3640Core`（純邏輯）+ `Ov3640SimDevice`（同一介面）+ FPP 宣告
- FPP 宣告：型號 `OV3640`、角色 `US_CAM`
- 介面：SCCB（16-bit reg / 8-bit data）、DVP 幀來源；影像只出 `imageOut` → sdLogger
- 遙測：僅 `CamStatus`（state / fault / frameCount / model / role）
- Rate group：1 Hz 握手、100 Hz 擷取；不接 1 kHz；無 sleep／系統時間／動態配置
- Host 測試：`./run_host_tests.sh`（`zig c++ -std=c++17` 或 `g++`）
- 狀態：Core／Sim 可 host 自測；FPP 未用 F´ 工具鏈驗證；F´ 元件殼（.cpp 接線）尚未實作，僅 FPP 宣告
