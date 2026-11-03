# Ov3640Driver

- 負責：相機驅動組子Bot2（US OV3640）
- 範圍：`Ov3640Core` + `Ov3640SimDevice` + F´ 外殼 `Ov3640Driver`／`Ov3640DriverSim` + FPP
- FPP 宣告：型號 `OV3640`、角色 `US_CAM`
- 遙測 `CamStatus` 欄位：`camState`（FPP 保留字 `state` 已改名）、`fault`、`frameCount`、`model`、`role`
- 影像埠：`Ov3640ImageOut(index, ref data: Fw.Buffer)` → sdLogger
- Rate group：`sched1Hz`／`sched100Hz`；不接 1 kHz；無 sleep／系統時間／動態配置
- Host：`./run_host_tests.sh`（含 `fpp-check` + Sim／Adapter 測試）
- `fpp_stubs/` 僅供本機 fpp-check，不進飛行映像
