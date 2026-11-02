# [GNSS][梁承業] GnssDriver＋GnssDriverSim（TEL0157，GnssFix v0.2）

分支：`Gnss/gnss-driver`（**尚未推 GitHub／未開 PR**；本機交付給首席工程師開 PR）

- 安裝位置：**DS／US 皆可**（由拓撲決定；檔頭與本說明皆註明）。
- Base ID：依 `docs/conventions.md` 為 **GnssDriver `0x5000`／Sim `0x5080`**（組內曾暫定 `0x0A00` 已作廢）。

## 內容
- `Components/Gnss/GnssDriver/`：真驅動
  - `GnssCore`（與 F´ 無關）：NMEA GGA/RMC/VTG/GSA；握手 OFF→INIT→SELFTEST→READY/FAULT；每 tick 一次 `IGnssUart::readBytes`；不 sleep、不讀系統時間。
  - READY 每 tick 輸出一筆 `GnssFix`（`seq` 每次 +1）；`dataAge_s = now − last new GGA`；無垂直速度時 `velEnu_mps[2]=NaN`、`velUValid=false`。
  - `posValid` = 3D 且 `hdop <= HDOP_MAX`（prmDb）。
  - 截斷：計數 + `TRUNCATED` 事件，**不**進 FAULT；校驗和錯誤丟棄並計數。
  - F´ 外殼：`uartRecv`（`Drv.ByteStreamData`）→ ring buffer；`schedIn`（rg1Hz）；`fixOut: RocketTypes.GnssData` **只應接 tlmSend／sdLogger**。
- `Components/Gnss/GnssDriverSim/`：同一 `GnssCore` + `GnssSimDevice`（真值→雜訊／延遲／掉 fix→NMEA 位元組流）。指令：`SIM_SET_TRAJ`／`NOISE`／`LATENCY`／`INJECT`／`HDOP_BIAS`。
- `Components/Gnss/fpp_stubs/`：GNSS 專用 host `fpp-check` stubs（含 `BufferSend`／`ByteStream*`）。**未改** `Components/EnvSensing/fpp_stubs/`。
- `RocketTypes/RocketTypes.fpp`：僅加 GNSS 組標註區段；**未改** `GnssFix` 欄位（主幹已有）。
- `Gnss::toFpp`：`GnssToFpp.hpp`。FPP module／namespace：`Components`。

## 測試（見 deliver `test_results.log`）
- 主機 T1–T9：`Components/Gnss/run_host_tests.sh` **ALL PASS**。
- `fpp-check`（`Components/Gnss/fpp_stubs`）**OK**。
- `fpp-to-cpp` + 對 F´ **v4.4.1** 標頭 `-Wall` 編譯 **PASS**。

## 已知缺口
1. 主幹無 vendored `lib/fprime`，未做完整 F´ CMake 連結／實機 UART；編譯檢查對 `/workspace/fprime441`。
2. 拓撲尚未接 rg1Hz、prmDb、tlmSend、sdLogger、UART 驅動（DS／US）。
3. 僅 NMEA；UBX 未做。
4. 共用機以 `python3 -m ziglang c++` 編譯。
5. 未推遠端、未開 PR（交首席工程師）。
