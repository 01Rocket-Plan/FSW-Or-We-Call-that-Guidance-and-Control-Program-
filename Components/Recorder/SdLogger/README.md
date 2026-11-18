# SdLogger

- 負責：紀錄組（Malcolm）／子Bot1＝遙測與事件字典；子Bot2＝DS/US 實作（本目錄）
- 範圍：DS/US sdLogger、遙測/事件字典、高水位、序號連續、Core+Sim、record-only 上游層
- Base ID：`0x2000`（Sim `0x2080`），見 `docs/conventions.md` §5
- 狀態：FPP 字典（子Bot1）+ Core/Sim/佇列實作（子Bot2）

## FPP-declared model / role

| Enum | Values | Notes |
|------|--------|-------|
| `SdCardModel` | `MICRO_SDHC_UHS_I_16GB` | Removable media card (Edward／首席). **Card reader module / interface board: TBD** |
| `SdCardRole` | `DS_LOG`, `US_LOG` | Topology instance role via param `CARD_ROLE` |
| `SdCardStatus` | OK/INIT/BUSY/CARD_MISSING/IO_ERROR/FULL | Media health |
| `SdSimFault` | NONE/FULL/NACK/WRITE_FAIL/CARD_MISSING | Sim inject |

## Policies

- Queue full → **drop newest** + `QueueFullDrop` (never block)
- Record **logger seq** + forward **SampleHeader.seq** (`LoggerSeq` / `ProducerSeq` / `LastSeq`)
- Contiguous logger write-seq; detect upstream `SeqGap` (0x2004) and `TimeNonMonotonic` (0x2005)
- Time only from F´ time service
- HWM: `RecordQueueHwm` (0x2002), `SdWritePoolHwm` (0x2003), `ImageQueueHwm` (0x2011)
- Image queue full → **drop newest** + `ImageChunkDrop` (never block)
- **不用動態記憶體**（Edward 2026-11-14 PR #55）：兩個佇列是 `StaticRing`（`SdLoggerCore.hpp`），容量編譯時固定：感測 64 筆（`PackedRecord` 約 280 B）、影像 8 塊（`ImageChunkRecord` 約 2,072 B），合計約 34.5 KB，開機就佔著。`RECORD_QUEUE_DEPTH`／`IMAGE_QUEUE_DEPTH` 是「最多用幾格」，設超過 64／8 → 用上限並發 `QueueDepthClamped`（0x200A）。`SdCardSimDevice` 的 `std::vector` 只在模擬／主機測試用，不進飛行板。

## New TLM (子Bot2)

| Abs | Name |
|-----|------|
| 0x2008 | HsState |
| 0x2009 | LoggerSeq |
| 0x200A | ProducerSeq |
| 0x200B | CardModel |
| 0x200C | CardRole |
| 0x200D | ImageChunkCount |
| 0x200E | ImageDropCount |
| 0x200F | ImageFrameCount |
| 0x2010 | ImageBytesWritten |
| 0x2011 | ImageQueueHwm |
| 0x2012 | LastImageFrame |
| 0x2013 | LastImageChunk |
| 0x2014 | SdBusyCount |

## Image EVR (子Bot2, after Bot1 0x2000–0x2005)

| Abs | Name |
|-----|------|
| 0x2006 | ImageChunkDrop |
| 0x2007 | ImageFrameGap |
| 0x2008 | ImageChunkGap |
| 0x2009 | ImageTruncatedSeen |
| 0x200A | QueueDepthClamped |

Ports: `usImageIn: CamImageOut`, `dsImageIn: CamImageOut`（型別在 `Components/Camera/CamTypes`） (2KB chunks; END=1, TRUNCATED=2). Param `IMAGE_QUEUE_DEPTH` default 8; channel `IMAGE=11`.

## Layout

- `SdCardCore` + `ISdMedia` + `SdCardSimDevice` (BME680-style)
- `SdLoggerCore` (queue / seq / time / drop / image chunks)
- `RecordChannels.hpp` — MPU6050 / DHT22 / GNSS + ImageChunkRecord
- `SdLogger` / `SdLoggerSim` F´ shells

## Tests

```bash
./test/run_host_tests.sh
python3 test/dict_id_selftest.py
# with fprime vendored:
fprime-util check
```

## SdCardSimDevice 寫入速度模型（SSS_Mission_Status #4／#5「SIL 測試發現」：寫多少都當場寫完）

- 每個 `advance()`（＝一個 `schedIn` tick，rg100Hz → 10 ms 視窗）最多寫 `writeBytesPerSec × 10 ms`；額度用完 `append()` 回 false、狀態 `BUSY`，`SdLoggerCore::flush()` 把那筆留在佇列下一 tick 再寫（不 FAULT、不丟）。可透支一筆、下個視窗扣回，長期平均＝設定速度。
- prm `SIM_WRITE_BYTES_PER_S`（SdLoggerSim）預設 **10 000 000 B/s** ＝ SD Speed Class 10／UHS U1 最低連續寫入 10 MB/s（SD Association Speed Class；性能單位 1 MB/s = 1 000 000 B/s）。repo 只寫卡型 `MICRO_SDHC_UHS_I_16GB`，沒寫速度等級，所以用 UHS-I 卡最低標示等級當保守下限。**紀錄組可再改**（Class 2/4/6 = 2/4/6 MB/s、U3 = 30 MB/s）。0 ＝ 不限速。
- 單次寫入忙碌：SD 規格只給上限（SDHC 寫入逾時 250 ms，SD Physical Layer Simplified Spec §4.6.2），沒有典型值，所以不編典型延遲；`injectBusyWindows(25)` 可注入 250 ms 最壞情況做測試。典型值待冷測量。
- BUSY 是背壓、不是錯誤（`SdLoggerCore::flush()`）：遇到 BUSY 不發 `SdError`、`SdCardCore` 不記錯誤，只把遙測 `SdBusyCount`（0x2014）+1，這一 tick 直接結束（影像佇列也等下一 tick）。連續「BUSY 且一筆都沒寫進去」超過 25 tick（＝ 250 ms SDHC 寫入逾時 ÷ 10 ms schedIn；`setBusyTimeoutTicks()`）才發**一次** `SdError(BUSY, code=4)`，卡又寫得進去才重新計算。只數 tick、不讀系統時間。CARD_MISSING／IO_ERROR 照舊是錯誤（FAULT）。**紀錄組可再改**。
