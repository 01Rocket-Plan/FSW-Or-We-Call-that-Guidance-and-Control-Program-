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
- HWM: `RecordQueueHwm` (0x2002), `SdWritePoolHwm` (0x2003)

## New TLM (子Bot2, from abs 0x2008)

| Abs | Name |
|-----|------|
| 0x2008 | HsState |
| 0x2009 | LoggerSeq |
| 0x200A | ProducerSeq |
| 0x200B | CardModel |
| 0x200C | CardRole |

## Layout

- `SdCardCore` + `ISdMedia` + `SdCardSimDevice` (BME680-style)
- `SdLoggerCore` (queue / seq / time / drop)
- `RecordChannels.hpp` — MPU6050 / DHT22 / GNSS record-only packers
- `SdLogger` / `SdLoggerSim` F´ shells

## Tests

```bash
./test/run_host_tests.sh
python3 test/dict_id_selftest.py
# with fprime vendored:
fprime-util check
```
