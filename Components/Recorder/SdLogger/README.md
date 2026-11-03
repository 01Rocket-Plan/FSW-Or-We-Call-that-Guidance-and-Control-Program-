# SdLogger

- 負責：紀錄組（Malcolm）／子Bot1＝遙測與事件字典；子Bot2＝DS/US 實作
- 範圍：DS/US sdLogger、遙測/事件字典、高水位、序號連續
- Base ID：`0x2000`（Sim `0x2080`），見 `docs/conventions.md` §5
- 狀態：FPP 字典已定（本目錄 `SdLogger.fpp`）；寫卡／佇列實作交子Bot2

## SdLogger 頻道／事件 ID（相對 base 0x2000）

見 `../TELEMETRY_EVENT_DICTIONARY.md` 與 `SdLogger.fpp` 註解。
