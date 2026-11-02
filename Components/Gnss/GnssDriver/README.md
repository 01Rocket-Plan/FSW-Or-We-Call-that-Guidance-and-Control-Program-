# GnssDriver

- 範圍：TEL0157 `gnssDriver`（真驅動）＋ `gnssDriverSim`，`fixOut`（GnssFix v0.2）
- 安裝位置：DS／US 皆可（拓撲決定）
- Base ID：`0x5000`（docs/conventions.md）；Sim `0x5080`。先前暫定 `0x0A00` 已作廢。
- 核心 `GnssCore` 與 F´ 無關，可主機單測；UART 適配為 ByteStream 推入 + 每 tick 一次 `readBytes`。
