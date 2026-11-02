# NvFlags

- 負責：電源組（Zamir）
- 範圍：ESP32 NVS 非揮發旗標（sepReleased / sepCommanded / dsChuteCommanded / phase）
- Base ID：`0x6100`（Sim `0x6180`）
- 狀態：v0.1 host-tested core；ESP32 NVS backend 未實測

## Ports（具名型別在 `module Power`，見 `NvFlagsPorts.fpp`）
| 實例 | 型別 | 語意 |
|---|---|---|
| `sepReleasedSet` | `Power.SepReleasedSet` (guarded) | U8 mask bit0..3 OR + 立刻 commit |
| `sepReleasedGet` | `Power.SepReleasedGet` (sync) | RAM cache 讀取 |
| `sepCommanded` | `Power.SepCommanded` (guarded) | set-only latch |
| `dsChuteCommanded` | `Power.DsChuteCommanded` (guarded) | set-only latch |
| `phaseGet` | `Power.PhaseGet` (sync) | 讀 phase |
| `phaseSet` | `Power.PhaseSet` (guarded) | dual-slot + CRC；相同值跳過寫入 |

對接端（例如 SepServoDriver）宣告同型別的 `output port` 再在 topology 接線。
Events：`NvSepLatched`、`NvWriteFail`。

> FPP 保留字：`PhaseSet` 的形參名為 `phaseVal`（不可用 `phase`）。

## Host tests
See `../run_host_tests.sh`.
