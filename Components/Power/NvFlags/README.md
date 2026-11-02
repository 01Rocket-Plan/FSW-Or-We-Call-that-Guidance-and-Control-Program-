# NvFlags

- 負責：電源組（Zamir）
- 範圍：ESP32 NVS 非揮發旗標（sepReleased / sepCommanded / dsChuteCommanded / phase）
- Base ID：`0x6100`（Sim `0x6180`）
- 狀態：v0.1 host-tested core；ESP32 NVS backend 未實測

## Ports
- `sepReleasedSet` (guarded, U8 mask bit0..3) — OR + immediate commit
- `sepReleasedGet` (sync) — RAM cache
- `sepCommanded` / `dsChuteCommanded` — set-only latches
- `phaseGet` / `phaseSet` — dual-slot + CRC；相同值跳過寫入
- Events：`NvSepLatched`、`NvWriteFail`

## Host tests
See `../run_host_tests.sh`.
