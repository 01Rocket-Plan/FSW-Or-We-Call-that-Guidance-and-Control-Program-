# UplinkAuth — WhitelistGate（查表閘）

對齊：`wireless-link-v0.1.md` §8／§12.5，以及 **勘誤 v0.1a**（HMAC 預設全驗、僅 US 雲台豁免）。

## 目的
以可設定表執行 `opcode × mode × vehicle` 查表與參數範圍檢查，輸出
`ALLOW / DENY / AUTH_REQUIRED`。表內容屬 MCU／FlightMode；本模組不發明政策、
不修改 FlightMode 狀態機。

- **不實作 HMAC**、不讀金鑰（HMAC／防重放見組內另案＋勘誤 §6）。
- **Deny-by-default**：未知 opcode → DENY。
- 參數長度與（可選）範圍檢查；失敗 → DENY。
- 勘誤 v0.1a：HMAC 例外（預設僅 US 雲台 opcode，**待 Edward 定**）在閘內以
  `hmacExempt` 列表示，**仍須**通過本白名單與參數範圍；host 單測可在無 HMAC
  塊時直接 ALLOW。整合時由 HMAC 塊對豁免／驗過的指令設 `authVerified=true`。
- 其餘列需 `authVerified` 才 ALLOW；否則 `AUTH_REQUIRED` 且不轉發。

## 飛行模式（2026-11-10，issue #37-2）
`CmdView.mode` 直接用 `RocketTypes.FlightModeId`（FPP 自動碼，18 個值，對齊 SSS）。白名單列的 `modeMask` 只分兩類：
`ModeClass::PAD`（PAD_IDLE／ARMED／HANGFIRE）與 `ModeClass::AFTER_LIFTOFF`（其他），由 `modeClassOf()` 映射。
`US_WAIT`（上級分離前等待）涵蓋墊上與 DS 推進／滑翔，上級無法自行分辨，故歸 `AFTER_LIFTOFF`（fail-safe）。

## 路徑
`Components/Wireless/UplinkAuth/whitelist/`

## 測試
```bash
tools/verify/fprime_verify.sh --ut Components/Wireless/UplinkAuth   # 需要 RocketTypes 自動碼，改用真 F´ UT
```
