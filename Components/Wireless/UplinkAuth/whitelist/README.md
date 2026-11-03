# UplinkAuth — WhitelistGate（查表閘）

## 目的
以 `opcode × mode × vehicle` 查可設定白名單表，強制執行
`ALLOW / DENY / AUTH_REQUIRED`。表內容屬 MCU／FlightMode；本模組不發明政策、
不修改 FlightMode 狀態機。

- **不實作 HMAC**、不讀金鑰。
- **Deny-by-default**：未知 opcode 一律 DENY。
- 參數長度與（可選）範圍檢查；失敗 → DENY。
- HMAC 豁免列（預設僅 US 雲台 opcode，**待 Edward 定**）可不帶 `authVerified`
  放行，但**仍須**通過本白名單與參數範圍。
- 其餘列需 `authVerified` 才 ALLOW；否則回 `AUTH_REQUIRED` 且不轉發。

## 路徑
`Components/Wireless/UplinkAuth/whitelist/`

## 測試
```bash
./Components/Wireless/UplinkAuth/whitelist/run_host_tests.sh
```
