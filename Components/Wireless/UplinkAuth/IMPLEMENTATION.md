# UplinkAuth 組內實作說明（HMAC＋防重放）

> 本檔不含任何金鑰、salt 或真實測試向量。真實 PSK 只存飛控 NVS，由 Edward 地面寫入。  
> Host 測試 PSK 為**明顯假值**（`FAKE_TEST_PSK_DO_NOT_USE_IN_FLIGHT!!`），非飛行金鑰。

## 待 Edward 定（provisional）

下列皆為暫定參數，**待 Edward 定**後再凍結進 prm／任務設定：

1. **HMAC 必驗 opcode 表**（`OpcodeAuthTable`）：預設僅含一筆 `kProvisionalHmacOpcode0`。可 `setHmacTable`／`addOpcode` 擴充。
2. **該表預設第一筆 opcode 數值**：僅存於 `OpcodeAuthTable::kProvisionalHmacOpcode0`（參數表常數；呼叫端不得寫死字面量）。語意與最終值 **待 Edward 定**。

## 處理順序

1. 幀 CRC、來源／目標 — 上游負責。
2. `cmdCounter == 0xFFFFFFFF` → `CounterInvalid`。
3. 防重放（無 window）：`cmdCounter` **嚴格大於** 最後值，否則 `Replay`。
4. 若 `hmacTable().requiresHmac(opcode)`：驗證 HMAC；否則跳過。
5. **Persist before forward**：先寫 NVS；失敗 → `StoreFail`、不 forward；成功後才 `onForward` → `Ok`。
6. §8 白名單在本 Core 之外。

## MAC 輸入（大端）

```
authTag = HMAC-SHA256(
  psk,
  domainTag || vehicleId_u8 || cmdCounter_be32 || opcode_be16 || argLen_be16 || args
)
```

標籤 32 bytes；常數時間比較。

## 測試金鑰政策

- 僅用假常數；不得提交 `*.key`／`secrets*`／真實向量。
