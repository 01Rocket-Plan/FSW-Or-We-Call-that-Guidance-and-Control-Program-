# UplinkAuth 組內實作說明（HMAC＋防重放）

> 本檔不含任何金鑰、salt 或真實測試向量。  
> 飛行 PSK **只**經外部設定／NVS 介面（`IAuthNvStore::readPsk`）載入，程式內無真實金鑰。  
> Host 測試 PSK 為**明顯假值**（`FAKE_TEST_PSK_DO_NOT_USE_IN_FLIGHT!!`），經假 NVS 注入。

## 待 Edward 定（provisional）

1. **`kProvisionalAbortOpcode`**（目前 `0x0001`）：具名常數；**非** HMAC 豁免，預設仍要 HMAC。**待 Edward 定**。
2. **`kProvisionalGimbalOpcode`**（US 雲台舵機控制）：預設 HMAC **豁免表唯一**項目。**待 Edward 定**。
3. 豁免表內容可 `setHmacExemptTable`／`addExempt` 調整；未知 opcode **永遠**要 HMAC（deny-by-default）。

## HMAC 政策（deny-by-default）

- 預設：**所有** opcode 都要 HMAC。
- 僅豁免表內 opcode 可跳過 HMAC。
- **HMAC 豁免 ≠ 白名單放行**：豁免的雲台指令仍必須通過下游 **§8 白名單**與**參數範圍檢查**；本層只決定是否驗 tag。

## 處理順序

1. 幀 CRC、來源／目標 — 上游負責。
2. `cmdCounter == 0xFFFFFFFF` → `CounterInvalid`。
3. 防重放（無 window）：`cmdCounter` **嚴格大於** 最後值。
4. 若 `requiresHmac(opcode)`（非豁免）：驗 HMAC；否則跳過。
5. Persist `last` 至 NVS **之後**才 `onForward`；NVS 失敗 → `StoreFail`、不 forward。
6. §8 白名單／參數範圍在本 Core 之外（下游）。

## MAC 輸入（大端）

```
authTag = HMAC-SHA256(
  psk,
  domainTag || vehicleId_u8 || cmdCounter_be32 || opcode_be16 || argLen_be16 || args
)
```

`domainTag` = `"01RP-UPL-v1"`。標籤 32 bytes；常數時間比較。

## 測試金鑰政策

- 僅假常數經 NVS 假實作載入；不得提交 `*.key`／`secrets*`／真實向量。
