# 密鑰放哪裡（CONTRIBUTING 第 6 條）

本 repo 是 **public**。飛行用的上行 HMAC 金鑰、中止密碼一旦進 repo（就算之後刪掉，git 歷史與 GitHub 快取都還在），任何人都能偽造上行指令：中止、改參數、轉雲台。

## 規則
| 東西 | 放哪 |
|---|---|
| 驗證程式、金鑰載入程式、opcode 表、白名單 | repo |
| 測試金鑰、測試向量（名字或註解標 `TEST`／`DEV`） | repo |
| **飛行金鑰、飛行中止密碼** | **本機 `secrets/`**（`.gitignore` 已擋 `secrets*`） |
| 第三方 API key | 哪裡都不准 |

## 本機檔案格式（提案，UplinkAuth 實作時照這個讀）
```
secrets/
  uplink_hmac_key.hex      # 64 個十六進位字元（32 B），單行
  pad_hmac_key.hex         # 發射架 PadCmdGate 用，同格式
```
- SIL：部署啟動時從 `secrets/` 讀；檔案不存在 → 用 `TEST` 金鑰並發 `WARNING_HI` 事件，**拒絕進 ARMED**。
- ESP32：燒錄前由 Edward 在本機把 `secrets/` 寫進 NVS 加密分區，不經 repo、不經 CI。

## 如果 Edward 要讓飛行金鑰進 repo
先把 repo 改成 **private**，再改這份文件與 CONTRIBUTING 第 6 條。public 狀態下不要做。
