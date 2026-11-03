# UplinkAuth

- 負責：無線組子Bot2（上行與地面）
- Base ID：0x8300
- 範圍：防重放（strict >，禁 0xFFFFFFFF）＋可配置 HMAC 必驗 opcode 表（預設一筆，**待 Edward 定**）
- 測試 PSK 為假值；真實金鑰不入庫
- 無實體射頻硬體 → 無 FPP 硬體型號宣告

```bash
./Components/Wireless/UplinkAuth/run_host_tests.sh
```

見 `IMPLEMENTATION.md`、`OpcodeAuthTable.hpp`。
