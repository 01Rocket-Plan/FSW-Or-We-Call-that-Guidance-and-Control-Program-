# UplinkAuth

- 負責：無線組子Bot2（上行與地面）
- Base ID：0x8300
- 狀態：HMAC＋防重放核心（`UplinkAuthCore`）＋白名單查表閘（`whitelist/`）
- HMAC：**deny-by-default**；豁免表預設僅 US 雲台 opcode（**待 Edward 定**）
- Abort opcode：具名暫定常數 `kProvisionalAbortOpcode`（**待 Edward 定**），仍要 HMAC
- HMAC 豁免不是 §8 白名單旁路：雲台 `0x0200` 免簽名，但仍須過白名單＋參數範圍
- 白名單政策／表內容屬 MCU／FlightMode；`whitelist/` 只做查表與參數範圍強制執行
- 金鑰只經 NVS／外部設定載入、不入庫（見 `.gitignore` 的 `*.key`）；測試用假 PSK

```bash
./Components/Wireless/UplinkAuth/run_host_tests.sh            # HMAC + anti-replay
./Components/Wireless/UplinkAuth/whitelist/run_host_tests.sh  # whitelist gate
```
