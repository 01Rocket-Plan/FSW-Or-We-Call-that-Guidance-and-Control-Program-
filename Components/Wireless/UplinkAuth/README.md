# UplinkAuth

- 負責：無線組（上行與地面）
- Base ID：0x8300
- 狀態：骨架＋白名單查表閘（`whitelist/`）
- 白名單政策／表內容屬 MCU／FlightMode；本目錄只做查表與參數範圍強制執行
- HMAC／防重放為組內另案；金鑰不入庫（見 `.gitignore` 的 `*.key`）

```bash
./Components/Wireless/UplinkAuth/whitelist/run_host_tests.sh
```
