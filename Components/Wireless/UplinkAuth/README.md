# UplinkAuth

- 負責：無線組子Bot2（上行與地面）
- Base ID：0x8300
- HMAC：**deny-by-default**；豁免表預設僅 US 雲台 opcode（**待 Edward 定**）
- Abort opcode：具名暫定常數 `kProvisionalAbortOpcode`（**待 Edward 定**），仍要 HMAC
- 金鑰只經 NVS／外部設定載入；測試用假 PSK
- HMAC 豁免不是 §8 白名單旁路

```bash
./Components/Wireless/UplinkAuth/run_host_tests.sh
```
