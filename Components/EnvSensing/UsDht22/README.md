# UsDht22

- 負責：環境感測組（Farhad）
- 範圍：US DHT22（RMT 讀取），僅記錄
- 狀態：**v0.1 可運行**（2026-11-10 更新，原寫「骨架」已過時）
  - `Dht22Core`：時序解碼、校驗和、狀態機（host 可測）
  - `Dht22RmtEsp32`：ESP32 RMT 後端
  - `Dht22SimIo`＋`UsDht22Sim`：SIL 用模擬
  - `UsDht22`：F´ 外殼
  - host 測試：`test/Dht22HostTest.cpp`
- 真 F´ 檢驗：`tools/verify/fprime_verify.sh Components/EnvSensing/UsDht22`
