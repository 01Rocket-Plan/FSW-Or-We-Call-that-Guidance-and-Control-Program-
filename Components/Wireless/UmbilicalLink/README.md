# UmbilicalLink

- 負責：無線組（Edward）— Issue #24
- 部署：`DsMcu`（臍帶在下級箭尾）
- Base ID：`0x8500`
- 範圍：有線臍帶 UART **雙向**，10 Hz
  - 箭 → 架：RocketStatus（模式、BIT、ready、電池電壓）
  - 架 → 箭：PadToRocket（架狀態、倒數、點火／推力確認／夾持已鬆、架有沒有收到箭）
- 方案 A：鬆夾持由架決定，箭用 ready 否決；架 → 箭 只是狀態回報，經 `padStatusOut` 給 FlightMode 參考
- 起飛後臍帶拔脫是預期行為：寫入失敗、收不到架狀態只計數，不報錯、不影響飛行
- 設計見 `../PadLink/README.md`
