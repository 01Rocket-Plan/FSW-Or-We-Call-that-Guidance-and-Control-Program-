# UmbilicalTx

- 負責：無線組（Edward）— Issue #24
- 部署：`DsMcu`（臍帶在下級箭尾）
- Base ID：`0x8500`
- 範圍：經有線臍帶 UART 單向送 RocketStatus（模式、BIT、ready、電池電壓）給發射架，10 Hz
- 起飛後臍帶拔脫是預期行為：寫入失敗只計數，不報錯、不影響飛行
- 設計見 `../PadLink/README.md`
