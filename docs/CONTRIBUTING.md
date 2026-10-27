# 協作規則

1. 動手前先看團隊 Google Drive「01RocketPlan專案須知」資料夾。
2. 每個 Bot 一條分支：`<組>/<bot名>/<主題>`，例：`servo/evgeny/cmdIn`。
3. `main` 受保護，只能經 PR 合併；不得直接推送或強推。
4. Commit 與 PR 標題格式：`[組名][Bot名] 摘要`，例：`[伺服][Evgeny] 新增 cmdIn 埠`。
5. 開 PR 前 `fprime-util check` 需通過，每個元件附單元測試。
6. 不得寫入任何密鑰（含中止 HMAC 金鑰），不得修改 Edward 的驗證工具。
7. `Components/MCU/FlightMode` 只有介面，狀態機邏輯由 Edward 撰寫。
8. 要改共用型別（`RocketTypes/`）先提給首席工程師。
