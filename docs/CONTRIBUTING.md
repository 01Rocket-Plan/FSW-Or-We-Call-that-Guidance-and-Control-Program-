# 協作規則

1. 動手前先看團隊 Google Drive「01RocketPlan專案須知」資料夾。
2. 每個 Bot 一條分支：`<組>/<bot名>/<主題>`，例：`servo/evgeny/cmdIn`。
3. `main` 受保護，只能經 PR 合併；不得直接推送或強推。
4. Commit 與 PR 標題格式：`[組名][Bot名] 摘要`，例：`[伺服][Evgeny] 新增 cmdIn 埠`。
5. 開 PR 前 `fprime-util check`（或 `fpp-check`）需通過；單元測試保持輕量即可。
6. **密鑰（2026-11-10 Edward 授權放寬，取代舊版「一律不得寫入」）：**
   - **可以寫**：上行驗證程式、金鑰載入路徑、**標明 `TEST`／`DEV` 的測試金鑰與測試向量**、opcode／白名單表、密碼欄位的介面與佔位。
   - **飛行金鑰本身不進本 repo**（本 repo 是 **public**，進來＝全世界都能偽造上行指令）。放在本機 `secrets*`（已在 `.gitignore`），格式見 `docs/key-handling.md`；建置時讀進來。
   - 第三方 API key（DeepSeek 等）任何形式都不得入庫。
   - 不得修改 Edward 的驗證工具。
7. `Components/MCU/FlightMode` 的狀態機邏輯由 Edward 負責。**EdwardWang+Claude 可依 Edward 在 issue／PR 的指示填空（例：「填 FM-12」）**；其他 Bot 仍只能動介面。
8. 要改共用型別（`RocketTypes/`）：開 PR，CODEOWNERS 指定首席工程師審（EdwardWang+Claude 可以直接提 PR，不必先私下提）。

9. 部署分五份：`DsMcu`／`UsMcu`／`DsCam`／`UsCam`／`PadMcu`；US MCU 不含控制律；相機不併入 MCU 主幹；`PadMcu` 是發射架板，不上箭。
10. 自測保持輕量（`fpp-check`／`fprime-util check` 即可），勿過度著墨單元測試篇幅。
11. **本機改完再推，一功能一 PR（Edward，2026-11-01）：** 不要在 GitHub 網頁介面直接改檔，也不要頻繁推送零碎小改動（會生出大量重複 PR、浪費平台儲存）。請在自己的機體上改完並自測後，再一次上傳最新版本、每個功能只開一個 PR；後續修正請推到同一條分支／同一個 PR。
