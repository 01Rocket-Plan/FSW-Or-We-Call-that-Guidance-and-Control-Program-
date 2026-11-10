# 協作規則

1. 動手前先看團隊 Google Drive「01RocketPlan專案須知」資料夾。
2. 每個 Bot 一條分支：`<組>/<bot名>/<主題>`，例：`servo/evgeny/cmdIn`。
3. `main` 受保護，只能經 PR 合併；不得直接推送或強推。
4. Commit 與 PR 標題格式：`[組名][Bot名] 摘要`，例：`[伺服][Evgeny] 新增 cmdIn 埠`。
5. 開 PR 前 `fprime-util check`（或 `fpp-check`）需通過；單元測試保持輕量即可。
6. 不得寫入任何密鑰（含中止 HMAC 金鑰），不得修改 Edward 的驗證工具。
7. `Components/MCU/FlightMode` 只有介面，狀態機邏輯由 Edward 撰寫。
8. 要改共用型別（`RocketTypes/`）先提給首席工程師。

9. 部署分五份：`DsMcu`／`UsMcu`／`DsCam`／`UsCam`／`PadMcu`；US MCU 不含控制律；相機不併入 MCU 主幹；`PadMcu` 是發射架板，不上箭。
10. 自測保持輕量（`fpp-check`／`fprime-util check` 即可），勿過度著墨單元測試篇幅。
11. **本機改完再推，一功能一 PR（Edward，2026-11-01）：** 不要在 GitHub 網頁介面直接改檔，也不要頻繁推送零碎小改動（會生出大量重複 PR、浪費平台儲存）。請在自己的機體上改完並自測後，再一次上傳最新版本、每個功能只開一個 PR；後續修正請推到同一條分支／同一個 PR。
12. **禁止拿 stub 當最終交差結果（Edward，2026-11-10，取代第 5、10 條的「`fpp-check` 即可」）：** 最終 PR 前必須用真 F´（`lib/fprime` v4.4.1）跑 `tools/verify/fprime_verify.sh` 通過，並把結果貼在 PR 內文。`fpp_stubs/`、手寫假自動碼標頭、只編 Core 的 host 測試都只算開發中自測。見 `docs/REAL_FPRIME_VERIFICATION.md`。
