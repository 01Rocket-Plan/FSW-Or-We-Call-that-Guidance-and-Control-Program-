# UvState — DeepSeek 草稿（未審查）

**狀態：未審查 DeepSeek 草稿。未接 F´ 完整建置；僅可能做過 host 語法檢查。**

## 投放路徑
- 建議放入 `Components/Power/PowerMonitor/`（骨架 README 已列 UvState／卸載等級），
  或新增 `Components/Power/UvState/` 並註冊 CMake。
- `RocketTypes.fpp` 已有 `enum UvState` 與 `UvStatePort`；請對齊共用型別，避免重複定義衝突。
  草稿核心用 `UvLevel` 避開與 FPP 列舉名稱衝突。

## Base ID
- **TODO（由電力組確認）**。若與 PowerMonitor 同元件可共用 `0x6000`；若獨立元件請在 conventions 段內另編。

## 內容
- FPP 元件（UvStateComp / PowerUvState 等命名以草稿為準）+ Core + host 測試
- L1／L2 卸載請求埠（不下令舵機／尾翼鎖定）
- 門檻／dwell／遲滯皆為參數占位，**正式數值由 prmDb／組內決定**

## 注意
- valid=false 樣本不計入 dwell。
- 產生時間：2026-11-02（台北時間）。

## Host quick-check (2026-11-02)
- zig c++ build OK; UvStateHostTest: FAILED at UV_WARN assert (draft)
