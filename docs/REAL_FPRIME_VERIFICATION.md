# 交差規則：一律用真 F´ 檢驗（禁止拿 stub 當最終結果）

> Edward，2026-11-10（issue #34）：「禁止拿 stub 當最終交差 PR 結果，所有的最終 PR 結果都必須經過正統的 F´ 檢驗通過才能上傳。」
>
> 本文件**取代** `docs/CONTRIBUTING.md` 第 5、10 條裡「`fpp-check` 即可」的說法。

## 規則

1. **最終 PR（標「可審」、請 Edward 審）之前，必須用真 F´ 建過一次**，並把結果貼在 PR 內文：
   ```bash
   . <你的 venv>/bin/activate            # fprime-tools==4.4.0、fprime-fpp==3.4.0
   git submodule update --init lib/fprime  # F´ v4.4.1
   tools/verify/fprime_verify.sh                    # 整個 repo
   # 或只建自己的模組（依賴一起列）：
   tools/verify/fprime_verify.sh Components/Power/NvFlags Components/Power/PowerMonitor
   ```
   最後一行要是 `== 通過：configure＋build 成功`。
2. **「真 F´」＝ `lib/fprime` 子模組（v4.4.1）**，用 `fprime-tools`／`fprime-fpp` 自動產生的程式碼去編你的 `.cpp`。
   下面這些**都不算**：
   - `fpp_stubs/` 的假 `Fw`／`Svc`／`Drv` 定義搭配 `fpp-check`
   - `test/host_ac/` 之類**手寫的假自動碼標頭**
   - 只編純 C++ 核心（Core）的 host 測試
3. stub 和 host 測試**可以留著**，當開發中的快速自測；但它們只證明「核心邏輯對」，**不**證明「元件能進 F´」。
4. PR 內文要寫：用哪個指令驗的、框架版本（腳本第一行會印）、結果。沒有這段，審查者可以直接退回。
5. 同時改到別組介面（例如埠簽名）時，要把**受影響的模組一起列進去**再跑一次。

## 為什麼（2026-11-10 實際踩到的 5 個洞）

PR #31 第一次用真 F´ 建整個 repo，馬上抓到 5 個 stub 完全看不到的錯：

| issue | 錯誤 | stub 為什麼沒抓到 |
|---|---|---|
| #32 | 同一個埠 `Esp32CamImageOut` 定義兩次 → **整個 repo configure 失敗** | `fpp-check` 只檢查命令列給它的檔案，沒有把兩組的 `.fpp` 放在一起看 |
| #33 | `Pwm`／`ModelCores` 找不到自己的標頭 | host 測試用 `-I` 自己加路徑；真 F´ 的 include 路徑來自 `MOD_DEPS`，沒寫就沒有 |
| #34 | `EnvSensingBus` 未定義 | `.fpp` 沒進任何 CMake 模組；stub 是把檔案直接列在命令列 |
| #35 | handler 簽名 `PowerRailId` 應為 `const PowerRailId&` | 真自動碼的虛擬函式簽名跟手猜的不同；F´ 外殼 `.cpp` 從來沒被編過 |
| #36 | 手寫 `CamTypes.hpp` 和 FPP 自動碼重複定義 | 用假的 `host_ac` 標頭代替自動碼，假的跟假的當然對得上 |

共同點：**stub／假標頭是自己寫的，所以它們和自己的程式永遠一致**。只有真框架產生的程式碼才是「對方」，才測得出介面對不對。

## 常見錯誤速查

| 症狀 | 原因 | 修法 |
|---|---|---|
| configure 時 `inconsistent location path` | 同一個 FPP 符號在兩個檔案定義 | 只留一份，另一邊用 `MOD_DEPS` 依賴擁有它的模組 |
| configure 時 `symbol X is not defined` | 定義 X 的 `.fpp` 沒進任何模組 | 把那個 `.fpp` 加到某個模組的 `SOURCE_FILES`，用到的模組 `MOD_DEPS` 它 |
| `fatal error: Components/.../X.hpp: No such file` | 模組沒有 `MOD_DEPS` → F´ 不給 include 路徑 | `set(MOD_DEPS Fw_Types)`（或真正依賴的模組） |
| `marked 'override', but does not override` | handler 簽名跟自動碼不同 | 打開 `build/.../XxxComponentAc.hpp` 照抄簽名（列舉／結構參數是 `const T&`） |
| `redefinition of 'class Components::X'` | 手寫型別和 FPP 型別同 namespace 同名 | 用 FPP 產生的型別，刪掉手寫的 |

## 環境（一次設定）

```bash
python3 -m venv ~/fppenv && . ~/fppenv/bin/activate
pip install fprime-tools==4.4.0 fprime-fpp==3.4.0
git submodule update --init lib/fprime
```
Windows 請用 WSL（Ubuntu 22.04 驗過：g++ 11.4、cmake 3.22、Java 給 fpp 用）。
repo 放在 `/mnt/c/...` 也可以跑，但建議 clone 到 WSL 自己的檔案系統（快很多，框架可以用硬連結）。
