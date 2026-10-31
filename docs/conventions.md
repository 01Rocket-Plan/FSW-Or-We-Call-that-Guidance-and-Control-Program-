# 慣例（conventions）

> 依據：介面合約 v0.1＋勘誤 v0.2（E1–E8）。要改請先提首席工程師。

## 1. 命名
- 元件目錄：`Components/<組>/<元件>/`，元件型別 UpperCamelCase（`GnssDriver`），實例 lowerCamelCase（`gnssDriver`）。
- 埠：`xxxIn`／`xxxOut`；型別放 `RocketTypes/`。
- 變數帶單位後綴：`_m _mps _mps2 _rad _rads _s _V _A _Pa`；感測器原生軸再加 `_sensor`。度只出現在 UI 與報告。
- 分支 `<組>/<bot名>/<主題>`；commit／PR 標題 `[組名][Bot名] 摘要`。

## 2. 座標與時間
- 世界系 ENU；機體系 Z 沿箭體長軸指向鼻錐（平行導軌）、X／Y 為兩個橫向軸（右手系）；四元數 Hamilton (w, x, y, z)。
- 機體→IMU 常值 DCM 放 prmDb；軸不合只改 DCM，不改控制律符號。
- 發射台靜置比力（機體系）＝ (0, 0, +1 g)（Edward 2026-10-27 定案）。
- 時間只來自 F´ 時間服務；不得 sleep、不得讀系統時間。所有資料年齡由時間服務算（E4）。

## 3. 共用型別（`RocketTypes/RocketTypes.fpp`）
- 共用標頭 `SampleHeader { t_s: F64, seq: U32, valid: bool }`，所有樣本與命令都用（E2、E5、E8）。
- 握手狀態 `OFF/INIT/SELFTEST/READY/FAULT` 只走遙測，不放進樣本；只有 READY 時 `valid` 可為 true（E1）。
- `GnssFix`：`posValid`、`velHValid`、`velUValid`；缺垂直速度時 U 軸填 NaN。不接 estimator。
- `BaroSample` 不帶年齡；estimator 經 `StateOut.baroAge_s` 送出（E4）。氣壓只供速度／高度，不直接觸發轉移。
- `EnvSample`（濕度、氣體）只送 Recorder。
- US MPU6050 例外輸出原生軸，欄位加 `_sensor`，安裝四元數寫入設定與紀錄標頭（E3）。
- `StateOut` v1.0；姿態不可信時控制器令尾翼鎖中位。
- `ServoCmd`：µs 只出現在 actuatorMgr 之後，硬限 1000–2000 µs，50 Hz 零階保持。欠壓鎖中位由 actuatorMgr 經 `lockNeutral` 下令，驅動收到 `UvState` 只記事件（E6）。
- `SepCmd`：帶 `valid`；SEP 通道逾時或故障一律不動，每通道只釋放一次並鎖在 nvFlags（E7）。

## 4. 部署
四份獨立部署（同一份元件庫）：
- `Deployments/DsMcu`（DsMcuDeployment）：DS 飛控 MCU，含控制律；Core 1 只跑 1 kHz 鏈與 flightMode 快速路徑，其餘在 Core 0。
- `Deployments/UsMcu`（UsMcuDeployment）：US 飛控 MCU，**不含**控制律（無 Controller／ActuatorMgr／尾翼 ServoDriver）。
- `Deployments/DsCam`（DsCamDeployment）：ESP32-CAM 獨立板，不併入 DsMcu。
- `Deployments/UsCam`（UsCamDeployment）：OV3640 獨立板，不併入 UsMcu。

## 5. Base ID 分配（暫定，MCU 組可提修改）
| 範圍 | 用途 |
|---|---|
| 0x0000–0x0FFF | F´ 標準元件（cmdDisp、eventLogger、prmDb、rateGroup…） |
| 0x1000–0x1FFF | MCU 組（Platform 0x1000、Estimator 0x1100、Controller 0x1200、ActuatorMgr 0x1300、FlightMode 0x1400） |
| 0x2000–0x2FFF | 紀錄組（SdLogger 0x2000） |
| 0x3000–0x3FFF | 伺服組（ServoDriver 0x3000、SepServoDriver 0x3100、雲台實例 0x3200） |
| 0x4000–0x4FFF | 環境感測（ImuDriver 0x4000、BaroDriver 0x4100、UsMpu6050 0x4200、UsDht22 0x4300） |
| 0x5000–0x5FFF | GNSS（GnssDriver 0x5000） |
| 0x6000–0x6FFF | 電源（PowerMonitor 0x6000、NvFlags 0x6100） |
| 0x7000–0x7FFF | 相機（DsCam 0x7000、UsCam 0x7100） |
| 0x8000–0x8FFF | 無線（TlmSend 0x8000、Framer 0x8100、ComQueue 0x8200、UplinkAuth 0x8300） |
Sim 驅動用同元件基底 +0x80。四部署可重用同一表（相機部署只用 0x7000 段）。

## 6. 禁止事項
- 不得有實際作用的 FTS 輸出腳位。
- 不得入庫任何密鑰（中止 HMAC 金鑰由 Edward 地面設定）。
