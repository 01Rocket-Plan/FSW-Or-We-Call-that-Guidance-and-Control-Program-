# PwmOut（PWM 硬體抽象）

驅動（`ServoDriver`／`SepServoDriver`）**只**透過抽象 `PwmOut` 操作 PWM，**不**直接呼叫 LEDC／TIMx／RP2040 PWM API。

## 抽象介面

| 方法 | 語意 |
|---|---|
| `init(ch, pin, freq_hz)` | 綁定通道腳位與頻率（板級表亦可先 `configure` 再批次 `init(freq_hz)`） |
| `setPulse_us(ch, us)` | 設定脈寬（µs） |
| `disable(ch)` | 關閉該通道輸出（duty=0／停波） |

換板＝只改 **板級設定**（`ServoPwmPins.hpp`／拓樸 `configure`）＋編譯期後端，驅動邏輯不變。

## Backend A（現行）— ESP32 LEDC

- High-speed、共用 timer 0、50 Hz、14-bit（見 `ServoPwmPins`；**MCU 組 v0.1 草案，pending Anatoly review**）。
- Host／CI：stub，`init` 在 `configure` 後成功，不碰 GPIO。
- 禁止 low-speed 通道與 timer 1–3。

## Backend B（設計保留，**程式尚未實作**）— 泛用後備

優先順序：

1. 目標 MCU **硬體 timer PWM**（例：STM32 TIMx、RP2040 PWM slice）
2. 若無 → **timer-ISR 軟體 PWM**：50 Hz、週期 20 ms；後端脈寬範圍 **500–2500 µs**、解析度 **≥ 1 µs**
3. 驅動仍以 prmDb **1000–2000 µs** clamp（與後端範圍解耦）

## 其它約束

- 不 `sleep`、不取系統時鐘（時間只來自 F´ Time）。
- 詳見設計文件 `servo_ports_final_v0.5.md` §7。
