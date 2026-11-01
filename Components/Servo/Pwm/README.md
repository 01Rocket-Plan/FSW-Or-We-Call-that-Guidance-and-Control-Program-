# ServoPwm（LEDC HAL）

- ESP32：`driver/ledc.h`，預設 50 Hz、14-bit duty，脈寬單位 µs。
- Host：stub，`start()` 在 `configure` 後成功，不碰 GPIO。
- 腳位表：`ServoPwmPins.hpp`（倉庫無正式腳位 → 皆 `kUnassigned`，TODO MCU／配線）。
- 不 sleep、不取系統時鐘。
