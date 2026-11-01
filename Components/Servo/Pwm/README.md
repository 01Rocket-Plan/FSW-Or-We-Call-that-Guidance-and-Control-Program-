# ServoPwm（LEDC HAL）

- ESP32：`driver/ledc.h`，**high-speed**、共用 timer 0、50 Hz、14-bit；脈寬單位 µs。
- Host：stub，`start()` 在 `configure` 後成功。
- 腳位／通道：`ServoPwmPins.hpp`（**MCU 組 v0.1 草案，pending Anatoly review**）。
- 禁止 low-speed 通道與 timer 1–3。
- 不 sleep、不取系統時鐘。
