# SepServoDriver

- 負責：伺服組（Evgeny）
- 範圍：MG996R SEP_1–4（DsMcu），真驅動；NvFlags 經 `SepNvPlaceholder`（TODO Power v0.1）
- Sim／單元測試：子Bot1
- 狀態：實作中（feat/servo-drivers）
- PWM：共用 `Components/Servo/Pwm`；SEP 腳位 `ServoPwmPins::SEP_*`（TODO MCU）
