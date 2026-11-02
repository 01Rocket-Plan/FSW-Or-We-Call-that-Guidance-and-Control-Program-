# Servo SimCore（伺服組子Bot1）

ServoDriverSim／SepServoDriverSim 的無框架核心與 host 測試（規格：伺服組 v0.4、電源組 Power.SepReleasedGet/Set（NvFlagsPorts））。
- `ServoSimCore`：尾翼 4ch／雲台 2ch（gimbalServoDriverSim 實例 channel_count=2）
- `SepSimCore`：SEP_1..4，只能釋放一次，NvFlags 用 mock
- 測試：`CXX="python3 -m ziglang c++" Components/Servo/run_sim_host_tests.sh`
- F´ 外殼：`ServoDriverSim/`、`SepServoDriverSim/`，`.fpp` 由 feat/servo-drivers 真驅動複製改名（埠／參數／事件／遙測逐行相同）。
- 啟用需在 `Components/CMakeLists.txt` 加 `Servo/SimCore`、`Servo/ServoDriverSim`、`Servo/SepServoDriverSim` 三行。
