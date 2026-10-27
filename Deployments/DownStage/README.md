# DownStageDeployment

- Core 1（APP_CPU）：只跑 1 kHz 即時鏈 imuDriver → estimator → controller → actuatorMgr → servoDriver，以及 flightMode 快速路徑。
- Core 0：WiFi／無線鏈路與其他元件。跨核用 seqlock 與非阻塞佇列。
- 拓樸與 base id：TODO（MCU 組）。
