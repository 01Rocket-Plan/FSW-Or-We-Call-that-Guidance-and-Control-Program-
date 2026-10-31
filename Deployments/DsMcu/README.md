# DsMcuDeployment

DS 飛控 MCU（獨立板），含控制律。

## 元件（規劃）
- MCU：Platform、Estimator、Controller、ActuatorMgr、FlightMode（空殼）
- 感測：ImuDriver（ICM20948）、BaroDriver、GnssDriver
- 致動：ServoDriver（4×SG90 尾翼）
- 紀錄／電源／無線：SdLogger、PowerMonitor、NvFlags、TlmSend、Framer、ComQueue、UplinkAuth

## 拓樸佔位
- Core 1（APP_CPU）：1 kHz 鏈 `imuDriver → estimator → controller → actuatorMgr → servoDriver`，以及 flightMode 快速路徑。
- Core 0：WiFi／無線與其餘元件；跨核用 seqlock 與非阻塞佇列。
- Topology.fpp／base id：TODO（MCU 組）。相機不併入本部署（見 `Deployments/DsCam`）。
