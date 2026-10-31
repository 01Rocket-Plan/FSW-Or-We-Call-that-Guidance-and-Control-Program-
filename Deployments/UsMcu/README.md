# UsMcuDeployment

US 飛控 MCU（獨立板）。**不含控制律**：不安裝 Controller、ActuatorMgr、ServoDriver（尾翼）。

## 元件（規劃）
- MCU：Platform、FlightMode（空殼；分離相關路徑）
- 感測：UsMpu6050、UsDht22
- 分離：SepServoDriver（4×MG996R）
- 紀錄／電源／無線：SdLogger、PowerMonitor、NvFlags、TlmSend、Framer、ComQueue、UplinkAuth

## 拓樸佔位
- 速率群組（rg1kHz／rg100Hz／rg1Hz lockstep）與 base id：TODO（MCU 組）。
- 相機不併入本部署（見 `Deployments/UsCam`）。
