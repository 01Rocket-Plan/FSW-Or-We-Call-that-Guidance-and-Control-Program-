# UsMcuDeployment

US 飛控 MCU（獨立板）。**不含控制律**：不安裝 Controller、ActuatorMgr、ServoDriver（尾翼），也不負責級間分離（分離時機判斷、SEP_FAIL 與分離供電在 DS，見 gate A9）。

## 元件（規劃）
- MCU：Platform、FlightMode（空殼；只管上級自己的傘、紀錄、相機）
- 感測：UsMpu6050、UsDht22
- 雲台：ServoDriver 的 US 雲台實例（base 0x3200），2×SG90 GIMBAL_PITCH／ROLL；地面站可上行調整兩軸角度帶相機轉向。只是景觀用途，不進控制律；雲台指令只有 US 執行（DS 解析器看到就丟），也不影響開傘計時（合約 v0.1、gate B15／C14）
- 紀錄／電源／無線：SdLogger、PowerMonitor、NvFlags、TlmSend、Framer、ComQueue、UplinkAuth

## 拓樸佔位
- 速率群組（rg1kHz／rg100Hz／rg1Hz lockstep）與 base id：TODO（MCU 組）。
- 相機不併入本部署（見 `Deployments/UsCam`）。
