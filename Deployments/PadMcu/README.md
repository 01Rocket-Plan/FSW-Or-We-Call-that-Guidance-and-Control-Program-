# PadMcuDeployment

發射架控制板（ESP32，獨立板），**不上箭**。Issue #24 新增的第五份部署。

## 元件（規劃）
- 無線：`PadController`（0x8700）— 發射許可互鎖、待點火倒數、點火窗、推力確認後鬆夾持、離軌判斷
- 驅動（待開 PR）：NRF24L01（SPI，架 ↔ 地面）、臍帶 UART（箭 ↔ 架）、GPIO／ADC（seated、迴路連續性、鑰匙讀回、離軌開關、推力、傾角、架電源）、點火繼電器與夾持輸出
- 電源：12 V 繼電器電源給點火與夾持（xmind「DC12V／X0122」）

## 鏈路
- 箭 ↔ 架：有線臍帶 UART 雙向（箭送 READY／BIT，架回送架狀態），起飛時拔脫（預期中斷，不報錯）
- 架 ↔ 地面：NRF24L01 2.4 GHz
- 點火指令只走地面 → 架；火箭不在點火迴路裡（gate C15）

## 拓樸佔位
- timer → RateGroupDriver → rg100Hz：`padController.schedIn`
- `nrf24Driver.recv → padController.radioIn`；`padController.radioOut → nrf24Driver.send`
- `umbilicalUart.recv → padController.umbilicalIn`
- `padController.ioGet → padIo`；`padController.actOut → padIo`
- Topology.fpp／base id：TODO（MCU 組；無線段 0x8500–0x87FF 已預留）

## 安全
- 架上不准裝真馬達點火直到 GATE-2（gate H11）；地面先用假負載測聯鎖與連續性。
- 實體鑰匙在硬體上串聯點火迴路；軟體只是第二道。
