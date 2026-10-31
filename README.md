# 01RocketPlan 飛控軟體（F´）

兩級模型火箭（100 m 構型，Ø80×1100 mm）的 F´ 飛控。同一份元件庫、四份部署：DS／US 各一顆 MCU，相機各自獨立板。

## 結構
- `RocketTypes/`：共用型別與埠（介面合約 v0.1）
- `Components/<組>/<元件>/`：各組元件
- `Deployments/DsMcu`、`Deployments/UsMcu`、`Deployments/DsCam`、`Deployments/UsCam`
- `docs/`：協作規則（CONTRIBUTING.md）與慣例（conventions.md）

## 部署分工
| 部署 | 板子 | 控制律 | 備註 |
|---|---|---|---|
| DsMcu | DS 飛控 MCU | 有（Controller／ActuatorMgr／ServoDriver） | 1 kHz 即時鏈 |
| UsMcu | US 飛控 MCU | **無** | 感測、紀錄、分離舵機 |
| DsCam | ESP32-CAM | — | 不併入 DsMcu |
| UsCam | OV3640 自有板 | — | 不併入 UsMcu |

## 慣例
ENU 世界系；機體 Z 沿箭體長軸指向鼻錐（平行導軌）、X／Y 為兩個橫向軸（右手系）；Hamilton 純量在前四元數；SI 單位加後綴；時間只來自 F´ 時間服務。

測試順序：SIL → PIL → HIL。自測保持輕量即可。
