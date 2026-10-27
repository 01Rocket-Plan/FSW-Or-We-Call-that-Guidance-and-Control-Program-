# 01RocketPlan 飛控軟體（F´）

兩級模型火箭（100 m 構型，Ø80×1100 mm）的 F´ 飛控。DownStage（DS）與 UpStage（US）各一顆 ESP32，同一份原始碼、兩個部署。

## 結構
- `RocketTypes/`：共用型別與埠（介面合約 v0.1）
- `Components/<組>/<元件>/`：各組元件
- `Deployments/DownStage`、`Deployments/UpStage`
- `docs/`：協作規則與慣例

## 慣例
ENU 世界系；機體 X 前／Y 右／Z 上；Hamilton 純量在前四元數；SI 單位加後綴；時間只來自 F´ 時間服務。

測試順序：SIL → PIL → HIL。
