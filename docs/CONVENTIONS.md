# 慣例
- 座標：世界 ENU；機體 X 前、Y 右、Z 上；機體→IMU DCM 放 prmDb，不改控制律符號。
- 四元數：Hamilton，(w, x, y, z)。
- 單位：SI，後綴 `_m _mps _mps2 _rad _rads _s _V _A _Pa`；度只用於 UI。
- 樣本標頭：`t_s` F64、`seq` U32、`valid` bool（只有 READY 可為 true）。
- 不得 sleep、不得讀系統時間；所有數值放 prmDb。
