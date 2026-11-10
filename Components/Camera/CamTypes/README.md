# CamTypes（相機共用型別）

- `CamTypes.fpp`：`CamState`、`CamFault`、`CamModel`（`OV7670`、`ESP32_CAM`）、`CamRole`、`CamChunkFlag`、`CamStatus`、影像分塊埠 `CamImageOut`、`CAM_IMAGE_CHUNK_MAX`
- `CamFrameView.hpp`：幀來源借用視圖（指標＋長度，FPP 表達不了，所以是 C++）
- 模組名：`Components_Camera_CamTypes`。用到這些型別的模組在 `MOD_DEPS` 加它。

## 規則（issue #36，Edward 2026-11-10）
**用 FPP 自動產生的型別，不准手寫同名 C++ 型別。** C++ 端這樣用：

```cpp
#include "Components/Camera/CamTypes/CamStateEnumAc.hpp"
CamState s = CamState::READY;          // 列舉值是 CamState::T
if (s == CamState::FAULT) { ... }
switch (s.e) { case CamState::OFF: ... }
CamStatus st(s, CamFault::NONE, 0u, CamModel::OV7670, CamRole::US_CAM);  // 結構用建構子／get_／set_
```
