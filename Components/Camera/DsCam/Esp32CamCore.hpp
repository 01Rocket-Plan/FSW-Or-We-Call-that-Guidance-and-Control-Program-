// Esp32CamCore：ESP32-CAM 驅動核心（與 F´ 無關，可在 host 測試）
// 規則：不 sleep、不讀系統時間、不配置記憶體；時間以 tick 由呼叫端注入。
// 狀態機於 1 Hz tick；擷取與 frameCount 於 100 Hz tick。
// ICamLink（I2C）只傳指令／狀態／幀數通知；完整錄影與地面預覽在相機側
// （見 ILocalSdStore／IGroundPreviewLink），不得經 ICamLink 或飛控遙測傳像素。
#ifndef COMPONENTS_CAMERA_DSCAM_ESP32CAMCORE_HPP
#define COMPONENTS_CAMERA_DSCAM_ESP32CAMCORE_HPP

// 型別來自 FPP 自動碼（../CamTypes/CamTypes.fpp，issue #36）
#include "Components/Camera/CamTypes/CamChunkFlagEnumAc.hpp"
#include "Components/Camera/CamTypes/CamFaultEnumAc.hpp"
#include "Components/Camera/CamTypes/CamModelEnumAc.hpp"
#include "Components/Camera/CamTypes/CamRoleEnumAc.hpp"
#include "Components/Camera/CamTypes/CamStateEnumAc.hpp"
#include "Components/Camera/CamTypes/CamStatusSerializableAc.hpp"
#include "Components/Camera/DsCam/ICamLink.hpp"
#include "Components/Camera/DsCam/Esp32CamImageStreamer.hpp"
#include <cstdint>

namespace Components {

class ICamSink {
 public:
  virtual ~ICamSink() = default;
  virtual void onStateChange(CamState from, CamState to) = 0;
  virtual void onFault(CamFault code, uint32_t detail) = 0;
};

class Esp32CamCore {
 public:
  static constexpr uint8_t kDefaultMissLimit = 3;     // N
  static constexpr uint32_t kDefaultFrameLimit = 20;  // M（100 Hz ticks）
  static constexpr uint32_t kDefaultI2cClockHz = 400000;

  Esp32CamCore();

  void configure(CamModel model, CamRole role, uint8_t missLimit, uint32_t frameLimit);
  void setI2cClockHz(uint32_t hz);
  uint32_t i2cClockHz() const { return m_i2c.clockHz; }
  const CamI2cConfig& i2cConfig() const { return m_i2c; }

  void setLink(ICamLink* link) { m_link = link; }
  void setSink(ICamSink* sink) { m_sink = sink; }

  void tick1Hz();
  void tick100Hz();

  // 回傳 false = EXECUTION_ERROR（狀態不允許）
  bool powerOn();
  bool reset();
  bool captureStart();
  bool captureStop();
  // 過大／壞幀：事件 only，不進 FAULT（CamFault::FRAME_TRUNCATED）
  void notifyFrameTruncated(uint32_t length);
  // 2KB 分塊影像輸出（對齊 CamImageOut）；截斷則不計 frameCount、維持 READY
  void setImageChunkSink(ICamImageChunkSink* sink) { m_streamer.setSink(sink); }
  void setFrameMaxBytes(uint32_t n) { m_streamer.setFrameMaxBytes(n); }
  uint32_t frameMaxBytes() const { return m_streamer.frameMaxBytes(); }
  bool ingestCapturedFrame(const uint8_t* data, uint32_t length);
  uint32_t lastImageChunkCount() const { return m_streamer.lastChunkCount(); }

  CamStatus status() const;
  CamState state() const { return m_state; }
  CamFault fault() const { return m_fault; }
  uint32_t frameCount() const { return m_frameCount; }
  bool captureOn() const { return m_captureOn; }

 private:
  void enter(CamState s);
  void enterFault(CamFault f, uint32_t detail);
  void sendHandshake();
  void checkSelftest();
  bool pullByte(uint8_t* out);
  bool consumeFrameNotify();

  ICamLink* m_link = nullptr;
  ICamSink* m_sink = nullptr;

  CamState m_state = CamState::OFF;
  CamFault m_fault = CamFault::NONE;
  uint32_t m_frameCount = 0;

  CamModel m_model = CamModel::ESP32_CAM;
  CamRole m_role = CamRole::DS_CAM;
  uint8_t m_missLimit = kDefaultMissLimit;
  uint32_t m_frameLimit = kDefaultFrameLimit;
  CamI2cConfig m_i2c{};

  uint8_t m_missCount = 0;
  uint32_t m_noFrameTicks = 0;
  bool m_captureOn = false;
  bool m_selftestMsgSeen = false;
  bool m_selftestFrameSeen = false;
  Esp32CamImageStreamer m_streamer;
};

}  // namespace Components
#endif
