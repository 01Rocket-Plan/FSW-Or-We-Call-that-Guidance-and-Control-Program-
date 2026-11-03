// Ov3640Core：OV3640 相機驅動核心（純邏輯，與 F´ 無關，可在 host 測試）
// 規則：不 sleep、不讀系統時間、不動態配置；時間由呼叫端每 tick 傳入。
// 只接受 1 Hz／100 Hz；每個 tick 工作量有上限（SCCB 批次、最多一幀）。
#ifndef COMPONENTS_CAMERA_OV3640DRIVER_OV3640CORE_HPP
#define COMPONENTS_CAMERA_OV3640DRIVER_OV3640CORE_HPP

#include "CamTypes.hpp"
#include "Ov3640Regs.hpp"
#include <cstdint>

namespace Components {

class IOv3640Sccb {
 public:
  virtual ~IOv3640Sccb() = default;
  virtual bool writeReg(uint16_t reg, uint8_t val) = 0;
  virtual bool readReg(uint16_t reg, uint8_t* val) = 0;
};

class IOv3640FrameSource {
 public:
  virtual ~IOv3640FrameSource() = default;
  virtual bool pollFrame(Ov3640FrameView& out) = 0;
};

class IOv3640Sink {
 public:
  virtual ~IOv3640Sink() = default;
  virtual void onFrame(const uint8_t* data, uint32_t length, uint32_t frameIndex) = 0;
  virtual void onStateChange(CamState from, CamState to) = 0;
  virtual void onFault(CamFault code, uint32_t detail) = 0;
};

struct Ov3640Params {
  uint32_t busTimeoutMisses = 3;
  uint32_t frameTimeoutTicks = 20;
  uint8_t entriesPerTick = 8;
  CamModel model = CamModel::OV3640;
  CamRole role = CamRole::US_CAM;
};

class Ov3640Core {
 public:
  static constexpr CamModel kDeclaredModel = CamModel::OV3640;
  static constexpr CamRole kDeclaredRole = CamRole::US_CAM;

  Ov3640Core();
  ~Ov3640Core();

  void configure(IOv3640Sccb* sccb, IOv3640FrameSource* frameSrc, IOv3640Sink* sink = nullptr);
  void setParams(const Ov3640Params& p);
  const Ov3640Params& params() const { return m_params; }

  bool tick(uint32_t tickHz);

  void start();
  void stop();
  void reset();
  // 僅 READY 回 true；否則 false（呼叫端回 EXECUTION_ERROR）
  bool captureStart();
  void captureStop();
  bool capturing() const { return m_capturing; }

  CamState state() const { return m_status.camState; }
  CamFault fault() const { return m_status.fault; }
  uint32_t frameCount() const { return m_status.frameCount; }
  CamStatus status() const { return m_status; }
  uint16_t lastChipId() const { return m_lastChipId; }
  uint32_t initIndex() const { return m_initIdx; }

 private:
  void enterState(CamState s);
  void enterFault(CamFault f, uint32_t detail);
  bool stepInit();
  bool stepSelftest(uint32_t tickHz);
  bool stepReady(uint32_t tickHz);
  void recordBus(bool ok);

  IOv3640Sccb* m_sccb = nullptr;
  IOv3640FrameSource* m_frameSrc = nullptr;
  IOv3640Sink* m_sink = nullptr;

  Ov3640Params m_params{};
  CamStatus m_status{};

  uint32_t m_initIdx = 0;
  uint32_t m_busMisses = 0;
  uint32_t m_noFrameTicks = 0;
  uint16_t m_lastChipId = 0;
  bool m_idOk = false;
  bool m_testFrameOk = false;
  bool m_capturing = false;
};

}  // namespace Components

#endif
