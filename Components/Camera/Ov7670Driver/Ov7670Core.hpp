// Ov7670Core：OV7670 相機驅動核心（純邏輯；型別用 FPP 自動碼，見 ../CamTypes）
// 規則：不 sleep、不讀系統時間、不動態配置；時間由呼叫端每 tick 傳入。
// 只接受 1 Hz／100 Hz；每個 tick 工作量有上限（SCCB 批次、最多一幀）。
#ifndef COMPONENTS_CAMERA_OV7670DRIVER_OV7670CORE_HPP
#define COMPONENTS_CAMERA_OV7670DRIVER_OV7670CORE_HPP

#include "Components/Camera/CamTypes/CamFaultEnumAc.hpp"
#include "Components/Camera/CamTypes/CamFrameView.hpp"
#include "Components/Camera/CamTypes/CamModelEnumAc.hpp"
#include "Components/Camera/CamTypes/CamRoleEnumAc.hpp"
#include "Components/Camera/CamTypes/CamStateEnumAc.hpp"
#include "Components/Camera/CamTypes/CamStatusSerializableAc.hpp"
#include "Components/Camera/Ov7670Driver/Ov7670Regs.hpp"
#include <cstdint>

namespace Components {

//! SCCB（I2C 相容）暫存器存取：OV7670 是 8-bit 位址
class IOv7670Sccb {
 public:
  virtual ~IOv7670Sccb() = default;
  virtual bool writeReg(uint8_t reg, uint8_t val) = 0;
  virtual bool readReg(uint8_t reg, uint8_t* val) = 0;
};

//! DVP 幀來源（真機：ESP32 平行相機介面；SIL：Ov7670SimDevice）
class IOv7670FrameSource {
 public:
  virtual ~IOv7670FrameSource() = default;
  virtual bool pollFrame(CamFrameView& out) = 0;
};

class IOv7670Sink {
 public:
  virtual ~IOv7670Sink() = default;
  // Stream/accept one logical frame. Return true → count toward frameCount; false → truncated/rejected.
  virtual bool onFrame(const uint8_t* data, uint32_t length, uint32_t frameNumber) = 0;
  virtual void onStateChange(CamState from, CamState to) = 0;
  virtual void onFault(CamFault code, uint32_t detail) = 0;
  // Emitted on each FAULT→INIT auto-retry (attempt in 1..R).
  virtual void onRecoverAttempt(uint32_t attempt, uint32_t limit) = 0;
};

struct Ov7670Params {
  uint32_t busTimeoutMisses = 3;
  uint32_t frameTimeoutTicks = 20;
  uint8_t entriesPerTick = 8;
  CamModel model = CamModel::OV7670;
  CamRole role = CamRole::US_CAM;
  // FAULT auto-recover on 1 Hz: every K ticks retry INIT, up to R times; then latch.
  uint32_t faultRecoverK = 5;
  uint32_t faultRecoverR = 3;
  // Max bytes streamed per logical frame (adapter enforces; Core forwards length).
  uint32_t frameMaxBytes = Ov7670Regs::FRAME_BYTES;
  // 幀率上限：兩次轉存之間至少隔幾個 100 Hz 拍。100＝1 fps（Edward 2026-11-14）；0＝不限（每幀都轉存）。
  uint32_t framePeriodTicks = 100;
};

class Ov7670Core {
 public:
  static constexpr CamModel::T kDeclaredModel = CamModel::OV7670;
  static constexpr CamRole::T kDeclaredRole = CamRole::US_CAM;

  Ov7670Core();
  ~Ov7670Core();

  void configure(IOv7670Sccb* sccb, IOv7670FrameSource* frameSrc, IOv7670Sink* sink = nullptr);
  void setParams(const Ov7670Params& p);
  const Ov7670Params& params() const { return m_params; }

  bool tick(uint32_t tickHz);

  void start();
  void stop();
  void reset();
  bool captureStart();
  void captureStop();
  bool capturing() const { return m_capturing; }

  CamState state() const { return m_state; }
  CamFault fault() const { return m_fault; }
  uint32_t frameCount() const { return m_frameCount; }
  //! 遙測用：由 FPP 自動產生的 CamStatus 組出來
  CamStatus status() const;
  uint16_t lastChipId() const { return m_lastChipId; }
  uint32_t initIndex() const { return m_initIdx; }
  uint32_t recoverAttempts() const { return m_recoverAttempts; }
  bool recoverLatched() const { return m_recoverLatched; }

 private:
  void enterState(CamState::T s);
  void enterFault(CamFault::T f, uint32_t detail);
  bool stepInit();
  bool stepSelftest(uint32_t tickHz);
  bool stepReady(uint32_t tickHz);
  bool stepFault(uint32_t tickHz);
  void recordBus(bool ok);
  void beginInitHandshake();

  IOv7670Sccb* m_sccb = nullptr;
  IOv7670FrameSource* m_frameSrc = nullptr;
  IOv7670Sink* m_sink = nullptr;

  Ov7670Params m_params{};
  CamState m_state = CamState::OFF;
  CamFault m_fault = CamFault::NONE;
  uint32_t m_frameCount = 0;
  CamModel m_model = CamModel::OV7670;
  CamRole m_role = CamRole::US_CAM;

  uint32_t m_initIdx = 0;
  uint32_t m_busMisses = 0;
  uint32_t m_noFrameTicks = 0;
  uint32_t m_sinceSendTicks = 0;  // 距上次轉存幾個 100 Hz 拍（幀率上限用）
  uint16_t m_lastChipId = 0;
  bool m_idOk = false;
  bool m_testFrameOk = false;
  bool m_capturing = false;

  uint32_t m_faultTicks = 0;
  uint32_t m_recoverAttempts = 0;
  bool m_recoverLatched = false;
};

}  // namespace Components

#endif
