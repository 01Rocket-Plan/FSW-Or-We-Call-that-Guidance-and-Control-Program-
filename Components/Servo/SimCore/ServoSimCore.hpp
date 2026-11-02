// ServoSimCore：ServoDriverSim／SepServoDriverSim 的無框架核心（不需 F´，可 host 測試）
// 規格依伺服組 v0.4（Evgeny，2026-11-01）；介面以 main 4b1d342 RocketTypes.fpp 為準。
// 時間只由 tick() 推進（schedIn 一次一 tick），不 sleep、不讀系統時間。
#ifndef SERVO_SIMCORE_HPP
#define SERVO_SIMCORE_HPP
#include <cstdint>

namespace ServoSim {

enum class DrvState : uint8_t { OFF, INIT, SELFTEST, READY, FAULT };
enum class UvState : uint8_t { NOMINAL, UV_WARN, UV_SHED, UV_CRITICAL };
enum class DropReason : uint8_t { NOT_READY, SEQ_DUP, SEQ_BACKWARD, INVALID };
enum class FaultReason : uint8_t { SELFTEST_FAIL, SELFTEST_TIMEOUT, NV_READ_FAIL };
enum class NeutralSource : uint8_t { CMD, TIMEOUT };

struct ServoCmd { double t_s; uint32_t seq; uint16_t pulse_us[4]; bool lockNeutral; };
struct SepCmd { double t_s; uint32_t seq; uint8_t channelMask; bool valid; };

// 模擬硬體：自測結果可注入
struct ISimHw {
  virtual ~ISimHw() = default;
  // 回傳 true=自測通過；false=尚未完成（逾時由核心判斷）；fail=true 表示明確失敗
  virtual bool selfTestDone(bool& fail) = 0;
};

// ---------------- 尾翼／雲台 ----------------
struct ServoParams {
  uint16_t neutral_us = 1500, pulse_min_us = 1000, pulse_max_us = 2000;
  float cmd_timeout_s = 0.05f, pwm_hz = 50.0f, selftest_deadline_s = 1.0f;
  uint8_t channel_count = 4;
};

struct IServoSink {
  virtual ~IServoSink() = default;
  virtual void pwmOut(uint8_t ch, uint16_t used_us) = 0;   // 實際採用（夾限／回中立後）
  virtual void evEnterReady() = 0;
  virtual void evEnterFault(FaultReason r) = 0;
  virtual void evCmdDropped(uint32_t seq, DropReason r) = 0;
  virtual void evPulseClamped(uint8_t ch, uint16_t raw, uint16_t used) = 0;
  virtual void evCmdTimeout(float age_s) = 0;
  virtual void evLockNeutral(NeutralSource s) = 0;
  virtual void evUvSeen(UvState st) = 0;
};

class ServoSimCore {
 public:
  ServoSimCore(const ServoParams& p, ISimHw& hw, IServoSink& sink) : m_p(p), m_hw(hw), m_sink(sink) {
    for (auto& v : m_pulse) v = p.neutral_us;
  }
  void configure(const ServoParams& p) { if (m_state == DrvState::OFF) { m_p = p; for (auto& v : m_pulse) v = p.neutral_us; } }  // 只在 OFF 生效
  void start();                 // OFF→INIT
  void tick();                  // 每個 schedIn（1/pwm_hz 秒）
  void cmdIn(const ServoCmd& c);
  void uvIn(UvState st);
  // 遙測
  DrvState drvState() const { return m_state; }
  uint32_t lastSeq() const { return m_lastSeq; }
  uint16_t lastPulse_us(uint8_t ch) const { return m_pulse[ch]; }
  float cmdAge_s() const { return m_ageTicks * dt(); }
  UvState uvState() const { return m_uv; }
  uint32_t clampCount() const { return m_clamp; }
  uint32_t dropCount() const { return m_drop; }

 private:
  float dt() const { return 1.0f / m_p.pwm_hz; }
  void allNeutral();
  void emit();
  void enter(DrvState s);
  ServoParams m_p; ISimHw& m_hw; IServoSink& m_sink;
  DrvState m_state = DrvState::OFF;
  uint16_t m_pulse[4];
  uint32_t m_lastSeq = 0; bool m_haveSeq = false;
  uint32_t m_ageTicks = 0, m_stateTicks = 0;
  bool m_timedOut = false;
  UvState m_uv = UvState::NOMINAL;
  uint32_t m_clamp = 0, m_drop = 0;
};

// ---------------- 分離 SEP_1..4 ----------------
struct SepParams {
  uint16_t lock_us = 1000, fire_us = 2000, pulse_min_us = 1000, pulse_max_us = 2000;
  float fire_hold_s = 0.5f, selftest_deadline_s = 1.0f, tick_hz = 50.0f;
};

// 非揮發旗標（電源組 v0.1 暫定）：get 回 mask；set 只能 set 不能清，失敗回 false
struct INvFlags {
  virtual ~INvFlags() = default;
  virtual bool sepReleasedGet(uint8_t& mask) = 0;
  virtual bool sepReleasedSet(uint8_t mask) = 0;
};

struct ISepSink {
  virtual ~ISepSink() = default;
  virtual void pwmOut(uint8_t ch, uint16_t used_us) = 0;
  virtual void evEnterReady() = 0;
  virtual void evEnterFault(FaultReason r) = 0;
  virtual void evCmdDropped(uint32_t seq, DropReason r) = 0;
  virtual void evSepFire(uint8_t ch) = 0;
  virtual void evSepAlreadyFired(uint8_t ch) = 0;
  virtual void evSepFaultHold(uint8_t mask) = 0;
  virtual void evSepNvWriteFail(uint8_t ch) = 0;
  virtual void evUvSeen(UvState st) = 0;
};

class SepSimCore {
 public:
  SepSimCore(const SepParams& p, ISimHw& hw, INvFlags& nv, ISepSink& sink) : m_p(p), m_hw(hw), m_nv(nv), m_sink(sink) {}
  void configure(const SepParams& p) { if (m_state == DrvState::OFF) m_p = p; }  // 只在 OFF 生效
  void start();
  void tick();
  void cmdIn(const SepCmd& c);
  void uvIn(UvState st);
  DrvState drvState() const { return m_state; }
  uint32_t lastSeq() const { return m_lastSeq; }
  uint8_t firedMask() const { return m_fired; }
  uint8_t lockedMask() const { return static_cast<uint8_t>(~m_fired & 0x0F); }
  UvState uvState() const { return m_uv; }
  uint16_t lastPulse_us(uint8_t ch) const { return m_pulse[ch]; }
  uint8_t activeMask() const { uint8_t m = 0; for (uint8_t i = 0; i < 4; i++) if (m_holdTicks[i]) m |= static_cast<uint8_t>(1u << i); return m; }

 private:
  uint16_t clampUs(uint16_t v) const;
  void emit();
  void enter(DrvState s, FaultReason r = FaultReason::SELFTEST_FAIL);
  SepParams m_p; ISimHw& m_hw; INvFlags& m_nv; ISepSink& m_sink;
  DrvState m_state = DrvState::OFF;
  uint16_t m_pulse[4] = {0, 0, 0, 0};
  uint8_t m_fired = 0;
  uint32_t m_holdTicks[4] = {0, 0, 0, 0};
  uint32_t m_lastSeq = 0; bool m_haveSeq = false;
  uint32_t m_stateTicks = 0;
  UvState m_uv = UvState::NOMINAL;
};

}  // namespace ServoSim
#endif
