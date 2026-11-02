// GnssCore.hpp - TEL0157 GNSS 驅動核心 (與 F´ 無關，可主機單測)
// schedIn＝rg100Hz（週期 rgPeriod_ms，預設 10 ms）。UART 只在 poll 時刻讀一次。不 sleep。
//
// Poll 排程：整數 tick 分頻（避免 float 累加漂移）。
//   pollPeriodTicks = poll_*_ms / rgPeriod_ms
//   預設 100／1000／2000 ms → 10／100／200 ticks @ rg100Hz。
//
// Edward 2026-11-02 失鎖表：
//   55 s 自最後合格 GGA 起算；LOSS_NO_GGA_S 計入 FAST；邊界 15／25／55 s。
//   任一段成功 → TRACKING。開機 ACQ 最多 300 s。
// Gate C6：不自己中止；fixOut 只接 tlm/sdLogger。E4：完全逾時＝進 FIX_FAILED。
//
// 假設：
//   A1. 開機 ACQ 逾時 → 一次 INIT_ACQ_TIMEOUT，進 FIX_FAILED（READY、SLOW；可恢復）。
//       BUS_ERROR／SELFTEST_FAIL → FAULT。
//   A2. FIX_FAILED 後 SLOW 輪詢，鎖定成功回 TRACKING。
//   A3. 失鎖＝劣質 GGA／LOSS_NO_GGA_S／STUCK_POLLS。
//   A4. 合格 3D＝FIX_3D∧hdop<=HDOP_MAX；重鎖中 posValid=false。
//   A5. 空 poll 正常；不以單次空讀當失鎖。
//   A6. BUS_ERROR → FAULT；TRUNCATED／BAD_CHECKSUM 只計數。
//
// 輸出／seq：每次發出的 fix（含降級）都 ++seq（團隊：seq 只在有輸出時遞增）。
//   新 GGA → 立刻發一筆；無新 GGA 的重鎖／靜默 → 最多 1 Hz 降級（posValid=false、dataAge↑）。
//
// 讀取緩衝 kPollReadMax=2048（2 s@9600 含 GSV ≈1920 B 餘量）。每 poll 一次 bus read。
#pragma once
#include <cstdint>
#include "GnssIfaces.hpp"
namespace Gnss {
struct GnssCoreParams {
  float hdopMax = 2.5f;
  // Poll 間隔（ms）；ticks = ms / rgPeriod_ms（見檔頭）
  uint32_t rgPeriod_ms = 10;           // schedIn＝rg100Hz → 10 ms
  uint32_t pollTracking_ms = 100;      // → 10 ticks
  uint32_t pollReacqFast_ms = 100;     // → 10 ticks
  uint32_t pollReacqMed_ms = 1000;     // → 100 ticks
  uint32_t pollReacqSlow_ms = 2000;    // → 200 ticks
  // 自 last-good 起算的階段配額（邊界＝15、25、55）
  float reacqFastDur_s = 15.f;
  float reacqMedDur_s = 10.f;
  float reacqSlowDur_s = 30.f;
  float initAcqTimeout_s = 300.f;
  float selftestTimeout_s = 5.f;
  float lossNoGga_s = 2.0f;
  uint32_t stuckPolls = 3;
};

constexpr uint32_t kPollReadMax = 2048;

class GnssCore {
 public:
  GnssCore(IGnssUart& uart, IGnssSink& sink) : m_uart(uart), m_sink(sink) {}
  void setParams(const GnssCoreParams& p) { m_p = p; }
  void start();
  void tick(double t_s);  // 每次 rg tick 呼叫一次；內部整數分頻決定是否 poll
  DrvState state() const { return m_state; }
  LockStage lockStage() const { return m_lock; }
  uint32_t badChecksumCount() const { return m_badCk; }
  uint32_t truncatedCount() const { return m_trunc; }
  uint32_t pollsInStage() const { return m_pollsInStage; }
  uint32_t pollPeriodTicks() const;
  double lastGoodPos_s() const { return m_lastGoodPos_s; }
  bool feedSentence(const char* s, double t_s);
 private:
  void setState(DrvState s);
  void setLock(LockStage s, double t_s);
  uint32_t pollIntervalMs() const;
  void doPoll(double t_s);
  void feedByte(uint8_t b, double t_s);
  void handleSentence(char* body, double t_s);
  void emitFix(double t_s, bool posValid);
  void maybeEmit(double t_s);
  bool qualityOk() const;
  bool detectLoss(double t_s) const;
  void onGoodFix(double t_s);
  void advanceReacq(double t_s);
  double outageAge_s(double t_s) const;
  IGnssUart& m_uart; IGnssSink& m_sink; GnssCoreParams m_p;
  DrvState m_state = DrvState::OFF;
  LockStage m_lock = LockStage::ACQ;
  uint32_t m_ticksSincePoll = 0;
  double m_stageEnter_s = 0.0;
  double m_selftestEnter_s = 0.0;
  double m_acqEnter_s = 0.0;
  double m_lastNewGga_s = -1e9;
  double m_lastGoodPos_s = -1e9;
  double m_lastEmit_s = -1e9;
  uint32_t m_sameGgaPolls = 0, m_lastGgaHash = 0;
  uint32_t m_badCk = 0, m_trunc = 0, m_seq = 0;
  uint32_t m_pollsInStage = 0;
  bool m_gotValidSentence = false, m_newGga = false, m_haveFix = false;
  bool m_gotGgaThisPoll = false, m_fixFailedMsgSent = false;
  bool m_acqTimeoutSent = false;
  bool m_inLine = false;
  char m_line[96]; uint32_t m_len = 0;
  GnssFix m_fix{};
  uint8_t m_gsaMode = 1;
};
}  // namespace Gnss
