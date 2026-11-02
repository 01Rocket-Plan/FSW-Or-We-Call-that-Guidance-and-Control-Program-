// GnssCore.hpp — TEL0157 GNSS 驅動核心（與 F´ 無關，可主機單測）
//
// =============================================================================
// 設計意圖（為何這樣做）
// =============================================================================
// 1) 閉環不依賴 GNSS（Gate C6）：飛行中失鎖／斷線時驅動「不自己中止」——維持
//    DrvState::READY，持續以較疏的間隔輪詢，並輸出降級 fix（posValid=false）。
//    只有 BUS／SELFTEST 這類「硬體或帶起失敗」才進 FAULT。
// 2) Edward 2026-11-02 失鎖表：重鎖分三段加快→慢，合計 55 s；時鐘從「最後一次
//    合格 GGA」起算（中斷真正開始的時刻），因此 LOSS_NO_GGA_S 的偵測耗時算進
//    REACQ_FAST，避免「偵測 2 s + FAST 15 s」變成 17 s。
// 3) schedIn＝rg100Hz：需要 100 ms poll，但不可在 handler 裡 sleep。用整數 tick
//    分頻（見下）在不 poll 的拍直接 return，每 poll 最多一次 UART read。
// 4) 為何不用 float 秒差排程：t_s - lastPoll 在 0.01 累加下會漂成 ~110 ms，
//    T12 量到 FAST poll 偏少。改為 ticks = POLL_*_MS / rgPeriod_ms。
//
// Gate：只引 C6／E4 ID；原文不進 repo。
// =============================================================================
#pragma once
#include <cstdint>
#include "GnssIfaces.hpp"
namespace Gnss {

// -----------------------------------------------------------------------------
// prmDb／測試可調參數（單位見各欄）。預設對齊 Edward 表與 rg100Hz。
// -----------------------------------------------------------------------------
struct GnssCoreParams {
  // 合格 3D 的 HDOP 上限（無單位比值）。超過則 posValid 不可為 true，並可視為失鎖。
  float hdopMax = 2.5f;

  // --- 整數 tick 分頻 ---
  // rgPeriod_ms：速率組一拍的標稱週期（ms）。schedIn＝rg100Hz → 10。
  // poll_*_ms：各 LockStage 希望的 UART 輪詢間隔（ms）。
  // 實際 periodTicks = poll_*_ms / rgPeriod_ms（至少 1）→ 預設 10／100／200。
  // 用意：與 F´ RateGroup 對齊、可重現、無 float 漂移。
  uint32_t rgPeriod_ms = 10;
  uint32_t pollTracking_ms = 100;   // TRACKING／ACQ：緊密追蹤
  uint32_t pollReacqFast_ms = 100;  // 剛失鎖：仍高頻，利於快速重鎖
  uint32_t pollReacqMed_ms = 1000;  // 中段：省匯流排／CPU
  uint32_t pollReacqSlow_ms = 2000; // 末段／FIX_FAILED：最低負荷續等

  // --- 自 last-good 起算的階段配額（秒）---
  // 邊界 = 15、25、55（fast、fast+med、fast+med+slow），不是「進入本階段後再數」。
  float reacqFastDur_s = 15.f;
  float reacqMedDur_s = 10.f;
  float reacqSlowDur_s = 30.f;

  // 開機 ACQ：尚未進過 TRACKING 的搜星時限（秒）。逾時→一次事件＋FIX_FAILED（A1）。
  float initAcqTimeout_s = 300.f;
  // SELFTEST：上電後須在此時限（秒）內看到任意合法 NMEA，否則 FAULT（帶起失敗）。
  float selftestTimeout_s = 5.f;
  // 無「新 GGA」超過此秒 → 失鎖（含 UART 靜默）。須大於 NMEA 標稱週期（常 1 Hz），
  // 且此偵測時間計入 REACQ_FAST（見 outageAge）。
  float lossNoGga_s = 2.0f;
  // 連續若干次 poll 收到相同 GGA 內容 → 視為接收機卡住（STUCK）→ 失鎖。
  uint32_t stuckPolls = 3;
};

// 每 poll 一次 read 的緩衝上限（位元組）。
// 為何 2048：9600 8N1 ≈ 960 B/s；SLOW 間隔 2 s 若堆積含 GSV 的 NMEA ≈ 1920 B；
// 2048 留餘量，避免 SLOW 階段一次 read 吃不完而落後。規則仍是「一 poll 一次 bus read」。
constexpr uint32_t kPollReadMax = 2048;

// -----------------------------------------------------------------------------
// GnssCore
//
// DrvState（握手／健康，只走遙測）：
//   OFF → start() → INIT →（首 tick）SELFTEST →（合法 NMEA）READY
//   SELFTEST 逾時無句 → FAULT；READY 中 BUS_ERROR → FAULT。
//   FAULT 鎖存至再次 start()（不因新 GGA 自動清，避免 BIT 失敗被下一句洗掉）。
//
// LockStage（Edward 失鎖／重鎖，READY 內）：
//   ACQ          開機搜星；逾時 → FIX_FAILED（不 FAULT）
//   TRACKING     已鎖定；poll 緊；失鎖 → REACQ_FAST
//   REACQ_FAST   last-good 起至 15 s；成功 → TRACKING
//   REACQ_MED    15→25 s
//   REACQ_SLOW   25→55 s
//   FIX_FAILED   ≥55 s 或 ACQ 逾時；發原文一次；仍 SLOW 輪詢，可回 TRACKING
//
// 輸出／seq：每次 onFix 都 ++seq（團隊：有輸出才遞增）。新 GGA → 立刻發；
// 無新 GGA 的重鎖／靜默 → 最多 1 Hz 降級，讓 dataAge 進遙測。
// -----------------------------------------------------------------------------
class GnssCore {
 public:
  GnssCore(IGnssUart& uart, IGnssSink& sink) : m_uart(uart), m_sink(sink) {}
  void setParams(const GnssCoreParams& p) { m_p = p; }
  void start();                 // 重設狀態機，進入 INIT
  void tick(double t_s);        // 每個 rg tick 呼叫；內部分頻決定是否 doPoll
  DrvState state() const { return m_state; }
  LockStage lockStage() const { return m_lock; }
  uint32_t badChecksumCount() const { return m_badCk; }
  uint32_t truncatedCount() const { return m_trunc; }
  uint32_t pollsInStage() const { return m_pollsInStage; }  // 測用：本階段 poll 次數
  uint32_t pollPeriodTicks() const;                         // 測用／文件：目前分頻
  double lastGoodPos_s() const { return m_lastGoodPos_s; }
  bool feedSentence(const char* s, double t_s);             // 測用：直接餵一句
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
  uint32_t m_ticksSincePoll = 0;   // 整數分頻計數（距上次 poll 的 tick 數）
  double m_stageEnter_s = 0.0;
  double m_selftestEnter_s = 0.0;
  double m_acqEnter_s = 0.0;
  double m_lastNewGga_s = -1e9;    // 任一 GGA（含劣質）到達時刻
  double m_lastGoodPos_s = -1e9;   // 最後合格 3D（失鎖表時鐘原點）
  double m_lastEmit_s = -1e9;      // 降級 1 Hz 節流
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
