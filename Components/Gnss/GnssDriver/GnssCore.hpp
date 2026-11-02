// GnssCore.hpp - TEL0157 GNSS 驅動核心 (與 F´ 無關，可主機單測)
// 安裝位置: 依部署 (DS/US) 由 F´ 元件決定。NMEA: GGA/RMC/VTG/GSA。
// 不 sleep、不讀系統時間；每 tick 由呼叫端傳入 F´ 時間 t_s。
//
// Gate C6／E4（見 .local_gate_*，勿外傳；此處只引 ID）：
//   C6: 100 m 要 GNSS；失鎖不准自己中止；重試；FIX_FAILED 只報一次；只影響紀錄／落點。
//   E4: 失鎖依現況重鎖；完全逾時後報錯（事件，不中止）。
//   本核心：READY 後 UART 靜默／失鎖／STUCK → 維持 READY、降級（posValid=false、dataAge↑、edge 事件）。
// 假設（gate 未寫死處）：
//   A1. 接收機內部以自身速率重鎖；驅動以 rg1Hz 輪詢（不在驅動內 sleep 100 ms）。
//   A2. READY 後 STUCK 視同失鎖（降級＋edge）；SELFTEST 中 STUCK → FAULT（帶起／BIT）。
//   A3. BUS_ERROR／SELFTEST_FAIL → FAULT。
//   A4. dataAge > FIX_STALE_AGE_S 或 STUCK 中 → posValid=false（預設 1.5 s）。
//   A5. NODATA_WARN_TICKS 後 NO_DATA；NODATA_TIMEOUT_TICKS 後 NODATA_TIMEOUT（E4，預設 10）；
//       位元組恢復後清除各靜默 latch。NODATA_FAULT_TICKS 預設 0（C6）。
//   A6. SELFTEST：至少 2 tick 且最新 GGA 非重複（sameGgaTicks==0）才進 READY，以便帶起階段能偵測 STUCK。
#pragma once
#include <cstdint>
#include "GnssIfaces.hpp"  // repo: Components/Gnss/GnssDriver/
namespace Gnss {
struct GnssCoreParams {
  float    hdopMax = 2.5f;           // prmDb: posValid 的 HDOP 門檻
  uint32_t selftestTicks = 5;        // SELFTEST 逾時無合法句子 → FAULT
  uint32_t noDataWarnTicks = 2;      // READY 靜默 → 發一次 NO_DATA
  uint32_t noDataTimeoutTicks = 10;  // E4 完全逾時 → 發一次 NODATA_TIMEOUT（維持 READY）；預設 10 s@rg1Hz
  uint32_t noDataFaultTicks = 0;     // 0＝READY 後永不因靜默進 FAULT（C6）；>0 可選地面 BIT
  uint32_t stuckTicks = 3;           // 相同 GGA 連續重複 → STUCK（READY=降級；SELFTEST=FAULT）
  float    fixStaleAge_s = 1.5f;     // dataAge 超過此值 → posValid=false
};
class GnssCore {
 public:
  GnssCore(IGnssUart& uart, IGnssSink& sink) : m_uart(uart), m_sink(sink) {}
  void setParams(const GnssCoreParams& p) { m_p = p; }
  void start();                      // OFF -> INIT
  void tick(double t_s);             // 每 rate-group tick 呼叫一次
  DrvState state() const { return m_state; }
  uint32_t badChecksumCount() const { return m_badCk; }
  uint32_t truncatedCount() const { return m_trunc; }
  uint32_t noDataTicks() const { return m_noDataTicks; }
  bool fixLostLatched() const { return m_fixLostLatched; }
  bool stuckLatched() const { return m_stuckLatched; }
  bool feedSentence(const char* s, double t_s);
 private:
  void setState(DrvState s);
  void feedByte(uint8_t b, double t_s);
  void handleSentence(char* body, double t_s);
  void emitFix(double t_s);
  bool computePosOk(float dataAge_s) const;
  void updateLockEvents(bool posOk);
  IGnssUart& m_uart; IGnssSink& m_sink; GnssCoreParams m_p;
  DrvState m_state = DrvState::OFF;
  uint32_t m_stateTicks = 0, m_noDataTicks = 0, m_sameGgaTicks = 0;
  uint32_t m_badCk = 0, m_trunc = 0, m_seq = 0;
  bool m_gotValidSentence = false, m_newGga = false, m_haveFix = false;
  bool m_fixLostLatched = false;
  bool m_noDataLatched = false;
  bool m_noDataTimeoutLatched = false;
  bool m_stuckLatched = false;
  bool m_hadPosValid = false;
  uint32_t m_lastGgaHash = 0;
  char m_line[96]; uint32_t m_len = 0; bool m_inLine = false;
  GnssFix m_fix{};
  double m_lastFixT_s = 0.0;
  uint8_t m_gsaMode = 1;
};
}  // namespace Gnss
