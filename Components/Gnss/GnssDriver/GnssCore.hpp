// GnssCore.hpp - TEL0157 GNSS 驅動核心 (與 F´ 無關，可主機單測)
// 安裝位置: 依部署 (DS/US) 由 F´ 元件決定。NMEA: GGA/RMC/VTG/GSA。
// 不 sleep、不讀系統時間；每 tick 由呼叫端傳入 F´ 時間 t_s。
//
// Gate C6（飛行中失鎖／斷線，見 .local_gate_* 摘要，勿外傳）：
//   - 100 m 要 GNSS；失鎖不准自己中止；重試；FIX_FAILED 只報一次。
//   - FIX_FAILED 只影響紀錄／落點顯示；閉環不依賴 GNSS（fixOut→tlm/sdLogger）。
//   - 本核心：READY 後 UART 靜默／失鎖 → 維持 READY、降級輸出（posValid=false、dataAge↑、
//     FIX_LOST／NO_DATA edge 事件），可選 NODATA_FAULT_TICKS>0 才進 FAULT（預設 0＝永不）。
// 假設（gate 未寫死處）：
//   A1. 接收機內部以自身速率重鎖；驅動以 rg1Hz 輪詢（不在驅動內 sleep 100 ms）。
//   A2. STUCK（相同 GGA 連續重複）視為硬體異常 → 仍進 FAULT。
//   A3. BUS_ERROR／SELFTEST_FAIL → FAULT。
//   A4. dataAge > FIX_STALE_AGE_S 強制 posValid=false（預設 1.5 s）。
//   A5. NODATA_WARN_TICKS 後發一次 NO_DATA；位元組恢復後清除 latch。
#pragma once
#include <cstdint>
#include "GnssIfaces.hpp"  // repo: Components/Gnss/GnssDriver/
namespace Gnss {
struct GnssCoreParams {
  float    hdopMax = 2.5f;           // prmDb: posValid 的 HDOP 門檻
  uint32_t selftestTicks = 5;        // SELFTEST 內需收到合法句子，否則 FAULT
  uint32_t noDataWarnTicks = 2;      // READY 中連續無資料 → 發一次 NO_DATA（不進 FAULT）
  uint32_t noDataFaultTicks = 0;     // 0＝READY 後永不因靜默進 FAULT（C6）；>0 可選地面 BIT
  uint32_t stuckTicks = 3;           // 相同 GGA 連續重複 tick 數 -> STUCK → FAULT
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
  // 測試用: 解析單句 (含 $ 與 *hh)
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
  uint32_t m_badCk = 0, m_trunc = 0 /* 截斷只計數+發故障事件，不進 FAULT */, m_seq = 0;
  bool m_gotValidSentence = false, m_newGga = false, m_haveFix = false;
  bool m_fixLostLatched = false;     // FIX_LOST 已報、待 FIX_RECOVERED
  bool m_noDataLatched = false;      // 本段靜默已報過 NO_DATA
  bool m_hadPosValid = false;        // 曾有合格定位（用來決定是否算「失鎖」）
  uint32_t m_lastGgaHash = 0;
  char m_line[96]; uint32_t m_len = 0; bool m_inLine = false;
  GnssFix m_fix{};                   // 最新解析結果
  double m_lastFixT_s = 0.0;
  uint8_t m_gsaMode = 1;             // 1=no,2=2D,3=3D
};
}  // namespace Gnss
