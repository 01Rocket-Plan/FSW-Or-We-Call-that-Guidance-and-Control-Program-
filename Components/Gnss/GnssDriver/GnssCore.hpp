// GnssCore.hpp - TEL0157 GNSS 驅動核心 (與 F´ 無關，可主機單測)
// 安裝位置: 依部署 (DS/US) 由 F´ 元件決定。NMEA: GGA/RMC/VTG/GSA。
// 不 sleep、不讀系統時間；每 tick 由呼叫端傳入 F´ 時間 t_s。
#pragma once
#include <cstdint>
#include "GnssIfaces.hpp"  // repo: Components/Gnss/GnssDriver/
namespace Gnss {
struct GnssCoreParams {
  float    hdopMax = 2.5f;           // prmDb: posValid 的 HDOP 門檻
  uint32_t selftestTicks = 5;        // SELFTEST 內需收到合法句子，否則 FAULT
  uint32_t noDataFaultTicks = 5;     // READY 中連續無資料 tick 數 -> FAULT
  uint32_t stuckTicks = 3;           // 相同 GGA 連續重複 tick 數 -> STUCK
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
  // 測試用: 解析單句 (含 $ 與 *hh)
  bool feedSentence(const char* s, double t_s);
 private:
  void setState(DrvState s);
  void feedByte(uint8_t b, double t_s);
  void handleSentence(char* body, double t_s);
  void emitFix(double t_s);
  IGnssUart& m_uart; IGnssSink& m_sink; GnssCoreParams m_p;
  DrvState m_state = DrvState::OFF;
  uint32_t m_stateTicks = 0, m_noDataTicks = 0, m_sameGgaTicks = 0;
  uint32_t m_badCk = 0, m_trunc = 0 /* 截斷只計數+發故障事件，不進 FAULT */, m_seq = 0;
  bool m_gotValidSentence = false, m_newGga = false, m_haveFix = false;
  uint32_t m_lastGgaHash = 0;
  char m_line[96]; uint32_t m_len = 0; bool m_inLine = false;
  GnssFix m_fix{};                   // 最新解析結果
  double m_lastFixT_s = 0.0;
  uint8_t m_gsaMode = 1;             // 1=no,2=2D,3=3D
};
}  // namespace Gnss
