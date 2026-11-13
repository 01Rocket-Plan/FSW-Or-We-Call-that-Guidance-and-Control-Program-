// PadControllerCore：發射架主控（PadMcu，ESP32）— Issue #24
//
// 一顆 Core 同時負責：
//   - 臍帶 UART 收箭體狀態（箭 → 架），監看新鮮度；回送架上狀態（架 → 箭）
//   - NRF24 收地面指令（地面 → 架）、組架上狀態幀（架 → 地面）
//   - 發射許可互鎖、待點火倒數、點火窗、推力確認後鬆夾持、離軌判斷
//
// 狀態（全部轉移寫在 PadControllerCore.cpp 的 tick()／onGroundPacket()）：
//
//   SAFE ──ARM(簽名, 全綠)──► ARMED ──LAUNCH(簽名)──► COUNTDOWN ──倒數到──► IGNITION
//    ▲                         │                        │                    │
//    └──任一紅燈／ABORT／DISARM─┴────────────────────────┘                    │
//                                                                            │
//   IGNITION ──推力持續 thrustConfirm_s──►（IGNITION 內等 releaseMargin_s，推力要一直在）──► RELEASED
//      │  ├──點火窗內沒推力──► HANG_FIRE   （夾持保持鎖死、點火關、不重試）
//      │  ├──等 releaseMargin_s 時推力消失──► HOLD_ABORT／THRUST_LOST（不鬆）
//      │  └──超過 releaseDeadline_s 還沒鬆──► HOLD_ABORT／LATE_RELEASE（不鬆）
//      └──異常（鑰匙／架電／臍帶／導軌／ABORT）──► HOLD_ABORT（夾持保持鎖死、點火關）
//   RELEASED ──離軌判定（三層，見 tick()）──► LIFTED_OFF
//   RELEASED ──clampOpenTimeout_s 內沒讀到夾持開到位──► NO_LIFTOFF／CLAMP_STUCK
//   RELEASED ──liftoffTimeout_s 內沒離軌──► NO_LIFTOFF／NO_LIFTOFF（夾持已鬆、不能再夾回、通知人員）
//
// 離軌判定（鬆夾持後，Edward PR #52 審查）：
//   1. 主判定：離軌開關 offRail，單獨成立就算（未來可能是獨立部署的驅動，這裡只當輸入）。
//   2. 備援：荷重元 load_N 低於 liftoffLoadMin_N 且持續 liftoffLoadHold_s。
//   3. 輔助：臍帶逾時（umbilicalTimeout_s）單獨不算；要和「荷重元低於 liftoffLoadMin_N」同時成立才算。
//   2、3 只在已讀到夾持開到位（clampOpen）後才算：夾持沒開時荷重元掉下來不代表箭走了。
//
// 時序：tick() 跑在 1 kHz 速率群組（一拍 1 ms）；狀態幀（buildStatusFrame／buildUmbilicalFrame）
// 由呼叫端降頻到 10 Hz（每 100 拍一次），Core 本身不假設拍頻。
//
// 規則來源：gate C15（點火只走地面 → 架、硬體鑰匙）、E1（待點火約 30 s 持續評估）、
// E2（BIT 硬擋）、E6（全綠才解鎖）、F9／H2（hold-down：先點、確認推力、再鬆；
// 沒鬆時異常就保持鎖死；鬆了就不能再夾回去）、H4（實體鑰匙）、H6（架歪不准點）、
// SSS 狀態機「點火前發射台安檢」（seated、迴路、許可為 AND、兩道分開的人工指令、
// 一旦撤銷要重走整個解保險流程、點火後 T 秒沒離台 → 安全狀態且不自動重試）。
//
// 一次開機最多點火一次（ignitionUsed）。之後任何 ARM 都拒絕，要重新上電由人處理。
// 被動、不 sleep；now_s 由呼叫端（F´ 時間服務）傳入。
#ifndef WIRELESS_PADCONTROLLERCORE_HPP
#define WIRELESS_PADCONTROLLERCORE_HPP

#include "Components/Wireless/PadLink/PadCmdGate.hpp"
#include "Components/Wireless/PadLink/PadLinkCodec.hpp"

#include <cstddef>
#include <cstdint>

namespace PadLink {

enum class PadState : std::uint8_t {
  SAFE = 0,
  ARMED = 1,
  COUNTDOWN = 2,
  IGNITION = 3,
  RELEASED = 4,
  LIFTED_OFF = 5,
  HANG_FIRE = 6,
  HOLD_ABORT = 7,
  NO_LIFTOFF = 8,
};

enum class PadReason : std::uint8_t {
  NONE = 0,
  CMD_ABORT = 1,
  CMD_DISARM = 2,
  NOT_SEATED = 3,
  NO_CONTINUITY = 4,
  KEY_OFF = 5,
  TILT = 6,
  RAIL_JAM = 7,
  PAD_POWER = 8,
  GROUND_LINK_LOST = 9,
  ROCKET_LINK_LOST = 10,
  ROCKET_NOT_READY = 11,
  NO_THRUST = 12,
  NO_LIFTOFF = 13,
  IGNITION_LOCKED = 14,
  NOT_ARMED = 15,
  BUSY = 16,
  LATE_RELEASE = 17,  // 推力確認太晚（超過 releaseDeadline_s）→ 不鬆、繼續夾住
  THRUST_LOST = 18,   // 推力已確認、等 releaseMargin_s 時推力消失 → 不鬆、繼續夾住
  CLAMP_STUCK = 19,   // 下令鬆夾持後 clampOpenTimeout_s 內沒讀到夾持開到位 → NO_LIFTOFF
  BAD_PARAMS = 20,    // 參數不合理（例如時序加總超過 releaseDeadline_s）→ 拒絕 ARM
};

//! 時間比較的容差：吸收浮點累加誤差和排程抖動（1 kHz 一拍 1 ms 的 5%＝0.05 ms），不改變門檻本身的意義。
constexpr double kTimeEps_s = 0.00005;

//! 「還沒發生」標記：時間戳（開機後秒數）一定 ≥ 0，所以用負數代表「這件事還沒發生、沒有時間」。
//! 例：m_thrustFirst_s＝kNotYet_s 表示這次點火還沒看到任何推力。判斷一律寫成 < 0.0／>= 0.0。
constexpr double kNotYet_s = -1.0;

//! 門檻是無線組自訂的假設值（Edward 2026-11-13：「你們自己訂一個合理數值即可」），飛前以 HIL／100 m 資料修正。
struct PadParams {
  double countdown_s = 30.0;          // E1：待點火臨界狀態約 30 s
  // 點火窗：點火器通電後，多久內要出現「第一個推力樣本」（電點火頭加熱→推進劑起燃的點火延遲；C15 塔上 2 秒）。
  // 這不是推力確認：推力確認從第一個推力樣本起算 thrustConfirm_s（0.02 s），兩個碼錶起點不同。
  double ignitionWindow_s = 2.0;
  // 推力訊號持續多久才算「確認」（防單點雜訊）。2026-11-10 由 0.10 改 0.02（Edward #42 C2）：
  // 主機 CTI 74F85-15A 燃燒只有 0.873 s，0.1 s 等於把最大推力那段留在架上。
  // 1 kHz 下的實際意義：第一個有推力的樣本在 t0，t0＋1 ms … t0＋20 ms 都有推力（連 21 個樣本），
  // 就在 t0＋20 ms 那一拍確認推力。比較時加 kTimeEps_s，避免浮點誤差讓它隨機多等一拍（變 21 ms）。
  double thrustConfirm_s = 0.02;
  // 推力確認後至少再等多久才發鬆夾持命令（Edward PR #52 審查：避免「確認和鬆架同時」的零秒漏洞）。
  // 預設 1e-3 s＝1 kHz 一拍：t0＋20 ms 確認、t0＋21 ms 鬆。等的這段時間每一拍都要有推力，
  // 一拍沒有就 HOLD_ABORT／THRUST_LOST、不鬆。軟體最小單位是一拍，更小的值等同下一拍；0＝確認那拍就鬆。
  double releaseMargin_s = 0.001;
  // 從鬆夾持命令到夾持真正打開的時間（假設 0.02 s，型號定了以後用 HIL 實測）。
  // 只拿來檢查設定：thrustConfirm_s＋releaseMargin_s＋clampActuatorLatency_s 必須 ≤ releaseDeadline_s。
  double clampActuatorLatency_s = 0.02;
  // 鬆夾持後多久沒離軌 → NO_LIFTOFF。2026-11-10 由 3.0 改 1.0：74F85 燒 0.873 s，3 s 時發動機早熄了。
  double liftoffTimeout_s = 1.0;
  // 鬆夾持期限：從「第一次看到推力」起算，超過這個時間還沒完成推力確認 → 不鬆了，繼續夾住
  // （HOLD_ABORT／LATE_RELEASE）。Edward 2026-11-11 PR #52 審查提議（拖到 0.05～0.1 s 才鬆，峰值推力已過、
  // 飛不到預定高度，還會影響分離／開傘判斷），建議值 0.04～0.08 s。定為 0.05 s：
  // 20 ms 確認＋1 ms 裕度＋20 ms 夾持動作＝41 ms，剩 9 ms 給推力訊號抖動。
  double releaseDeadline_s = 0.05;
  // 離軌備援（荷重元，假設硬體）：讀值低於 liftoffLoadMin_N 且持續 liftoffLoadHold_s → 箭已不壓在架上。
  double liftoffLoadMin_N = 0.5;
  double liftoffLoadHold_s = 0.05;
  // 夾持開到位回饋（限位開關，假設硬體）：下令鬆夾持後這段時間內沒讀到 → NO_LIFTOFF／CLAMP_STUCK。
  // ≤ 0 表示這套架沒有裝開到位開關，不做這項檢查（荷重元備援也就不必等開到位）。
  double clampOpenTimeout_s = 0.1;
  double groundLinkTimeout_s = 1.0;   // 多久沒收到簽名幀 → 地面鏈路紅
  double umbilicalTimeout_s = 0.5;    // 多久沒收到箭體狀態 → 臍帶鏈路紅
  double maxTilt_rad = 0.0872665;     // E6：離垂線 5°（＝仰角 85°），單位 rad
  double padVmin_V = 11.0;            // 架側 12 V 繼電器電源下限
};

//! 架上硬體讀值（由 PadMcu 的 GPIO／ADC 驅動每拍提供）。
struct PadInputs {
  bool seated = false;      // 箭體到位開關
  bool continuity = false;  // 點火迴路連續性（只當布林）
  bool keyOn = false;       // 實體鑰匙讀回（真正的斷電在硬體串聯）
  bool railJam = false;     // 導軌卡死旗標
  bool offRail = false;     // 離軌開關（掛耳離開導軌末端；之後可能由獨立部署的驅動提供）
  bool thrustOk = false;    // 推力訊號（可由驅動從荷重元換算；和 load_N 分開當輸入）
  bool clampOpen = false;   // 夾持開到位限位開關（假設硬體）
  double load_N = 0.0;      // 架下荷重元：箭體對架的力（含夾持拉力），單位 N（假設硬體）
  double tilt_rad = 0.0;    // 架離垂線角度
  double padV_V = 0.0;      // 架側電源電壓
};

struct PadOutputs {
  bool igniterOn = false;     // 點火繼電器（軟體這一道；硬體鑰匙串聯在後）
  bool clampRelease = false;  // true＝鬆開夾持；一旦 true 就不會再變回 false
};

class IPadEvents {
 public:
  virtual ~IPadEvents() = default;
  virtual void onStateChange(PadState from, PadState to, PadReason reason) = 0;
  virtual void onCommand(PadCmdOp op, PadCmdResult result, PadReason reason) = 0;
  //! configure() 收到不合理的參數（之後 ARM 會以 BAD_PARAMS 拒絕）。
  virtual void onBadParams(const PadParams& p) { (void)p; }
};

class PadControllerCore {
 public:
  PadControllerCore(IPadCmdAuth* auth, std::uint8_t bootId, IPadEvents* events = nullptr);

  //! 設定參數；不合理（見 paramsValid）時仍會存下來，但 paramsOk()＝false，ARM 以 BAD_PARAMS 拒絕。
  //! 回傳參數是否合理。
  bool configure(const PadParams& p);
  const PadParams& params() const { return m_p; }
  bool paramsOk() const { return m_paramsOk; }
  //! 參數檢查：時間參數有限且不為負、thrustConfirm_s／liftoffTimeout_s＞0，
  //! 且 thrustConfirm_s＋releaseMargin_s＋clampActuatorLatency_s ≤ releaseDeadline_s。
  static bool paramsValid(const PadParams& p);

  //! 臍帶 UART 收到的位元組（可分段餵）。
  void onUmbilicalBytes(const std::uint8_t* data, std::size_t len, double now_s);
  //! NRF24 收到一包（一包＝一幀）。回傳指令處理結果。
  PadCmdResult onGroundPacket(const std::uint8_t* data, std::size_t len, double now_s);
  //! 每拍呼叫（1 kHz 速率群組）。回傳本拍要輸出的點火／夾持命令。
  PadOutputs tick(const PadInputs& in, double now_s);
  //! 組一幀 PadStatus（10 Hz 送 NRF24；呼叫端從 1 kHz 降頻）。回傳長度。
  std::size_t buildStatusFrame(std::uint8_t* out, std::size_t cap);
  //! 組一幀 PadToRocket（建議 10 Hz 寫臍帶 UART）。回傳長度。
  //! 只是狀態回報（方案 A）：鬆夾持仍由架決定，箭只能用 READY 否決。
  std::size_t buildUmbilicalFrame(std::uint8_t* out, std::size_t cap);

  PadState state() const { return m_state; }
  PadReason lastReason() const { return m_reason; }
  std::uint16_t interlocks() const { return m_il; }
  bool ignitionUsed() const { return m_ignitionUsed; }
  double countdownRemaining_s() const { return m_countdownRemaining_s; }
  bool rocketLinkOk() const { return m_rocketFresh; }
  const RocketStatusMsg& lastRocketStatus() const { return m_rocket; }
  std::uint32_t umbilicalCrcErrors() const { return m_deframer.crcErrors(); }
  std::uint32_t groundRejects() const { return m_groundRejects; }

 private:
  std::uint16_t computeInterlocks(const PadInputs& in, double now_s);
  static PadReason firstRed(std::uint16_t il);
  void setState(PadState s, PadReason r);
  PadCmdResult handleCommand(const GroundCmdMsg& cmd, double now_s, PadReason& why);

  PadParams m_p{};
  bool m_paramsOk = true;
  PadCmdGate m_gate;
  IPadEvents* m_events;
  StreamDeframer m_deframer;

  PadState m_state = PadState::SAFE;
  PadReason m_reason = PadReason::NONE;
  std::uint16_t m_il = 0;
  bool m_ignitionUsed = false;

  RocketStatusMsg m_rocket{};
  bool m_haveRocket = false;
  double m_lastRocketRx_s = 0.0;
  bool m_rocketFresh = false;

  bool m_haveGround = false;
  double m_lastGroundRx_s = 0.0;
  std::uint32_t m_groundRejects = 0;
  std::uint32_t m_lastCmdCounter = 0;
  PadCmdResult m_lastCmdResult = PadCmdResult::NONE;

  double m_tCountdown_s = 0.0;
  double m_countdownRemaining_s = 0.0;
  double m_tIgnition_s = 0.0;
  double m_thrustSince_s = kNotYet_s;  // 這一段連續推力的起點（推力中斷就回到 kNotYet_s）
  double m_thrustFirst_s = kNotYet_s;  // IGNITION 中第一次看到推力的時間（不因中斷而重設）
  double m_tConfirm_s = kNotYet_s;      // 推力確認的時間（之後等 releaseMargin_s 才鬆）
  double m_tRelease_s = 0.0;
  bool m_clampOpenSeen = false;    // 鬆夾持後是否讀到過夾持開到位（鎖存）
  double m_loadLowSince_s = kNotYet_s;  // 荷重元連續低於 liftoffLoadMin_N 的起點

  PadInputs m_lastIn{};
  PadOutputs m_lastOut{};
  std::uint16_t m_statusSeq = 0;
  std::uint16_t m_umbilicalSeq = 0;
};

}  // namespace PadLink

#endif
