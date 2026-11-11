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
//   IGNITION ──推力持續 thrustConfirm_s──► RELEASED ──離軌開關 或 臍帶斷──► LIFTED_OFF
//      │  └──點火窗內沒推力──► HANG_FIRE   （夾持保持鎖死、點火關、不重試）
//      └──異常（鑰匙／架電／臍帶／導軌／ABORT）──► HOLD_ABORT（夾持保持鎖死、點火關）
//   RELEASED ──liftoffTimeout_s 內沒離軌──► NO_LIFTOFF（夾持已鬆、不能再夾回、通知人員）
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
};

//! 時間比較的容差：吸收浮點累加誤差和排程抖動（100 Hz 一拍 10 ms 的 5%），不改變門檻本身的意義。
constexpr double kTimeEps_s = 0.0005;

//! 門檻全部是假設【待 Edward 定】，飛前以 HIL／100 m 資料修正。
struct PadParams {
  double countdown_s = 30.0;          // E1：待點火臨界狀態約 30 s
  double ignitionWindow_s = 2.0;      // C15：塔上 2 秒倒數，點火確認後鬆夾持
  // 推力訊號持續多久才算「確認」（防單點雜訊）。2026-11-10 由 0.10 改 0.02（Edward #42 C2）：
  // 主機 CTI 74F85-15A 燃燒只有 0.873 s，0.1 s 等於把最大推力那段留在架上。
  // 100 Hz 下的實際意義：第一個有推力的樣本在 t0，t0＋10 ms、t0＋20 ms 也都有推力（連 3 個樣本），
  // 就在 t0＋20 ms 那一拍鬆夾持。比較時加 kTimeEps_s，避免浮點誤差讓它隨機多等一拍（變 30 ms）。
  double thrustConfirm_s = 0.02;
  // 鬆夾持後多久沒離軌 → NO_LIFTOFF。2026-11-10 由 3.0 改 1.0：74F85 燒 0.873 s，3 s 時發動機早熄了。
  double liftoffTimeout_s = 1.0;
  // 鬆夾持期限：從「第一次看到推力」起算，超過這個時間還沒完成推力確認 → 不鬆了，繼續夾住
  // （HOLD_ABORT／LATE_RELEASE）。Edward 2026-11-11 PR #52 審查提議（拖到 0.05～0.1 s 才鬆，峰值推力已過、
  // 飛不到預定高度，還會影響分離／開傘判斷），建議值 0.04～0.08 s。暫定 0.05 s【待 Edward 定】：
  // 正常 20 ms 確認之外，留 30 ms（3 拍）給推力訊號抖動。小於 thrustConfirm_s 時以 thrustConfirm_s 為準。
  double releaseDeadline_s = 0.05;
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
  bool offRail = false;     // 離軌開關（掛耳離開導軌末端）
  bool thrustOk = false;    // 推力訊號（荷重元超過門檻；感測器型號待定）
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
};

class PadControllerCore {
 public:
  PadControllerCore(IPadCmdAuth* auth, std::uint8_t bootId, IPadEvents* events = nullptr);

  void configure(const PadParams& p) { m_p = p; }
  const PadParams& params() const { return m_p; }

  //! 臍帶 UART 收到的位元組（可分段餵）。
  void onUmbilicalBytes(const std::uint8_t* data, std::size_t len, double now_s);
  //! NRF24 收到一包（一包＝一幀）。回傳指令處理結果。
  PadCmdResult onGroundPacket(const std::uint8_t* data, std::size_t len, double now_s);
  //! 每拍呼叫（建議 100 Hz）。回傳本拍要輸出的點火／夾持命令。
  PadOutputs tick(const PadInputs& in, double now_s);
  //! 組一幀 PadStatus（建議 10 Hz 送 NRF24）。回傳長度。
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
  double m_thrustSince_s = -1.0;
  double m_thrustFirst_s = -1.0;  // IGNITION 中第一次看到推力的時間（不因中斷而重設）
  double m_tRelease_s = 0.0;

  PadInputs m_lastIn{};
  PadOutputs m_lastOut{};
  std::uint16_t m_statusSeq = 0;
  std::uint16_t m_umbilicalSeq = 0;
};

}  // namespace PadLink

#endif
