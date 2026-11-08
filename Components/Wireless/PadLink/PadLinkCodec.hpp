// PadLinkCodec：發射架相關兩條鏈路共用的精簡幀（Issue #24）
//   (1) 箭 ↔ 發射架：有線臍帶 UART（箭 → 架 RocketStatus；架 → 箭 PadToRocket）
//   (2) 發射架 ↔ 地面站：NRF24L01 2.4 GHz（PadStatus／GroundCmd）
//
// 為什麼不沿用 Framer（§2 幀）：Framer 固定開銷 30 B，NRF24L01 單包只有 32 B。
// 本幀固定開銷 8 B，最大酬載 24 B，任何一幀都塞得進一包 NRF24。
//
// 幀佈局（大端）：
//   0  sync0 0xA5
//   1  sync1 0x5A
//   2  type  (PadMsgType)
//   3  seq   U16
//   5  len   U8   (酬載長度 0..kMaxPayload)
//   6  payload[len]
//   6+len  CRC-16/CCITT-FALSE（涵蓋 type..payload）
//
// 被動、不 sleep、不讀系統時間、不配置記憶體。
#ifndef WIRELESS_PADLINKCODEC_HPP
#define WIRELESS_PADLINKCODEC_HPP

#include <cstddef>
#include <cstdint>

namespace PadLink {

static constexpr std::uint8_t kSync0 = 0xA5u;
static constexpr std::uint8_t kSync1 = 0x5Au;
static constexpr std::size_t kHeaderBytes = 6;   // sync×2 + type + seq×2 + len
static constexpr std::size_t kCrcBytes = 2;
static constexpr std::size_t kOverheadBytes = kHeaderBytes + kCrcBytes;  // 8
static constexpr std::size_t kNrf24PayloadBytes = 32;  // NRF24L01 單包上限
static constexpr std::size_t kMaxPayload = kNrf24PayloadBytes - kOverheadBytes;  // 24
static constexpr std::size_t kMaxFrameBytes = kOverheadBytes + kMaxPayload;    // 32

enum class PadMsgType : std::uint8_t {
  ROCKET_STATUS = 0x11,  // 箭 → 架（臍帶）
  PAD_TO_ROCKET = 0x12,  // 架 → 箭（臍帶）
  PAD_STATUS = 0x21,     // 架 → 地面（NRF24）
  GROUND_CMD = 0x31,     // 地面 → 架（NRF24）
};

std::uint16_t crc16Ccitt(const std::uint8_t* data, std::size_t len);

//! 組幀；成功回傳總長度，失敗回傳 0（酬載過長、out 太小、null）。
std::size_t encodeFrame(PadMsgType type, std::uint16_t seq, const std::uint8_t* payload,
                        std::size_t payloadLen, std::uint8_t* out, std::size_t outCap);

struct DecodedFrame {
  PadMsgType type = PadMsgType::ROCKET_STATUS;
  std::uint16_t seq = 0;
  std::uint8_t len = 0;
  std::uint8_t payload[kMaxPayload] = {};
};

//! 解一整包（NRF24 一次收一包時用）；sync／長度／CRC 任一不對回傳 false。
bool decodeFrame(const std::uint8_t* data, std::size_t len, DecodedFrame& out);

//! 位元組串流解幀（臍帶 UART 用）：逐位元組餵入，找 sync、驗 CRC。
//! 壞幀丟棄並計數，然後從下一個位元組重新找 sync（不會卡死）。
class StreamDeframer {
 public:
  //! 回傳 true 表示 out 剛好收到一幀完整且 CRC 正確的資料。
  bool push(std::uint8_t byte, DecodedFrame& out);
  std::uint32_t crcErrors() const { return m_crcErrors; }
  std::uint32_t lenErrors() const { return m_lenErrors; }
  void reset() { m_pos = 0; }

 private:
  std::uint8_t m_buf[kMaxFrameBytes] = {};
  std::size_t m_pos = 0;
  std::uint32_t m_crcErrors = 0;
  std::uint32_t m_lenErrors = 0;
};

// ---------------------------------------------------------------------------
// 訊息（酬載）定義
// ---------------------------------------------------------------------------

//! 箭體 BIT 旗標（gate E2）。HARD＝硬擋，任一沒過箭上就不准報 READY。
enum RocketBitFlag : std::uint16_t {
  BIT_IMU_OK = 1u << 0,            // HARD
  BIT_BATT_V_OK = 1u << 1,         // HARD
  BIT_SERVO_NEUTRAL_OK = 1u << 2,  // HARD
  BIT_BARO_OK = 1u << 3,           // HARD（估態要用）
  BIT_SD_OK = 1u << 8,             // 警告（E2：可在地面手動解鎖）
  BIT_TLM_OK = 1u << 9,            // 警告（E2：可在地面手動解鎖）
};
static constexpr std::uint16_t kRocketBitHardMask =
    BIT_IMU_OK | BIT_BATT_V_OK | BIT_SERVO_NEUTRAL_OK | BIT_BARO_OK;

//! 箭 → 架（臍帶）。7 B。
struct RocketStatusMsg {
  std::uint8_t flightMode = 0;  // RocketTypes.FlightModeId 數值
  std::uint16_t bitFlags = 0;
  bool ready = false;           // 箭上自己判定：硬擋全過 且 仍在架上模式
  std::uint16_t battV_mV = 0;
  std::uint8_t bootCount = 0;
};
static constexpr std::size_t kRocketStatusBytes = 7;
std::size_t packRocketStatus(const RocketStatusMsg& m, std::uint8_t* out, std::size_t cap);
bool unpackRocketStatus(const std::uint8_t* p, std::size_t len, RocketStatusMsg& m);

//! 架 → 箭 旗標（PadToRocket.flags）。
enum PadToRocketFlag : std::uint8_t {
  PTR_IGNITER_ON = 1u << 0,        // 點火繼電器本拍有輸出
  PTR_THRUST_CONFIRMED = 1u << 1,  // 推力已確認（之後就會鬆夾持）
  PTR_CLAMP_RELEASED = 1u << 2,    // 夾持已鬆（不會再夾回）
  PTR_ROCKET_LINK_OK = 1u << 3,    // 架這端有收到箭體狀態（讓箭知道自己的 READY 有被看到）
};

//! 架 → 箭（臍帶）。6 B。
//! 方案 A（Issue #24，Edward 2026-11-08）：鬆夾持由架決定，箭只用 RocketStatus.ready 否決；
//! 這則訊息只是讓箭知道架的狀態（例如倒數中，可預先準備起飛偵測），不是放行指令。
struct PadToRocketMsg {
  std::uint8_t padState = 0;       // PadState
  std::uint8_t lastReason = 0;     // PadReason
  std::uint16_t countdown_ds = 0;  // 倒數剩餘 ×10（0.1 s）
  std::uint8_t flags = 0;          // PadToRocketFlag
  std::uint8_t padBootId = 0;      // 架這次開機的 bootId（箭可藉此發現架重開機）
};
static constexpr std::size_t kPadToRocketBytes = 6;
std::size_t packPadToRocket(const PadToRocketMsg& m, std::uint8_t* out, std::size_t cap);
bool unpackPadToRocket(const std::uint8_t* p, std::size_t len, PadToRocketMsg& m);

//! 架上互鎖位元（架 → 地面狀態燈）。全部為 1＝綠。
enum PadInterlockBit : std::uint16_t {
  IL_SEATED = 1u << 0,          // 箭體到位
  IL_CONTINUITY = 1u << 1,      // 點火迴路連續性（只當布林）
  IL_KEY_ON = 1u << 2,          // 實體鑰匙（讀回值；真正的斷電在硬體）
  IL_TILT_OK = 1u << 3,         // 架傾角在範圍內
  IL_RAIL_FREE = 1u << 4,       // 導軌沒卡死
  IL_PAD_POWER_OK = 1u << 5,    // 架側電源
  IL_GROUND_LINK_OK = 1u << 6,  // 地面鏈路新鮮
  IL_ROCKET_LINK_OK = 1u << 7,  // 臍帶鏈路新鮮
  IL_ROCKET_READY = 1u << 8,    // 箭體回報 READY
  IL_SW_ARMED = 1u << 9,        // 軟體鎖已解（狀態，不是條件）
  IL_CLAMP_LOCKED = 1u << 10,   // 夾持鎖住（狀態）
};
//! 進 ARMED／維持 ARMED、COUNTDOWN 需要全綠的位元。
static constexpr std::uint16_t kPadPermitMask =
    IL_SEATED | IL_CONTINUITY | IL_KEY_ON | IL_TILT_OK | IL_RAIL_FREE | IL_PAD_POWER_OK |
    IL_GROUND_LINK_OK | IL_ROCKET_LINK_OK | IL_ROCKET_READY;

//! 架 → 地面（NRF24）。19 B。
struct PadStatusMsg {
  std::uint8_t padState = 0;     // PadState
  std::uint8_t lastReason = 0;   // PadReason（最近一次退回／拒絕原因）
  std::uint16_t interlocks = 0;  // PadInterlockBit
  std::uint8_t rocketMode = 0;
  std::uint16_t rocketBit = 0;
  std::uint16_t tilt_cdeg = 0;   // 離垂線角度 ×100（只給 UI；度只出現在顯示）
  std::uint16_t padV_mV = 0;
  std::uint16_t countdown_ds = 0;  // 倒數剩餘 ×10（0.1 s）
  std::uint32_t lastCmdCounter = 0;
  std::uint8_t lastCmdResult = 0;  // PadCmdResult
  std::uint8_t bootId = 0;         // 地面簽指令時要帶這個值
};
static constexpr std::size_t kPadStatusBytes = 19;
std::size_t packPadStatus(const PadStatusMsg& m, std::uint8_t* out, std::size_t cap);
bool unpackPadStatus(const std::uint8_t* p, std::size_t len, PadStatusMsg& m);

//! 地面 → 架指令碼。
enum class PadCmdOp : std::uint8_t {
  ARM = 0x01,     // 解軟體鎖（第一道人工指令）
  LAUNCH = 0x02,  // 點火鍵 → 進待點火倒數（第二道人工指令）
  ABORT = 0x03,   // 中止
  DISARM = 0x04,     // 上軟體鎖
  HEARTBEAT = 0x05,  // 地面心跳（要簽名；只有簽過名的幀才算「地面鏈路活著」）
};

static constexpr std::size_t kCmdTagBytes = 16;  // 截短 HMAC-SHA256（128 bit）

//! 地面 → 架（NRF24）。22 B。
struct GroundCmdMsg {
  PadCmdOp op = PadCmdOp::ABORT;
  std::uint32_t counter = 0;
  std::uint8_t bootId = 0;
  std::uint8_t tag[kCmdTagBytes] = {};
};
static constexpr std::size_t kGroundCmdBytes = 22;
std::size_t packGroundCmd(const GroundCmdMsg& m, std::uint8_t* out, std::size_t cap);
bool unpackGroundCmd(const std::uint8_t* p, std::size_t len, GroundCmdMsg& m);

static_assert(kRocketStatusBytes <= kMaxPayload, "RocketStatus must fit one NRF24 packet");
static_assert(kPadToRocketBytes <= kMaxPayload, "PadToRocket must fit one frame");
static_assert(kPadStatusBytes <= kMaxPayload, "PadStatus must fit one NRF24 packet");
static_assert(kGroundCmdBytes <= kMaxPayload, "GroundCmd must fit one NRF24 packet");

}  // namespace PadLink

#endif
