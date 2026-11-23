// ======================================================================
// \title  BleLinkCore.hpp
// \author 無線組（Viktor）
// \brief  BLE 傳輸層核心：把 RfDriverCore 的 IRfBus（整幀寫／位元組讀）接到 BLE GATT。
//
//   定位：LoRa（RfDriver，0x8400）的「備用傳輸」，硬體＝STM32WB55CGU6 內建 BLE（CPU2／M0+ 跑
//   ST 無線堆疊，CPU1／M4 跑本程式）。上層（Framer／ComQueue／UplinkAuth）完全不變：
//     ComQueue → Framer 幀 → RfDriverCore（同一份，不改）→ BleLinkCore（本檔）→ IBleHal → CPU2
//
//   為什麼要切片：Framer 幀上限 255 B（LoRa FIFO 決定），BLE 一個 GATT notification／write 的
//   資料最多 ATT_MTU − 3 B（MTU 協商前 23 → 20 B）。所以每幀切成多片，每片前面加 1 B 片頭：
//     bit7 FIRST、bit6 LAST、bit5..0 片序號（每方向各自累加，mod 64）
//   一幀只有一片時 FIRST|LAST 同時成立。
//
//   掉線處理（對齊 RfDriver 合約 §4「下行不重傳、丟了就計數」）：
//     - 沒有連線（中心端不在）→ writeFrame 回 0，RfDriverCore 記 TX_FAIL／TxDroppedCount，
//       本層另記 noLinkDrops。不阻塞、不排隊等重連。
//     - CPU2 TX 緩衝滿 → 剩下的片下一拍再送；未送完前新幀回 0（忙，丟）。
//     - 連線中斷 → 送到一半的幀、收到一半的幀都丟掉並計數，不拼接跨連線的資料。
//     - ready()＝CPU2 堆疊活著（不是「有連線」）：BLE 沒人連是常態，不能讓 RfDriverCore 因此進 FAULT。
//       堆疊掛掉（CPU2 無回應）才回 false → readyDeadline_s 後 FAULT（與 LoRa 一致）。
//
//   安全：BLE 鏈路層加密／配對只是選配，上行指令的真正驗證仍是 UplinkAuth（CRC→防重放→HMAC→白名單），
//   本層不碰也不繞過。本層不含任何金鑰；若之後啟用配對，配對資料只從 NVS／外部設定載入。
//
//   純 C++14，不 include 任何 HAL，不 sleep、不讀系統時間；主機可測（test/BleDriverHostTest.cpp）。
// ======================================================================
#ifndef WIRELESS_BLELINKCORE_HPP
#define WIRELESS_BLELINKCORE_HPP

#include <cstdint>

#include "Components/Wireless/RfDriver/RfDriverCore.hpp"

namespace Components {

//! BLE 實體層（對應 HCI LE Set PHY 的 1M／2M；WB55 不支援 LE Coded，見 BLE_FEASIBILITY）
enum class BlePhyMode : uint8_t { LE_1M = 0, LE_2M = 1 };

//! 交給 HAL 的無線設定（全部來自 prmDb；不含金鑰）
struct BleRadioConfig {
  int8_t txPower_dBm = 0;          //!< WB55 可設 −40..+6 dBm（datasheet DS11929）；超出由 clamp 處理
  BlePhyMode phy = BlePhyMode::LE_1M;
  uint16_t attMtu = 247;           //!< 要求的 ATT_MTU（23..247）；實際值看協商
  uint16_t advInterval_ms = 100;   //!< 沒連線時廣播間隔
  uint16_t connInterval_ms = 15;   //!< 期望連線間隔（中心端可不接受）
  uint16_t supervisionTimeout_ms = 2000;  //!< 連線監督逾時（超過沒收到封包就斷）
};

//! STM32WB55 BLE 硬體抽象（CPU1 端）。真機由 MCU 平台用 STM32_WPAN（SHCI／ACI）實作；
//! 測試用 BleHalSim。全部非阻塞：CPU2 事件回呼只把片段放進 HAL 自己的佇列，這裡用輪詢拿。
class IBleHal {
 public:
  virtual ~IBleHal() = default;
  //! 啟動 CPU2 堆疊、註冊 GATT 服務（1 個 notify 特徵＋1 個 write-without-response 特徵）、開始廣播
  virtual bool stackStart(const BleRadioConfig& cfg) = 0;
  //! CPU2 堆疊是否活著（IPCC 有回應、沒有 hard fault 回報）
  virtual bool stackAlive() = 0;
  //! 自測：讀 CPU2 堆疊版本／跑 HCI 讀本機位址等無副作用指令
  virtual bool selftest() = 0;
  //! 目前是否有中心端連著且已開啟 notification
  virtual bool linkUp() = 0;
  //! 協商後 ATT_MTU（沒連線回 23）
  virtual uint16_t attMtu() = 0;
  //! 送一個 GATT notification；回 false＝CPU2 TX 緩衝滿（這片沒送出，稍後再試）
  virtual bool notify(const uint8_t* data, uint16_t len) = 0;
  //! 取一筆中心端寫入的資料（＝一片）；沒有回 0
  virtual int32_t pollWrite(uint8_t* data, uint16_t maxLen) = 0;
};

//! 片頭位元
constexpr uint8_t kBleFragFirst = 0x80u;
constexpr uint8_t kBleFragLast = 0x40u;
constexpr uint8_t kBleFragSeqMask = 0x3Fu;
//! ATT notification／write 的協定開銷（opcode 1 B＋handle 2 B）
constexpr uint16_t kBleAttOverhead = 3u;
constexpr uint16_t kBleAttMtuMin = 23u;
constexpr uint16_t kBleAttMtuMax = 247u;  //!< 247 → 244 B 資料，剛好塞進 251 B LL PDU（DLE）
//! 一幀最大長度（與 RfDriverCore 的 m_frameBuf 一致）
constexpr uint32_t kBleMaxFrame = 512u;

//! 把設定值夾在 WB55 能接受的範圍（DS11929：TX −40..+6 dBm；ATT_MTU 23..247）
BleRadioConfig clampBleConfig(const BleRadioConfig& in);

class BleLinkCore final : public IRfBus {
 public:
  explicit BleLinkCore(IBleHal& hal);

  void configure(const BleRadioConfig& cfg) { m_cfg = clampBleConfig(cfg); }
  const BleRadioConfig& config() const { return m_cfg; }

  // ---- IRfBus（RfDriverCore 呼叫）
  bool init() override;
  int32_t writeFrame(const uint8_t* data, uint32_t len) override;
  int32_t readBytes(uint8_t* data, uint32_t maxLen) override;
  bool selftestLoopback() override;
  bool ready() override;

  //! 每拍呼叫一次（BleDriver 外殼在 RfDriverCore.tick 之前呼叫）：
  //! 追蹤連線變化、續送未送完的片、收片重組。
  void service();

  //! 每片最多帶幾 B 幀資料（目前 MTU）
  uint16_t fragPayload() const;

  // ---- 遙測
  bool linkUp() const { return m_linkUp; }
  uint32_t linkUpCount() const { return m_linkUpCount; }
  uint32_t linkDownCount() const { return m_linkDownCount; }
  uint32_t noLinkDrops() const { return m_noLinkDrops; }
  uint32_t busyDrops() const { return m_busyDrops; }
  uint32_t txAbortedOnLinkLoss() const { return m_txAbortedOnLinkLoss; }
  uint32_t fragmentsSent() const { return m_fragmentsSent; }
  uint32_t framesFullySent() const { return m_framesFullySent; }
  uint32_t rxFrames() const { return m_rxFrames; }
  uint32_t rxFragErrors() const { return m_rxFragErrors; }
  uint32_t rxOverflowDrops() const { return m_rxOverflowDrops; }
  bool txPending() const { return m_txLen > 0; }

 private:
  void flushTx();
  void pollRx();
  void onRxFragment(const uint8_t* d, uint32_t n);
  void dropPartialRx();
  void onLinkChange(bool up);

  IBleHal& m_hal;
  BleRadioConfig m_cfg{};
  bool m_started = false;
  bool m_linkUp = false;

  // TX：目前正在送的幀（一次只放一幀；未送完前新幀算忙）
  uint8_t m_txBuf[kBleMaxFrame]{};
  uint32_t m_txLen = 0;
  uint32_t m_txOff = 0;
  uint8_t m_txSeq = 0;

  // RX：重組中的幀＋已完成幀的位元組 FIFO（給 readBytes）
  uint8_t m_rxAsm[kBleMaxFrame]{};
  uint32_t m_rxAsmLen = 0;
  bool m_rxInFrame = false;
  uint8_t m_rxExpectSeq = 0;
  static constexpr uint32_t kRxFifoCap = 1024u;
  uint8_t m_rxFifo[kRxFifoCap]{};
  uint32_t m_rxFifoRd = 0;
  uint32_t m_rxFifoCount = 0;

  uint32_t m_linkUpCount = 0;
  uint32_t m_linkDownCount = 0;
  uint32_t m_noLinkDrops = 0;
  uint32_t m_busyDrops = 0;
  uint32_t m_txAbortedOnLinkLoss = 0;
  uint32_t m_fragmentsSent = 0;
  uint32_t m_framesFullySent = 0;
  uint32_t m_rxFrames = 0;
  uint32_t m_rxFragErrors = 0;
  uint32_t m_rxOverflowDrops = 0;
};

}  // namespace Components
#endif
