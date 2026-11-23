// ======================================================================
// \title  BleHalSim.hpp
// \author 無線組（Viktor）
// \brief  STM32WB55 BLE 的 IBleHal 模擬（主機／SIL 用）。不讀任何 truth／飛行狀態。
//
//   模擬重點（都是 prm 可調，數字不是實測，是給測試製造情境用）：
//     - CPU2 堆疊：可注入「啟動失敗」「自測失敗」「堆疊當掉」
//     - 連線：中心端連上／斷線由測試或 prm 控制（linkDropEveryTicks／reconnectAfterTicks）
//     - ATT_MTU：協商後值（預設 247；設 23 可測最小 MTU 的切片）
//     - CPU2 TX 緩衝：每拍最多收幾個 notification（txSlotsPerTick），滿了 notify 回 false
//     - 中心端：收到的 notification 依片頭重組回整幀（peerFrames）；也可模擬中心端寫入片段
// ======================================================================
#ifndef WIRELESS_BLEHALSIM_HPP
#define WIRELESS_BLEHALSIM_HPP

#include <cstdint>

#include "Components/Wireless/BleDriver/BleLinkCore.hpp"

namespace Components {

enum class BleSimFault : uint8_t { NONE = 0, START_FAIL = 1, SELFTEST_FAIL = 2, STACK_DEAD = 3 };

class BleHalSim final : public IBleHal {
 public:
  BleHalSim();

  // ---- 情境注入
  void setFault(BleSimFault f) { m_fault = f; }
  void setLinkUp(bool up);
  void setNegotiatedMtu(uint16_t mtu) { m_mtu = mtu; }
  void setTxSlotsPerTick(uint32_t n) { m_txSlotsPerTick = n; }
  //! >0：連上後每 N 拍自動斷一次；reconnectAfterTicks 拍後自動重連（0＝不自動重連）
  void setAutoLink(uint32_t dropEveryTicks, uint32_t reconnectAfterTicks);
  //! 模擬中心端寫入一片（含片頭）；佇列滿回 false
  bool peerWriteFragment(const uint8_t* data, uint16_t len);
  //! 模擬中心端把一整幀切片寫進來（用與 BleLinkCore 相同的片頭格式）
  bool peerWriteFrame(const uint8_t* data, uint32_t len, uint16_t fragPayload);

  //! 每拍呼叫一次（模擬連線事件：清 TX 緩衝、跑自動斷線／重連）
  void advance();

  // ---- 中心端看到的下行
  uint32_t peerFrames() const { return m_peerFrames; }
  uint32_t peerFragments() const { return m_peerFragments; }
  uint32_t peerFragErrors() const { return m_peerFragErrors; }
  uint32_t lastPeerFrameLen() const { return m_lastPeerFrameLen; }
  const uint8_t* lastPeerFrame() const { return m_lastPeerFrame; }
  uint32_t notifyRejectedFull() const { return m_notifyRejectedFull; }
  uint32_t maxNotifyLen() const { return m_maxNotifyLen; }
  const BleRadioConfig& lastConfig() const { return m_lastCfg; }
  uint32_t startCalls() const { return m_startCalls; }

  // ---- IBleHal
  bool stackStart(const BleRadioConfig& cfg) override;
  bool stackAlive() override;
  bool selftest() override;
  bool linkUp() override;
  uint16_t attMtu() override;
  bool notify(const uint8_t* data, uint16_t len) override;
  int32_t pollWrite(uint8_t* data, uint16_t maxLen) override;

 private:
  static constexpr uint32_t kWriteSlots = 32u;

  BleSimFault m_fault = BleSimFault::NONE;
  bool m_started = false;
  bool m_link = false;
  uint16_t m_mtu = 247u;
  uint32_t m_txSlotsPerTick = 8u;
  uint32_t m_txUsedThisTick = 0u;

  uint32_t m_dropEvery = 0u;
  uint32_t m_reconnectAfter = 0u;
  uint32_t m_linkTicks = 0u;
  uint32_t m_downTicks = 0u;

  // 中心端寫入佇列（每格一片）
  uint8_t m_writeBuf[kWriteSlots][kBleAttMtuMax]{};
  uint16_t m_writeLen[kWriteSlots]{};
  uint32_t m_writeRd = 0u;
  uint32_t m_writeCount = 0u;
  uint8_t m_peerTxSeq = 0u;

  // 中心端重組下行
  uint8_t m_peerAsm[kBleMaxFrame]{};
  uint32_t m_peerAsmLen = 0u;
  bool m_peerInFrame = false;
  uint8_t m_peerExpect = 0u;
  uint8_t m_lastPeerFrame[kBleMaxFrame]{};
  uint32_t m_lastPeerFrameLen = 0u;
  uint32_t m_peerFrames = 0u;
  uint32_t m_peerFragments = 0u;
  uint32_t m_peerFragErrors = 0u;
  uint32_t m_notifyRejectedFull = 0u;
  uint32_t m_maxNotifyLen = 0u;

  BleRadioConfig m_lastCfg{};
  uint32_t m_startCalls = 0u;
};

}  // namespace Components
#endif
