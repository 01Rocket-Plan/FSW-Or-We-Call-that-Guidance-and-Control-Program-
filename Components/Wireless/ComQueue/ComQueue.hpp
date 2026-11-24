// ======================================================================
// \title  ComQueue.hpp
// \brief  comQueue F´ 外殼（passive；run 掛 Core 0 rg100Hz）。P0..P4 規則全在 ComQueueCore。
//
// comIn[k] ＝ 優先層 Pk（埠號就是層號）；guarded → F´ 元件 mutex 已序列化各 comIn。
// run（sync）另用 m_coreLock 跟 comIn 互斥，再把 Core 吐出的幀「放開鎖之後」才往下游送
// （下游 framer → rfDriver.comStatusOut 會同步回呼 comStatusIn，不可在持鎖時呼叫）。
// 每次 run：先 runEmit 取 1 幀（同時推進丟包節流／高水位），之後每送 1 幀就重看 rfDriver 忙碌狀態，
// 最多 maxPktsPerRun 幀；rfDriver 回報忙碌就停，不會把幀取出來又送不出去。
// ======================================================================
#ifndef COMPONENTS_WIRELESS_COMQUEUE_HPP
#define COMPONENTS_WIRELESS_COMQUEUE_HPP

#include "Components/Wireless/ComQueue/ComQueueComponentAc.hpp"
#include "Components/Wireless/ComQueue/ComQueueCore.hpp"
#include "Os/Mutex.hpp"

namespace Components {

class ComQueue final : public ComQueueComponentBase, private IComQueueEvents {
 public:
  explicit ComQueue(const char* compName);
  ~ComQueue() override = default;

  //! 遙測寫出間隔（rg100Hz 下 100 拍＝1 Hz）
  static constexpr U32 kTlmEveryTicks = 100;

 private:
  // ---- 埠
  void comIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) override;
  void run_handler(FwIndexType portNum, U32 context) override;
  void comStatusIn_handler(FwIndexType portNum, Fw::Success& condition) override;

  // ---- 參數
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;
  void loadParams();

  // ---- Core 事件
  void onQueueFullDrop(ComPriority p, uint32_t dropCount) override;

  void sendFrame();
  void writeTlm();

  ComQueueCore m_core;
  Os::Mutex m_coreLock;
  //! rfDriver 回報的忙碌狀態；開機先當忙碌，等 rfDriver READY 送 SUCCESS
  bool m_linkBusy;
  bool m_paramsLoaded;
  U32 m_tlmDiv;
  //! 下游 framer.comIn 在呼叫內同步處理（全鏈被動）→ 單一暫存即可
  ComFrame m_txFrame;
};

}  // namespace Components

#endif
