// ======================================================================
// \title  UsCamTopology.cpp
// \brief  拓樸建立：F´ v4.4.1 部署範本＋100 Hz／1 Hz 速率群組
// ======================================================================
#include "Deployments/UsCam/Top/UsCamTopologyAc.hpp"

namespace UsCam {

// 基準時鐘 1 ms；除數：100 Hz、1 Hz（順序＝Ports_RateGroups）
const Fw::TimeInterval rateGroupInterval(0, 1000);
Svc::RateGroupDriver::DividerSet rateGroupDivisorsSet{{{10, 0}, {1000, 0}}};

Svc::ActiveRateGroup::ContextArray rg100HzContext(0);
Svc::ActiveRateGroup::ContextArray rg1HzContext(0);

enum TopologyConstants {
  COMM_PRIORITY = 34,
  //! 模擬感測器出幀：100 Hz 下每 10 拍一幀 ＝ 10 fps；驅動的 FRAME_PERIOD_TICKS（100）只轉存 1 fps。
  SIL_FRAME_EVERY_N = 10,
  //! 模擬感測器一幀：QCIF 176×144 YUYV＝50,688 B；驅動取 Y → 灰階 25,344 B（13 塊）。
  //! SdLogger 影像佇列預設 8 塊 → START 後每幀會掉 5 塊（Edward 2026-11-14 已知、接受；見 Deployments/UsCam/README.md）。
  SIL_FRAME_BYTES = 50688,
};

void configureTopology() {
  rateGroupDriver.configure(rateGroupDivisorsSet);
  rg100Hz.configure(rg100HzContext);
  rg1Hz.configure(rg1HzContext);
  FileHandling::prmDb.configure("UsCamPrmDb.dat");
  // SIL 沒有 UsMcu 送 CamLink 指令：開機自動上電＋開始擷取
  usCam.configureSil(true, true, SIL_FRAME_EVERY_N, SIL_FRAME_BYTES);
}

void setupTopology(const TopologyState& state) {
  initComponents(state);
  setBaseIds();
  connectComponents();
  regCommands();
  configComponents(state);
  if (state.hostname != nullptr && state.port != 0) {
    comDriver.configure(state.hostname, state.port);
  }
  configureTopology();
  readParameters();
  loadParameters();
  startTasks(state);
  if (state.hostname != nullptr && state.port != 0) {
    Os::TaskString name("ReceiveTask");
    comDriver.start(name, COMM_PRIORITY, Default::STACK_SIZE);
  }
}

void startRateGroups() { timer.startTimer(rateGroupInterval); }

void stopRateGroups() { timer.quit(); }

void teardownTopology(const TopologyState& state) {
  stopTasks(state);
  freeThreads(state);
  comDriver.stop();
  (void)comDriver.join();
  tearDownComponents(state);
  deinitComponents(state);
}

}  // namespace UsCam
