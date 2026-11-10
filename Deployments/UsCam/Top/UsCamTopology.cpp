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
  //! SIL 幀率：100 Hz 下每 10 拍一幀 ＝ 10 fps
  SIL_FRAME_EVERY_N = 10,
  //! SIL 幀大小：16 KB（8 塊）。一整張 QVGA RGB565 是 153,600 B（75 塊），
  //! 超過 SdLogger 影像佇列預設深度 8 → 會掉塊；兩組要一起定（見 Deployments/UsCam/README.md）。
  SIL_FRAME_BYTES = 16384,
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
