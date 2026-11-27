// ======================================================================
// \title  UsMcuTopology.cpp
// \brief  拓樸建立：F´ v4.4.1 部署範本＋1 kHz／100 Hz／1 Hz 速率群組
// ======================================================================
#include "Deployments/UsMcu/Top/UsMcuTopologyAc.hpp"
#include <Fw/Logger/Logger.hpp>

namespace UsMcu {

// 基準時鐘 1 ms；除數：1 kHz、100 Hz、1 Hz（順序＝Ports_RateGroups）
const Fw::TimeInterval rateGroupInterval(0, 1000);
Svc::RateGroupDriver::DividerSet rateGroupDivisorsSet{{{1, 0}, {10, 0}, {1000, 0}}};

Svc::ActiveRateGroup::ContextArray rg1kHzContext(0);
Svc::ActiveRateGroup::ContextArray rg100HzContext(0);
Svc::ActiveRateGroup::ContextArray rg1HzContext(0);

enum TopologyConstants {
  COMM_PRIORITY = 34,
};

void configureTopology() {
  rateGroupDriver.configure(rateGroupDivisorsSet);
  rg1kHz.configure(rg1kHzContext);
  rg100Hz.configure(rg100HzContext);
  rg1Hz.configure(rg1HzContext);
  FileHandling::prmDb.configure("UsMcuPrmDb.dat");
  flightMode.setDeploymentRole(Mcu::FmRole::US);
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
  // 飛行紀錄器開機即開始收／寫（不靠 GDS START）。CARD_ROLE 設為 US_LOG。
  // 放在 startTasks 之後，EnterReady 事件才進 logger 任務。
  if (!sdLogger.startLogging(Components::SdCardRole::US_LOG)) {
    Fw::Logger::log("UsMcu sdLogger.startLogging failed\n");
  }
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
  Fw::Logger::log("UsMcu sdLogger writeCount=%u dropCount=%u sdBusyCount=%u mediaBytes=%lu\n",
                  sdLogger.writeCount(), sdLogger.dropCount(), sdLogger.sdBusyCount(),
                  static_cast<unsigned long>(sdLogger.media().image().size()));
  tearDownComponents(state);
  deinitComponents(state);
}

}  // namespace UsMcu
