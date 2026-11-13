// ======================================================================
// \title  DsMcuTopology.cpp
// \brief  拓樸建立：F´ v4.4.1 部署範本＋1 kHz／100 Hz／1 Hz 速率群組
// ======================================================================
#include "Deployments/DsMcu/Top/DsMcuTopologyAc.hpp"

namespace DsMcu {

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
  FileHandling::prmDb.configure("DsMcuPrmDb.dat");
  flightMode.setDeploymentRole(Mcu::FmRole::DS);
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

}  // namespace DsMcu
