// ======================================================================
// \title  UsMcuTopology.hpp
// \brief  拓樸建立／拆除（F´ v4.4.1 部署範本）
// ======================================================================
#ifndef DEPLOYMENTS_USMCU_USMCUTOPOLOGY_HPP
#define DEPLOYMENTS_USMCU_USMCUTOPOLOGY_HPP

#include "Deployments/UsMcu/Top/UsMcuTopologyDefs.hpp"

namespace UsMcu {

void setupTopology(const TopologyState& state);
void teardownTopology(const TopologyState& state);
//! 以 1 ms 基準時鐘驅動速率群組；阻塞到 stopRateGroups()
void startRateGroups();
void stopRateGroups();

}  // namespace UsMcu

#endif
