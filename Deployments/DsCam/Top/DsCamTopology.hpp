// ======================================================================
// \title  DsCamTopology.hpp
// \brief  拓樸建立／拆除（F´ v4.4.1 部署範本）
// ======================================================================
#ifndef DEPLOYMENTS_DSCAM_DSCAMTOPOLOGY_HPP
#define DEPLOYMENTS_DSCAM_DSCAMTOPOLOGY_HPP

#include "Deployments/DsCam/Top/DsCamTopologyDefs.hpp"

namespace DsCam {

void setupTopology(const TopologyState& state);
void teardownTopology(const TopologyState& state);
//! 以 1 ms 基準時鐘驅動速率群組；阻塞到 stopRateGroups()
void startRateGroups();
void stopRateGroups();

}  // namespace DsCam

#endif
