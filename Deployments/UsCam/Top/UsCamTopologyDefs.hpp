// ======================================================================
// \title  UsCamTopologyDefs.hpp
// \brief  拓樸自動碼需要的定義（F´ v4.4.1 部署範本改寫；同 DsMcu／UsMcu）
// ======================================================================
#ifndef DEPLOYMENTS_USCAM_USCAMTOPOLOGYDEFS_HPP
#define DEPLOYMENTS_USCAM_USCAMTOPOLOGYDEFS_HPP

#include "Svc/Subtopologies/CdhCore/PingEntries.hpp"
#include "Svc/Subtopologies/ComCcsds/PingEntries.hpp"
#include "Svc/Subtopologies/FileHandling/PingEntries.hpp"

#include "Svc/Subtopologies/CdhCore/SubtopologyTopologyDefs.hpp"
#include "Svc/Subtopologies/ComCcsds/SubtopologyTopologyDefs.hpp"
#include "Svc/Subtopologies/FileHandling/SubtopologyTopologyDefs.hpp"

#include "Svc/Subtopologies/ComCcsds/Ports_ComPacketQueueEnumAc.hpp"
#include "Svc/Subtopologies/ComCcsds/Ports_ComBufferQueueEnumAc.hpp"

#include "Deployments/UsCam/Top/FppConstantsAc.hpp"

//! 健康 ping：只有主動元件（速率群組）需要
namespace PingEntries {
namespace UsCam_rg100Hz {
enum { WARN = 3, FATAL = 5 };
}
namespace UsCam_rg1Hz {
enum { WARN = 3, FATAL = 5 };
}
}  // namespace PingEntries

namespace UsCam {

struct TopologyState {
  const char* hostname;  //!< GDS 位址（nullptr＝不連）
  U16 port;              //!< GDS 埠
  CdhCore::SubtopologyState cdhCore;
  ComCcsds::SubtopologyState comCcsds;
  FileHandling::SubtopologyState fileHandling;
};

namespace PingEntries = ::PingEntries;
}  // namespace UsCam

#endif
