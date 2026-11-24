/*
 * CmdDispatcherImplCfg.hpp
 *
 *  Created on: May 6, 2015
 *      Author: tcanham
 */

#ifndef CMDDISPATCHER_COMMANDDISPATCHERIMPLCFG_HPP_
#define CMDDISPATCHER_COMMANDDISPATCHERIMPLCFG_HPP_

#include <Fw/FPrimeBasicTypes.hpp>

// Define configuration values for dispatcher
//
// 01RocketPlan 覆寫（Issue #60）：DsMcu 接上無線下行鏈（tlmSend／comQueue／framer／rfDriverSim）後
// 指令＋參數 SET/SAVE opcode 共 180 個，超過 F´ 預設 150 → 開機註冊時 CommandDispatcherImpl.cpp:73 FW_ASSERT。
// 改 256 留餘量。RAM 代價：host 實測每個部署 +5.9 KB；ESP32 估計約 +3 KB。
// 其餘與 F´ v4.4.1 default/config 相同。

enum {
    CMD_DISPATCHER_DISPATCH_TABLE_SIZE = 256,  // !< The size of the table holding opcodes to dispatch
    CMD_DISPATCHER_SEQUENCER_TABLE_SIZE = 25,  // !< The size of the table holding commands in progress
};

namespace Svc {
namespace CmdDispatcherCfg {

//! Include command opcodes in events when true.
//! When false, opcode fields are set to the maximum FwOpcodeType value.
constexpr bool IncludeCommandOpcodesInEvents = true;

constexpr FwOpcodeType getEventOpcode(const FwOpcodeType opcode) {
    return IncludeCommandOpcodesInEvents ? opcode : std::numeric_limits<FwOpcodeType>::max();
}

//! Default for whether a sequence command is dispatched when the sequence tracker table is full.
//! Per-instance override: CommandDispatcherImpl::configure(bool).
constexpr bool EXECUTE_WHEN_SEQUENCE_TABLE_FULL_DEFAULT = false;

}  // namespace CmdDispatcherCfg
}  // namespace Svc

#endif /* CMDDISPATCHER_COMMANDDISPATCHERIMPLCFG_HPP_ */
