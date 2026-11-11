module Mcu {

  @ FlightMode 部署角色（SSS ROLE_DS／ROLE_US）
  enum FmRoleId {
    DS = 0
    US = 1
  }

  @ 飛行模式狀態機「殼」。轉移邏輯由 Edward 撰寫（CONTRIBUTING 第 7 條）。
  @ Base ID 0x1400（docs/conventions.md §5）。DsMcu＝role DS；UsMcu＝role US。
  @ 接 rg1kHz，排在 estimator 之後（用這拍的 StateOut）、controller 之前（給這拍的模式）。
  @ 殼的預設行為：DS 永遠 PAD_IDLE、US 永遠 US_WAIT；不分離、不開傘、不寫 NV。
  passive component FlightMode {

    @ rg1kHz
    guarded input port schedIn: Svc.Sched

    @ estimator → 估態（DS）
    guarded input port stateIn: RocketTypes.StateData

    @ powerMonitor／UvState → 欠壓
    guarded input port uvIn: RocketTypes.UvStatePort

    @ → estimator、controller、umbilicalLink、sdLogger、tlmSend、（備用）
    output port modeOut: [6] RocketTypes.ModePort

    @ [FM-IN-2／3] 箭上離散訊號 → 分離面開關、本級著地（#43 DiscreteData）
    guarded input port discreteIn: RocketTypes.DiscreteData

    @ [FM-IN-5] 級間鏈路收（#43 StageLinkPort；US 部署用）
    guarded input port stageLinkIn: RocketTypes.StageLinkPort

    @ [FM-IN-5] 級間鏈路送（#43 StageLinkPort；DS 部署用）
    output port stageLinkOut: RocketTypes.StageLinkPort

    @ [FM-IN-7] → 開傘驅動（#43 ChuteCmdPort；驅動元件 repo 尚無）
    output port chuteCmdOut: RocketTypes.ChuteCmdPort

    @ → actuatorMgr（DS 分離）
    output port sepCmdOut: RocketTypes.SepCmdPort

    @ → nvFlags：分離已下令（set-only 閂鎖）
    output port sepCommandedOut: Power.SepCommanded

    @ → nvFlags：DS 傘已下令（set-only 閂鎖）
    output port dsChuteCommandedOut: Power.DsChuteCommanded

    @ → nvFlags：讀／寫飛行階段（MCU 重啟後恢復用；[FM-20]）
    output port phaseGetOut: Power.PhaseGet
    output port phaseSetOut: Power.PhaseSet

    time get port timeCaller
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut
    telemetry port tlmOut
    event port eventOut
    text event port textEventOut
    param get port prmGetOut
    param set port prmSetOut

    telemetry mode: RocketTypes.FlightModeId
    telemetry role: FmRoleId

    event ModeChanged(fromMode: RocketTypes.FlightModeId, toMode: RocketTypes.FlightModeId) \
      severity activity high format "FlightMode {} → {}"
    event SepCommanded(mask: U8) severity activity high format "FlightMode 下令分離 mask=0x{x}"
    event ChuteDsCommanded() severity activity high format "FlightMode 下令下級開傘"
    event ChuteUsCommanded() severity activity high format "FlightMode 下令上級開傘"
    event ShellOnly() severity warning high format "FlightMode 仍是殼：不會離開 PAD_IDLE／US_WAIT"

    @ 部署角色：DsMcu 設 DS、UsMcu 設 US
    param role: FmRoleId default FmRoleId.DS

    # [FM-P] SSS FlightParams 的門檻參數由 Edward 加（README「FM-P」有完整數字表）
  }

}
