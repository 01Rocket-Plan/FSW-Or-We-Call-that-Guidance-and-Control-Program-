module UsMcu {

  @ rateGroupDriver 輸出埠編號（基準時鐘 1 ms）
  enum Ports_RateGroups {
    rg1kHz
    rg100Hz
    rg1Hz
  }

  @ US 飛控 MCU — SIL 拓樸（Linux）。不含 Controller／ActuatorMgr／尾翼 ServoDriver，
  @ 不負責級間分離（gate A9）。只管上級自己的傘、紀錄、相機、雲台。
  deployment topology UsMcu {

    import CdhCore.Subtopology
    import ComCcsds.Subtopology
    import FileHandling.Subtopology

    instance chronoTime
    instance rg1kHz
    instance rg100Hz
    instance rg1Hz
    instance rateGroupDriver
    instance systemResources
    instance timer
    instance comDriver

    instance platform
    instance flightMode
    instance nvFlags

    command connections instance CdhCore.cmdDisp
    event connections instance CdhCore.events
    telemetry connections instance CdhCore.tlmSend
    text event connections instance CdhCore.textLogger
    health connections instance CdhCore.$health
    param connections instance FileHandling.prmDb
    time connections instance chronoTime

    connections ComCcsds_CdhCore {
      CdhCore.events.PktSend -> ComCcsds.comQueue.comPacketQueueIn[ComCcsds.Ports_ComPacketQueue.EVENTS]
      CdhCore.tlmSend.PktSend -> ComCcsds.comQueue.comPacketQueueIn[ComCcsds.Ports_ComPacketQueue.TELEMETRY]
      ComCcsds.fprimeRouter.commandOut -> CdhCore.cmdDisp.seqCmdBuff
      CdhCore.cmdDisp.seqCmdStatus -> ComCcsds.fprimeRouter.cmdResponseIn
    }

    connections ComCcsds_FileHandling {
      FileHandling.fileDownlink.bufferSendOut -> ComCcsds.comQueue.bufferQueueIn[ComCcsds.Ports_ComBufferQueue.FILE]
      ComCcsds.comQueue.bufferReturnOut[ComCcsds.Ports_ComBufferQueue.FILE] -> FileHandling.fileDownlink.bufferReturn
      ComCcsds.fprimeRouter.fileOut -> FileHandling.fileUplink.bufferSendIn
      FileHandling.fileUplink.bufferSendOut -> ComCcsds.fprimeRouter.fileBufferReturnIn
    }

    connections Communications {
      comDriver.allocate -> ComCcsds.commsBufferManager.bufferGetCallee
      comDriver.deallocate -> ComCcsds.commsBufferManager.bufferSendIn
      comDriver.$recv -> ComCcsds.comStub.drvReceiveIn
      ComCcsds.comStub.drvReceiveReturnOut -> comDriver.recvReturnIn
      ComCcsds.comStub.drvSendOut -> comDriver.$send
      comDriver.ready -> ComCcsds.comStub.drvConnected
    }

    connections RateGroups {
      timer.CycleOut -> rateGroupDriver.CycleIn

      rateGroupDriver.CycleOut[Ports_RateGroups.rg1kHz] -> rg1kHz.CycleIn
      rg1kHz.RateGroupMemberOut[0] -> flightMode.schedIn
      rg1kHz.RateGroupMemberOut[1] -> platform.schedIn

      rateGroupDriver.CycleOut[Ports_RateGroups.rg100Hz] -> rg100Hz.CycleIn

      rateGroupDriver.CycleOut[Ports_RateGroups.rg1Hz] -> rg1Hz.CycleIn
      rg1Hz.RateGroupMemberOut[0] -> CdhCore.tlmSend.Run
      rg1Hz.RateGroupMemberOut[1] -> FileHandling.fileDownlink.Run
      rg1Hz.RateGroupMemberOut[2] -> systemResources.run
      rg1Hz.RateGroupMemberOut[3] -> ComCcsds.comQueue.run
      rg1Hz.RateGroupMemberOut[4] -> ComCcsds.aggregator.timeout
      rg1Hz.RateGroupMemberOut[5] -> CdhCore.cmdDisp.run
      rg1Hz.RateGroupMemberOut[6] -> CdhCore.$health.Run
      rg1Hz.RateGroupMemberOut[7] -> ComCcsds.commsBufferManager.schedIn
    }

    connections UsChain {
      # US 不分離：sepCmdOut、sepCommandedOut 不接
      flightMode.phaseGetOut -> nvFlags.phaseGet
      flightMode.phaseSetOut -> nvFlags.phaseSet

      # 待接（見 README「還沒接的」）：
      #   usMpu6050／usDht22 → rg100Hz、sdLogger
      #   gimbalServoDriver（0x3200）
      #   級間鏈路 → flightMode（FM-IN-5）
      #   上級開傘驅動（FM-IN-7）
    }

  }

}
