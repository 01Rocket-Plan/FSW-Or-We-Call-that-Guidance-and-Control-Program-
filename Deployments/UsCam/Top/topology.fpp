module UsCam {

  @ rateGroupDriver 輸出埠編號（基準時鐘 1 ms）
  enum Ports_RateGroups {
    rg100Hz
    rg1Hz
  }

  @ 上級相機獨立板 — SIL 拓樸（Linux）。不併入 UsMcu（CONTRIBUTING 第 9 條）。
  @ usCam.imageOut（2 KB 分塊）→ sdLogger.usImageIn；指令／遙測走 GDS。
  deployment topology UsCam {

    import CdhCore.Subtopology
    import ComCcsds.Subtopology
    import FileHandling.Subtopology

    instance chronoTime
    instance rg100Hz
    instance rg1Hz
    instance rateGroupDriver
    instance systemResources
    instance timer
    instance comDriver

    instance usCam
    instance sdLogger

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

      rateGroupDriver.CycleOut[Ports_RateGroups.rg100Hz] -> rg100Hz.CycleIn
      rg100Hz.RateGroupMemberOut[0] -> usCam.sched100Hz
      rg100Hz.RateGroupMemberOut[1] -> sdLogger.schedIn

      rateGroupDriver.CycleOut[Ports_RateGroups.rg1Hz] -> rg1Hz.CycleIn
      rg1Hz.RateGroupMemberOut[0] -> usCam.sched1Hz
      rg1Hz.RateGroupMemberOut[1] -> CdhCore.tlmSend.Run
      rg1Hz.RateGroupMemberOut[2] -> FileHandling.fileDownlink.Run
      rg1Hz.RateGroupMemberOut[3] -> systemResources.run
      rg1Hz.RateGroupMemberOut[4] -> ComCcsds.comQueue.run
      rg1Hz.RateGroupMemberOut[5] -> ComCcsds.aggregator.timeout
      rg1Hz.RateGroupMemberOut[6] -> CdhCore.cmdDisp.run
      rg1Hz.RateGroupMemberOut[7] -> CdhCore.$health.Run
      rg1Hz.RateGroupMemberOut[8] -> ComCcsds.commsBufferManager.schedIn
    }

    connections Camera {
      usCam.imageOut -> sdLogger.usImageIn
    }

  }

}
