module PadMcu {

  @ rateGroupDriver 輸出埠編號（基準時鐘 1 ms）
  enum Ports_RateGroups {
    rg1kHz
    rg1Hz
  }

  @ 發射架控制板 — SIL 拓樸（Linux）。不上箭（CONTRIBUTING 第 9 條）。
  @ padController 的 ioGet／actOut／radioIn／radioOut／umbilicalIn／umbilicalOut 等驅動 PR 再接
  @ （NRF24L01、臍帶 UART、GPIO／ADC、點火繼電器與夾持）；指令／遙測先走 GDS。
  deployment topology PadMcu {

    import CdhCore.Subtopology
    import ComCcsds.Subtopology
    import FileHandling.Subtopology

    instance chronoTime
    instance rg1kHz
    instance rg1Hz
    instance rateGroupDriver
    instance systemResources
    instance timer
    instance comDriver

    instance padController

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
      rg1kHz.RateGroupMemberOut[0] -> padController.schedIn

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

  }

}
