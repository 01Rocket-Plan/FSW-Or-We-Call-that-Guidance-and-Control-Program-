module DsMcu {

  @ rateGroupDriver 輸出埠編號（基準時鐘 1 ms）
  enum Ports_RateGroups {
    rg1kHz
    rg100Hz
    rg1Hz
  }

  @ DS 飛控 MCU — SIL 拓樸（Linux）。ESP32 版本在下一個 PR，連線圖相同、只換時鐘與 HAL。
  deployment topology DsMcu {

    # ------------------------------------------------------------ 子拓樸（F´ v4.4.1）
    import CdhCore.Subtopology
    import ComCcsds.Subtopology
    import FileHandling.Subtopology

    # ------------------------------------------------------------ 實例
    instance chronoTime
    instance rg1kHz
    instance rg100Hz
    instance rg1Hz
    instance rateGroupDriver
    instance systemResources
    instance timer
    instance comDriver

    instance platform
    instance silImuFeed
    instance estimator
    instance controller
    instance actuatorMgr
    instance flightMode
    instance baroDriverSim
    instance nvFlags

    # ------------------------------------------------------------ 樣板連線
    command connections instance CdhCore.cmdDisp
    event connections instance CdhCore.events
    telemetry connections instance CdhCore.tlmSend
    text event connections instance CdhCore.textLogger
    health connections instance CdhCore.$health
    param connections instance FileHandling.prmDb
    time connections instance chronoTime

    # ------------------------------------------------------------ F´ 標準連線（照 v4.4.1 部署範本）
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

    # ------------------------------------------------------------ 速率群組
    connections RateGroups {
      timer.CycleOut -> rateGroupDriver.CycleIn

      # 1 kHz：順序就是資料流順序（README「1 kHz 鏈」）
      rateGroupDriver.CycleOut[Ports_RateGroups.rg1kHz] -> rg1kHz.CycleIn
      rg1kHz.RateGroupMemberOut[0] -> silImuFeed.schedIn
      rg1kHz.RateGroupMemberOut[1] -> estimator.schedIn
      rg1kHz.RateGroupMemberOut[2] -> flightMode.schedIn
      rg1kHz.RateGroupMemberOut[3] -> controller.schedIn
      rg1kHz.RateGroupMemberOut[4] -> actuatorMgr.schedIn
      rg1kHz.RateGroupMemberOut[5] -> platform.schedIn

      # 100 Hz
      rateGroupDriver.CycleOut[Ports_RateGroups.rg100Hz] -> rg100Hz.CycleIn
      rg100Hz.RateGroupMemberOut[0] -> baroDriverSim.schedIn

      # 1 Hz
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

    # ------------------------------------------------------------ DS 資料流
    connections DsChain {
      # 感測 → 估態
      silImuFeed.imuOut -> estimator.imuIn
      baroDriverSim.baroOut -> estimator.baroIn

      # 估態 → 控制、模式
      estimator.stateOut[0] -> controller.stateIn
      estimator.stateOut[1] -> flightMode.stateIn

      # 模式廣播
      flightMode.modeOut[0] -> estimator.modeIn
      flightMode.modeOut[1] -> controller.modeIn

      # 控制 → 作動
      controller.finCmdOut -> actuatorMgr.finCmdIn
      flightMode.sepCmdOut -> actuatorMgr.sepCmdIn

      # FlightMode → NV 閂鎖
      flightMode.sepCommandedOut -> nvFlags.sepCommanded
      flightMode.dsChuteCommandedOut -> nvFlags.dsChuteCommanded
      flightMode.phaseGetOut -> nvFlags.phaseGet
      flightMode.phaseSetOut -> nvFlags.phaseSet

      # 待接（其他組模組還不能用真 F´ 編譯，見 README「還沒接的」）：
      #   actuatorMgr.servoCmdOut → servoDriverSim.cmdIn
      #   actuatorMgr.sepCmdOut   → sepServoDriverSim.cmdIn
      #   powerMonitor UvState    → actuatorMgr.uvIn／flightMode.uvIn
      #   flightMode.modeOut[2..] → umbilicalLink／sdLogger／tlmSend
      #   umbilicalLink.padStatusOut → flightMode.padStatusIn（FM-03 架安檢；UmbilicalLink 外殼還沒寫，#37-3）
    }

  }

}
