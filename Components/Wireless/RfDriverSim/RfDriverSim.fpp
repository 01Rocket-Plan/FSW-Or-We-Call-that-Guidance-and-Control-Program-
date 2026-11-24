module Components {

  @ RF 數傳 Sim；Base ID 0x8480（＝0x8400+0x80；首席已核准）
  @ 埠集合與 RfDriver 相同，可替換拓樸實例；不宣告硬體型號／角色。
  @ 不得讀取任何 truth／飛行狀態；掉包率／延遲／BER 由 prm 注入。
  passive component RfDriverSim {

    sync input port dataIn: Svc.ComDataWithContext
    output port dataOut: Svc.ComDataWithContext
    @ F´ v4：通訊狀態埠是 Fw.SuccessCondition（舊 stub 的 Svc.ComStatus 在真 F´ 不存在）
    output port comStatusOut: Fw.SuccessCondition
    output port healthOut: RfHealth

    sync input port run: Svc.Sched

    time get port timeCaller
    param get port prmGetOut
    param set port prmSetOut
    telemetry port tlmOut
    event port logOut
    text event port logTextOut
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut

    param channel: U32 default 0
    param power_dBm: I32 default 0
    param txTimeout_s: F64 default 1.0
    param initDeadline_s: F64 default 2.0
    param selftestDeadline_s: F64 default 2.0
    param readyDeadline_s: F64 default 5.0
    param txTimeoutLimit: U32 default 3

    param dropRate: F64 default 0.0
    @ 上行送達延遲（tick）：上行位元組要過幾個 run tick 才讀得到
    param latencyTicks: U32 default 0
    param ber: F64 default 0.0

    @ LoRa 空中時間模型（issue #60）。預設值與 RfDriver 相同：SF7／125 kHz／4:5 已定案（Edward），前導／標頭／CRC 待 Edward 定
    @ 注意：頻段仍在評估（433 MHz 法規問題），定案後可能換頻段／硬體
    @ SpreadingFactor 6..12（SF6 只能隱式標頭，Semtech §4.1.1.2）
    param loraSf: U8 default 7
    @ 頻寬 Hz：7800／10400／15600／20800／31250／41700／62500／125000／250000／500000
    param loraBw_Hz: U32 default 125000
    @ CodingRate 1..4 ＝ 4/5..4/8
    param loraCr: U8 default 1
    @ PreambleLength（實際前導 = PreambleLength + 4.25 符號，§4.1.1.7）
    param loraPreambleLen: U16 default 8
    @ 顯式標頭
    param loraExplicitHeader: bool default true
    @ 酬載 CRC（預設開，與 RfDriver 一致）
    param loraCrcOn: bool default true
    @ LowDataRateOptimize：0 關、1 開、2 自動（Tsym > 16 ms 開，§4.1.1.6）
    param loraLdro: U8 default 2
    @ 一個 run tick 的 µs；預設 10000（DsMcu 接在 rg100Hz，Issue #60 第 4 項）。改 rate group 要一起改
    param tickPeriod_us: U32 default 10000

    telemetry State: RocketTypes.DrvState
    telemetry TxTimeoutCount: U32
    telemetry TxDroppedCount: U32
    telemetry SentFrames: U32
    telemetry SimDropCount: U32

    event RfState(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high format "RfDriverSim {} -> {}"
    event RfFault(code: U32) severity warning high format "RfDriverSim FAULT code={}"
    event RfTxTimeout(len: U32) severity warning low format "RfDriverSim TX timeout len={}"
  }

}
